# -*- coding: utf-8 -*-
# =============================================================================
# Created By   : Paulo Jarschel
# First release: May 31, 2024
# Gleb Wataghin Physics Institute (IFGW), University of Campinas
# =============================================================================
"""
This module contains the TaperPullingMotors class, along with the GenericTLMotor
class, inherited by the specific classes for each motor.
They are responsible for controlling the movement of the motors.
Of course, this file is specific to our setup at IFGW. If you are using this
code elsewhere, you may need to change the motors code to adapt to your specific
motors.
"""
# =============================================================================


from enum import Enum
import time
from threading import Timer
from ctypes import *
import os
import numpy as np

        

class MotorType(Enum):
    """
    Enum for motor types
    """
    TCUBE_DCSERVO = 0
    TCUBE_BRUSHLESS = 1

class KinesisDLL:
    lib = None
    motor_type = None
    lib_prfx = ""
    libok = False
    error = False
    
    def __init__(self, motor_type: MotorType):
        self.motor_type = motor_type
            
        try:
            os.add_dll_directory(r"C:\Program Files\Thorlabs\Kinesis")
            if motor_type == MotorType.TCUBE_DCSERVO:
                self.lib_prfx = "CC"
                self.lib: CDLL = cdll.LoadLibrary("Thorlabs.MotionControl.TCube.DCServo.dll")
            elif motor_type == MotorType.TCUBE_BRUSHLESS:
                self.lib_prfx = "BMC"
                self.lib: CDLL = cdll.LoadLibrary("Thorlabs.MotionControl.TCube.BrushlessMotor.dll")
            else:
                raise Exception("Motor type invalid or not implemented.")
            self.libok = True
        except Exception as e:
            print("Error: Thorlabs library not found. See traceback below:")
            print(e)
            self.error = True

# Some interfaces for DLL structures    
class TLI_DeviceInfo(Structure):
    # enum MOT_MotorTypes
    MOT_NotMotor = c_int(0)
    MOT_DCMotor = c_int(1)
    MOT_StepperMotor = c_int(2)
    MOT_BrushlessMotor = c_int(3)
    MOT_CustomMotor = c_int(100)
    MOT_MotorTypes = c_int
    
    _fields_ = [("typeID", c_ulong),
                ("description", (65 * c_char)),
                ("serialNo", (9 * c_char)),
                ("PID", c_ushort),
                ("isKnownType", c_bool),
                ("motorType", MOT_MotorTypes),
                ("isPiezoDevice", c_bool),
                ("isLaser", c_bool),
                ("isCustomType", c_bool),
                ("isRack", c_bool),
                ("maxChannels", c_short)]

class MOT_HomingParameters(Structure):
        # enum MOT_TravelDirection
    MOT_TravelDirectionUndefined = c_short(0x00)
    MOT_Forwards = c_short(0x01)
    MOT_Reverse = c_short(0x02)
    MOT_TravelDirection = c_short

    # enum MOT_HomeLimitSwitchDirection
    MOT_LimitSwitchDirectionUndefined = c_short(0x00)
    MOT_ReverseLimitSwitch = c_short(0x01)
    MOT_ForwardLimitSwitch = c_short(0x04)
    MOT_HomeLimitSwitchDirection = c_short
    
    _fields_ = [("direction", MOT_TravelDirection),
                ("limitSwitch", MOT_HomeLimitSwitchDirection),
                ("velocity", c_uint),
                ("offsetDistance", c_uint)]

class GenericTLMotor:
    """
    A generic class for ThorLabs motors
    """
    
    class MoveDirection(Enum):
        """
        Enum for movement directions
        """
        NEGATIVE = -1
        STOPPED = 0
        POSITIVE = 1
        
    class Loop(Timer):
        """
        Class that manages a threaded loop.
        """
        def run(self):
            while not self.finished.wait(self.interval):
                self.function(*self.args, **self.kwargs)
    
    def __init__(self, motor_type: MotorType, kinesis_dll_lib=None):
        # Movement
        self.vel = 1.0
        self.max_vel = 1.0
        self.accel = 1.0
        self.max_accel = 1.0
        self.pos = 0.0
        self.target_pos = 0.0
        self.min_pos = 0.0
        self.max_pos = 100.0
        
        # Flow control
        self.initing = False
        self.homing = False
        self.homed = False
        self.ok = False
        self.error = False
        self.busy = False
        self.moving = False
        self.movement = self.MoveDirection.STOPPED
        
        # Device
        self.motor_type = None
        self.serial = ""
        self.serial_c = None
        self.simulate = False
        self.kinesis_poll = 200  # ms. Less than this is problematic
        self.lib_prfx = ""
        self.monitor_interval = 200  # ms
        self.lib = None
        self.libok = False
        self.unit_cal = False
        self.units = [0, 0, 0]
        self.units_mult = 100.0
        self.messageType = c_ushort(0)
        self.messageId = c_ushort(0)
        self.messageData = c_uint32(0)
        self.device_info = None
        self.homing_params = None
        self.move_loop = None
        self.home_loop = None
        self.motor_type = motor_type
        
        self.device_info = TLI_DeviceInfo()
        self.homing_params = MOT_HomingParameters()
        
        if kinesis_dll_lib == None:
            try:
                os.add_dll_directory(r"C:\Program Files\Thorlabs\Kinesis")
                if motor_type == MotorType.TCUBE_DCSERVO:
                    self.lib_prfx = "CC"
                    self.lib: CDLL = cdll.LoadLibrary("Thorlabs.MotionControl.TCube.DCServo.dll")
                elif motor_type == MotorType.TCUBE_BRUSHLESS:
                    self.lib_prfx = "BMC"
                    self.lib: CDLL = cdll.LoadLibrary("Thorlabs.MotionControl.TCube.BrushlessMotor.dll")
                else:
                    raise Exception("Motor type invalid or not implemented.")
                self.libok = True
            except Exception as e:
                print("Error: Thorlabs library not found. See traceback below:")
                print(e)
                self.error = True
        else:
            self.motor_type = kinesis_dll_lib.motor_type
            self.lib = kinesis_dll_lib.lib
            self.lib_prfx = kinesis_dll_lib.lib_prfx
            self.libok = kinesis_dll_lib.libok
            self.error = kinesis_dll_lib.error
    
    def __del__(self):
        self.close()
        
    def connect(self, serial="", poll_ms=200, simulate=False):
        """
        Connect to motor
        
        Args:
            serial (str, optional): Serial number to connect. Defaults to "".
            simulate (bool, optional): Simulate this motor. Defaults to False.
        """
        
        self.simulate = simulate
        if serial != "":
            self.serial = serial
            self.serial_c = c_char_p(bytes(serial, "utf-8"))
        if self.simulate:
                self.lib.TLI_InitializeSimulations()

        if self.serial != "":
            # Open the device
            self.kinesis_poll = poll_ms
            self.monitor_interval = poll_ms
            if self.libok:
                if self.lib.TLI_BuildDeviceList() == 0:
                    err = eval(f"self.lib.{self.lib_prfx}_Open(self.serial_c)")
                    if err == 0:
                        time.sleep(1.0)
                        eval(f"self.lib.{self.lib_prfx}_ClearMessageQueue(self.serial_c)")
                        eval(f"self.lib.{self.lib_prfx}_LoadSettings(self.serial_c)")
                        time.sleep(0.5)
                        eval(f"self.lib.{self.lib_prfx}_EnableChannel(self.serial_c)")
                        time.sleep(1.0)
                        eval(f"self.lib.{self.lib_prfx}_RequestSettings(self.serial_c)")      
                        time.sleep(0.5)
                        eval(f"self.lib.{self.lib_prfx}_RequestHomingParams(self.serial_c)")
                        time.sleep(0.5)
                        eval(f"self.lib.{self.lib_prfx}_GetHomingParamsBlock(self.serial_c, byref(self.homing_params))")
                        time.sleep(0.5)
                        # Force homing direction to reverse (towards 0)
                        if self.homing_params.direction != 2:
                            self.homing_params.direction = c_short(0x02)
                        eval(f"self.lib.{self.lib_prfx}_SetHomingParamsBlock(self.serial_c, byref(self.homing_params))")
                        time.sleep(0.5)
                        
                        # Start polling 
                        if poll_ms >= 10:
                            eval(f"self.lib.{self.lib_prfx}_StartPolling(self.serial_c, c_int(poll_ms))")
                        self.ok = True
                    else:
                        print("Error connecting to the device:", err)
                        self.error = True
                else:
                    print("Error: No devices found")
                    self.error = True
        
    def close(self):
        try:
            self.move_loop.cancel()
        except:
            pass
        try:
            self.home_loop.cancel()
        except:
            pass
        
        self.disconnect()
        
    def disconnect(self, disable=False):
        """
        Disconnect from the motor
        """

        if self.ok:
            if disable:
                eval(f"self.lib.{self.lib_prfx}_DisableChannel(self.serial_c)")
            eval(f"self.lib.{self.lib_prfx}_StopPolling(self.serial_c)")
            eval(f"self.lib.{self.lib_prfx}_Close(self.serial_c)")
        
            if self.simulate:
                self.lib.TLI_UninitializeSimulations()
        
        self.ok = False
        
    def get_status_bits(self):
        status_bits = eval(f"self.lib.{self.lib_prfx}_GetStatusBits(self.serial_c)")
        return status_bits
        
    def calibrate_units(self, mult=100.0):
        if self.ok:
            if not self.unit_cal:
                self.units_mult = mult
                self.units = [0, 0, 0]
                real = c_double(mult)
                self.dev_x = c_int(0)
                self.dev_v = c_int(0)
                self.dev_a = c_int(0)

                tries = 0
                while not all(self.units):
                    eval(f"self.lib.{self.lib_prfx}_GetDeviceUnitFromRealValue(self.serial_c, real, byref(self.dev_x), 0)")
                    time.sleep(0.5)
                    eval(f"self.lib.{self.lib_prfx}_GetDeviceUnitFromRealValue(self.serial_c, real, byref(self.dev_v), 1)")
                    time.sleep(0.5)
                    eval(f"self.lib.{self.lib_prfx}_GetDeviceUnitFromRealValue(self.serial_c, real, byref(self.dev_a), 2)")
                    time.sleep(0.5)
                    self.units[0] = self.dev_x.value
                    self.units[1] = self.dev_v.value
                    self.units[2] = self.dev_a.value
                    
                    if not all(self.units):
                        tries += 1
                        time.sleep(1.0)
                    if tries > 10:
                        self.error = True
                        self.ok = False
                        return False
                    
                self.unit_cal = True
        return self.unit_cal

    def real2dev(self, real_val, mode=0):
        if self.ok:
            if self.unit_cal:
                if mode > 2: mode = 0
                dev = int(np.round((real_val/self.units_mult)*self.units[mode]))
            else:
                dev = c_int(0)
                real = c_double(real_val)
                eval(f"self.lib.{self.lib_prfx}_GetDeviceUnitFromRealValue(self.serial_c, real, byref(dev), mode)")
                dev = dev.value
            return dev
        return 0
    
    def dev2real(self, dev_val, mode=0):
        if self.ok:
            if self.unit_cal:
                if mode > 2: mode = 0
                real = float((dev_val)/float(self.units[mode]))*self.units_mult
            else:
                dev = c_int(dev_val)
                real = c_double(0)
                eval(f"self.lib.{self.lib_prfx}_GetRealValueFromDeviceUnit(self.serial_c, dev, byref(real), mode)")
                real = real.value
            return real
        return 0.0
    
    def get_max_pos(self):
        if self.ok:
            max_dev = eval(f"self.lib.{self.lib_prfx}_GetNumberPositions(self.serial_c)")
            return self.dev2real(max_dev, 0)
        return 100.0
    
    def get_max_vel_params(self):
        if self.ok:
            max_v = c_double(0)
            max_a = c_double(0)
            eval(f"self.lib.{self.lib_prfx}_GetMotorVelocityLimits(self.serial_c, byref(max_v), byref(max_a))")
            return max_v.value, max_a.value
        return 1.0, 1.0
    
    def initialize(self):
        self.calibrate_units()
        self.get_homed()
        self.max_pos = self.get_max_pos()
        self.max_vel, self.max_accel = self.get_max_vel_params()
        self.pos = self.get_position()
    
    def get_vel_params(self):
        if self.ok:
            max_vel_dev = c_int(0)
            accel_dev = c_int(0)
            eval(f"self.lib.{self.lib_prfx}_GetVelParams(self.serial_c, byref(accel_dev), byref(max_vel_dev))")
            max_vel_real = self.dev2real(max_vel_dev.value, 1)
            accel_real = self.dev2real(accel_dev.value, 2)
            
            return max_vel_real, accel_real
        return 0.0, 0.0
    
    def set_vel_params(self, vel, accel):
        if self.ok:
            max_vel_dev = c_int(self.real2dev(vel, 1))
            accel_dev = c_int(self.real2dev(accel, 2))
            
            eval(f"self.lib.{self.lib_prfx}_SetVelParams(self.serial_c, accel_dev, max_vel_dev)")
      
    def set_velocity(self, vel: float):
        """
        Set motor velocity. If a value is not provided, it is not changed.

        Args:
            vel (float): Standard velocity.
        """
        if self.ok:
            if vel < 0.1: vel = 0.1
            if vel > self.max_vel: vel = self.max_vel
            curr_v, curr_a = self.get_vel_params()
            curr_v = vel
            self.set_vel_params(curr_v, curr_a)
            self.vel = vel
            
    def set_acceleration(self, accel: float):
        """
        Set motor acceleration. If a value is not provided, it is not changed.

        Args:
            accel (float): Standard acceleration.
        """
        if self.ok:
            if accel < 0.0: accel = 0.0
            if accel > self.max_accel: accel = self.max_accel
            curr_v, curr_a = self.get_vel_params()
            curr_a = accel
            self.set_vel_params(curr_v, curr_a)
            self.accel = accel
            
    def get_velocity(self):
        """
        Get motor velocity.
        """
        if self.ok:
            curr_v, curr_a = self.get_vel_params()
            self.vel = curr_v
            return curr_v
        else:
            return 1e-9
            
    def get_acceleration(self, accel: float):
        """
        Get motor acceleration.
        """
        if self.ok:
            curr_v, curr_a = self.get_vel_params()
            self.accel = curr_a
            return curr_a
        else:
            return 1e-9
        
            
    def get_homed(self) -> bool:
        """
        Get the motor's homing status

        Returns:
            bool: Wheter the motor is homed
        """
        if self.ok:
            # self.homed = bool(eval(f"self.lib.{self.lib_prfx}_CanMoveWithoutHomingFirst(self.serial_c)"))
            # This one is supposedly deprecated, but the other (above) is not found in the DLL ¯\_(ツ)_/¯
            # self.homed = not bool(eval(f"self.lib.{self.lib_prfx}_NeedsHoming(self.serial_c)"))
            
            # In fact, none of these are appropriate. It is best to use the status bits for this.
            status_bits = self.get_status_bits()
            self.homed = bool(status_bits & 0x00000400)
            
        return self.homed
    
    def get_position(self) -> float:
        """
        Get the motor's current position

        Returns:
            float: The position
        """
        if self.ok:
            if self.kinesis_poll < 10:
                eval(f"self.lib.{self.lib_prfx}_RequestPosition(self.serial_c)")
                time.sleep(0.1)
            dev_pos = c_int(eval(f"self.lib.{self.lib_prfx}_GetPosition(self.serial_c)"))
            self.pos = self.dev2real(dev_pos.value, 0)
        return self.pos
    
    def home(self, force=False):
        """
        Perform homing procedure
        """
        if self.ok:           
            self.homed = self.get_homed()
            if not self.homed or force:
                self.stop(0)
                eval(f"self.lib.{self.lib_prfx}_ClearMessageQueue(self.serial_c)")
                eval(f"self.lib.{self.lib_prfx}_Home(self.serial_c)")
                self.start_home_loop()
                
    def home_loop_function(self):
        if not self.busy:
            self.busy = True
            
            done = False
            eval(f"self.lib.{self.lib_prfx}_WaitForMessage(self.serial_c, byref(self.messageType), byref(self.messageId), byref(self.messageData))")
            if (self.messageType.value == 2 and self.messageId.value == 0):
                done = True
            
            if done:
                time.sleep(1.0)
                self.homing = False
                self.homed = self.get_homed()
                self.pos = self.get_position()
                self.home_loop.cancel()
            
            self.busy = False
                
    def wait_for_home(self, to=120):
        t0 = time.time()
        while self.homing:
            t1 = time.time()
            if (t1 - t0) > to:
                return 1
            time.sleep(self.monitor_interval/1000.0)
        return 0
                
    def start_home_loop(self):
        if not self.homing and self.ok:
            self.homing = True
            self.home_loop = None
            self.home_loop = self.Loop(self.monitor_interval/1000.0,self.home_loop_function)
            self.messageType = c_ushort(999)
            self.messageId = c_uint32(999)
            self.messageData = c_uint32(999)
            self.home_loop.start()       
        
    def move_loop_function(self):
        if not self.busy:
            self.busy = True
            
            done = False
            eval(f"self.lib.{self.lib_prfx}_WaitForMessage(self.serial_c, byref(self.messageType), byref(self.messageId), byref(self.messageData))")
            if (self.messageType.value == 2 and self.messageId.value == 1):
                done = True
            
            if done:
                self.moving = False
                self.movement = self.MoveDirection.STOPPED
                self.pos = self.get_position()
                self.move_loop.cancel()
            
            self.busy = False
                
    def start_move_loop(self):
        if (not self.moving) and (not self.homing) and self.ok:
            self.moving = True
            self.move_loop = None
            self.move_loop = self.Loop(self. monitor_interval/1000.0,self.move_loop_function)
            self.messageType = c_ushort(999)
            self.messageId = c_uint32(999)
            self.messageData = c_uint32(999)
            self.move_loop.start()       
            
    def go_to(self, pos:float, stop=True, stop_mode=0):
        """
        Go to position
        
        Args:
            pos (float): target position
        """
        self.target_pos = pos
        if self.ok:
            if pos > self.max_pos:
                pos = self.max_pos
            if pos < 0:
                pos = 0.0
            if pos < self.get_position():
                self.movement = self.MoveDirection.NEGATIVE
            else:
                self.movement = self.MoveDirection.POSITIVE
            
            new_pos_dev = self.real2dev(pos, 0)
            eval(f"self.lib.{self.lib_prfx}_ClearMessageQueue(self.serial_c)")
            
            if stop and self.moving:
                self.stop(stop_mode)

            eval(f"self.lib.{self.lib_prfx}_MoveToPosition(self.serial_c, new_pos_dev)")
            
            self.start_move_loop()        
    
    def move_relative(self, dist, stop=True, stop_mode=0):
        if self.ok:
            pos = self.get_position()
            final = pos + dist

            if final > self.max_pos:
                dist = self.max_pos - pos
            if final < 0:
                dist = -pos
            final = pos + dist
            self.target_pos = final
            
            if dist < 0:
                self.movement = self.MoveDirection.NEGATIVE
            else:
                self.movement = self.MoveDirection.POSITIVE
                
            dist_dev = self.real2dev(dist, 0)
            eval(f"self.lib.{self.lib_prfx}_ClearMessageQueue(self.serial_c)")
            
            if stop and self.moving:
                self.stop(stop_mode)

            eval(f"self.lib.{self.lib_prfx}_MoveRelative(self.serial_c, dist_dev)")
            
            self.start_move_loop()   
    
    def move(self, dir:MoveDirection, stop=True, stop_mode=0):
        """
        Move continuously in a certain direction

        Args:
            dir (MoveDirection): Direction to move
        """
        if self.ok:
            if dir == self.MoveDirection.NEGATIVE:
                dist = -1000
            elif dir == self.MoveDirection.POSITIVE:
                dist = 1000
            else:
                dist = 0.0
            
            self.move_relative(dist, stop, stop_mode)
            
    def wait_for_movement(self, to=60):
        t0 = time.time()
        while self.moving:
            t1 = time.time()
            if (t1 - t0) > to:
                return 1
            time.sleep(self.monitor_interval/1000.0)
        self.movement = self.MoveDirection.STOPPED
        return 0
    
    def stop(self, stop_mode=0):
        """
        Stop movement
        """
        if self.ok:
            try:
                self.move_loop.cancel()
            except:
                pass
            try:
                self.home_loop.cancel()
            except:
                pass
            
            if stop_mode == 1:
                eval(f"self.lib.{self.lib_prfx}_StopProfiled(self.serial_c)")
            else:
                eval(f"self.lib.{self.lib_prfx}_StopImmediate(self.serial_c)")
            self.movement = self.MoveDirection.STOPPED
            self.moving = False
            
    def measure_movement_start_time(self):
        if self.ok:
            pos = self.get_position()
            
            if pos <= 2:
                new_pos = pos + 1.0
            else:
                new_pos = pos - 1.0
            t0 = time.time()
            self.go_to(new_pos)

            prev_pos = pos
            to = 10.0
            t1 = time.time()
            while pos == prev_pos and (t1 - t0) < to:
                prev_pos = pos
                pos = self.get_position()
                t1 = time.time()
                time.sleep(self.monitor_interval/1000.0)
            t1 = time.time()
            move_time = t1 - t0

            return move_time
            
    
class Brusher(GenericTLMotor):
    """
    Class for the flame brusher motor. Inherits the generic ThorLabs motors class.
    """
        
    def __init__(self, kinesis_dll_lib=None):
        super().__init__(MotorType.TCUBE_DCSERVO, kinesis_dll_lib)
        
        # Default parameters
        self.vel = 2.5  # mm/s
        self.accel = 4.5  # mm/s2
        self.max_vel = self.vel # mm/s
        self.max_accel = self.accel # mm/s2
        self.serial = "83837733"
        self.max_pos = 50.0  # mm
        self.units = [3455496, 77296962, 26384]
        self.units_mult = 100.0
        self.unit_cal = True
        self.min_span = 0.1

class FlameIO(GenericTLMotor):
    """
    Class for the flame in/out motor. Inherits the generic ThorLabs motors class.
    """
    
    def __init__(self, kinesis_dll_lib=None):
        super().__init__(MotorType.TCUBE_DCSERVO, kinesis_dll_lib)
        
        # Default parameters
        self.vel = 2.0  # mm/s
        self.accel = 2.0  # mm/s2
        self.max_vel = self.vel # mm/s
        self.max_accel = self.accel # mm/s2
        self.serial = "83837788"
        self.max_pos = 25.0  # mm
        self.units = [3455496, 77296962, 26384]
        self.units_mult = 100.0
        self.unit_cal = True
        
class LeftPuller(GenericTLMotor):
    """
    Class for the left puller. Inherits the generic ThorLabs motors class.
    """
    
    def __init__(self, kinesis_dll_lib=None):
        super().__init__(MotorType.TCUBE_BRUSHLESS, kinesis_dll_lib)
        
        # Default parameters
        self.vel = 40.0  # mm/s
        self.pull_vel = 0.125  # mm/s
        self.accel = 100.0  # mm/s2
        self.max_vel = 500.0  # mm/s
        self.max_accel = 5000.0 # mm/s2
        self.serial = "67838837"
        self.max_pos = 100.0  # mm
        self.units = [200000, 1342157, 137]
        self.units_mult = 100.0
        self.unit_cal = True
        
class RightPuller(GenericTLMotor):
    """
    Class for the left puller. Inherits the generic ThorLabs motors class.
    """
    
    def __init__(self, kinesis_dll_lib=None):
        super().__init__(MotorType.TCUBE_BRUSHLESS, kinesis_dll_lib)
        
        # Default parameters
        self.vel = 40.0  # mm/s
        self.pull_vel = 0.125  # mm/s
        self.accel = 100.0  # mm/s2
        self.max_vel = 500.0  # mm/s
        self.max_accel = 5000.0 # mm/s2
        self.serial = "67839254"
        self.max_pos = 100.0  # mm
        self.units = [200000, 1342157, 137]
        self.units_mult = 100.0
        self.unit_cal = True

class TaperPullingMotors:
    """
    This class contains the motors relevant to the taper pulling process.
    """
    
    class MotorTypes(Enum):
        """
        Enum for motor types (specific to our setup)
        """
        BRUSHER = 0
        FLAME_IO = 1
        LEFT_PULLER = 2
        RIGHT_PULLER = 3
    
    def __init__(self):
        """
        This class contains the motors relevant to the taper pulling process.
        Initialization does not connect to any device.
        """
        self.lib_tdc = KinesisDLL(MotorType.TCUBE_DCSERVO)
        self.lib_tbm = KinesisDLL(MotorType.TCUBE_BRUSHLESS)
        self.brusher = Brusher(self.lib_tdc)
        self.flame_io = FlameIO(self.lib_tdc)
        self.left_puller = LeftPuller(self.lib_tbm)
        self.right_puller = RightPuller(self.lib_tbm)
        
    def __del__(self):
        self.close()
        
    def close(self):
        """
        Shutdown procedures
        """
        self.brusher.close()
        self.flame_io.close()
        self.left_puller.close()
        self.right_puller.close()
        
    def initialize_motor(self, motor_type: MotorTypes, serial: str="", poll_ms=50, simulate=False):
        """
        Connect to, configure, and home (if needed) a motor.
        If a parameter is not given, its current value (default at initialization) is used

        Args:
            motor_type (MotorTypes): Type of the motor to be initialized
            serial (str): Serial number. Defaults to empty.
            simulate (bool, optional): Simulate this motor. Defaults to False.
        """
        
        motor = None
        match motor_type:
            case self.MotorTypes.BRUSHER:
                motor = self.brusher
            case self.MotorTypes.FLAME_IO: 
                motor = self.flame_io
            case self.MotorTypes.LEFT_PULLER:
                motor = self.left_puller
            case self.MotorTypes.RIGHT_PULLER:
                motor = self.right_puller
            case _:  # Default case. Does nothing if a valid motor type is not specified.
                return 0
            
        if serial == "":
            serial = motor.serial
        
        print(f"Initializing {motor_type.name}...")
        motor.initing = True
        motor.connect(serial, poll_ms, simulate)
        motor.initialize()
        motor.set_velocity(motor.vel)
        motor.set_acceleration(motor.accel)
        motor.initing = False
        print(f"{motor_type.name} initialization OK: {motor.ok}")
            
        return motor.ok