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
        
    # Movement
    vel = 1.0
    max_vel = 1.0
    accel = 1.0
    max_accel = 1.0
    pos = 0.0
    target_pos = 0.0
    min_pos = 0.0
    max_pos = 100.0
    
    # Flow control
    initing = False
    homing = False
    homed = False
    ok = False
    error = False
    busy = False
    moving = False
    movement = MoveDirection.STOPPED
    
    # Device
    motor_type = None
    serial = ""
    serial_c = c_char_p()
    simulate = False
    kinesis_poll = 100  # ms. Less than this is problematic
    lib_prfx = ""
    monitor_interval = 50  # ms
    lib = None
    libok = False
    unit_cal = False
    units = [0, 0, 0]
    units_mult = 100.0
    messageType = c_ushort()
    messageId = c_ushort()
    messageData = c_uint32()
    
    def __init__(self, motor_type: MotorType):
        self.move_loop = None
        self.home_loop = None
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
    
    def __del__(self):
        self.close()
        
    def connect(self, serial="", poll_ms=100, simulate=False):
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
            
        if serial != "":
            # Open the device
            self.kinesis_poll = poll_ms
            if self.libok:
                if self.lib.TLI_BuildDeviceList() == 0:
                    err = eval(f"self.lib.{self.lib_prfx}_Open(self.serial_c)")
                    if err == 0:
                        time.sleep(0.5)
                        if poll_ms >= 10:
                            eval(f"self.lib.{self.lib_prfx}_StartPolling(self.serial_c, c_int(poll_ms))")
                        eval(f"self.lib.{self.lib_prfx}_ClearMessageQueue(self.serial_c)")
                        eval(f"self.lib.{self.lib_prfx}_LoadSettings(self.serial_c)")
                        eval(f"self.lib.{self.lib_prfx}_EnableChannel(self.serial_c)")
                        time.sleep(0.5)
                        eval(f"self.lib.{self.lib_prfx}_RequestSettings(self.serial_c)")      
                        
                        self.ok = True
                    else:
                        print("Error connecting to the device:", err)
                else:
                    print("Error: No devices found")
        
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
            self.units_mult = mult
            real = c_double(mult)
            dev_x = c_int()
            dev_v = c_int()
            dev_a = c_int()
            eval(f"self.lib.{self.lib_prfx}_GetDeviceUnitFromRealValue(self.serial_c, real, byref(dev_x), 0)")
            time.sleep(0.1)
            eval(f"self.lib.{self.lib_prfx}_GetDeviceUnitFromRealValue(self.serial_c, real, byref(dev_v), 1)")
            time.sleep(0.1)
            eval(f"self.lib.{self.lib_prfx}_GetDeviceUnitFromRealValue(self.serial_c, real, byref(dev_a), 2)")
            time.sleep(0.1)
            self.units[0] = dev_x.value
            self.units[1] = dev_v.value
            self.units[2] = dev_a.value
            self.unit_cal = True

    def real2dev(self, real_val, mode=0):
        if self.ok:
            if self.unit_cal:
                if mode > 2: mode = 0
                dev = int(np.round((real_val/self.units_mult)*self.units[mode]))
            else:
                dev = c_int()
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
                real = c_double()
                eval(f"self.lib.{self.lib_prfx}_GetRealValueFromDeviceUnit(self.serial_c, dev, byref(real), mode)")
                real = real.value
            return real
        return 0.0
    
    def get_max_pos(self):
        if self.ok:
            max_dev = eval(f"self.lib.{self.lib_prfx}_GetNumberPositions(self.serial_c)")
            return self.dev2real(max_dev, 0)
        return 0.0
    
    def get_max_vel_params(self):
        if self.ok:
            max_v = c_double()
            max_a = c_double()
            eval(f"self.lib.{self.lib_prfx}_GetMotorVelocityLimits(self.serial_c, byref(max_v), byref(max_a))")
            return max_v.value, max_a.value
        return 0.0, 0.0
    
    def initialize(self):
        self.calibrate_units()
        self.max_pos = self.get_max_pos()
        self.max_vel, self.max_accel = self.get_max_vel_params()
        self.pos = self.get_position()
    
    def get_vel_params(self):
        if self.ok:
            max_vel_dev = c_int()
            accel_dev = c_int()
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
            new_pos_dev = self.real2dev(pos, 0)
            eval(f"self.lib.{self.lib_prfx}_ClearMessageQueue(self.serial_c)")
            
            if stop and self.moving:
                self.stop(stop_mode)

            eval(f"self.lib.{self.lib_prfx}_MoveToPosition(self.serial_c, new_pos_dev)")
            if pos < self.get_position():
                self.movement = self.MoveDirection.NEGATIVE
            else:
                self.movement = self.MoveDirection.POSITIVE
            
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
            
            dist_dev = self.real2dev(dist, 0)
            eval(f"self.lib.{self.lib_prfx}_ClearMessageQueue(self.serial_c)")
            
            if stop and self.moving:
                self.stop(stop_mode)

            eval(f"self.lib.{self.lib_prfx}_MoveRelative(self.serial_c, dist_dev)")
            if dist < 0:
                self.movement = self.MoveDirection.NEGATIVE
            else:
                self.movement = self.MoveDirection.POSITIVE
            
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
            
    
class Brusher(GenericTLMotor):
    """
    Class for the flame brusher motor. Inherits the generic ThorLabs motors class.
    """
    # Default parameters
    vel = 2.5  # mm/s
    accel = 4.5  # mm/s2
    max_vel = vel # mm/s
    max_accel = accel # mm/s2
    serial = "83837733"
    max_pos = 50.0  # mm
    
    def __init__(self):
        super().__init__(MotorType.TCUBE_DCSERVO)

class FlameIO(GenericTLMotor):
    """
    Class for the flame in/out motor. Inherits the generic ThorLabs motors class.
    """
    # Default parameters
    vel = 2.0  # mm/s
    accel = 2.0  # mm/s2
    max_vel = vel # mm/s
    max_accel = accel # mm/s2
    serial = "83837788"
    max_pos = 25.0  # mm
    
    def __init__(self):
        super().__init__(MotorType.TCUBE_DCSERVO)
        
class LeftPuller(GenericTLMotor):
    """
    Class for the left puller. Inherits the generic ThorLabs motors class.
    """
    # Default parameters
    vel = 0.125  # mm/s
    accel = 50.0  # mm/s2
    max_vel = 20.0  # mm/s
    max_accel = accel # mm/s2
    pullers_l_serial = "67838837"
    max_pos = 100.0  # mm
    
    def __init__(self):
        super().__init__(MotorType.TCUBE_BRUSHLESS)
        
class RightPuller(GenericTLMotor):
    """
    Class for the left puller. Inherits the generic ThorLabs motors class.
    """
    # Default parameters
    vel = 0.125  # mm/s
    accel = 50.0  # mm/s2
    max_vel = 20.0  # mm/s
    max_accel = accel # mm/s2
    pullers_l_serial = "67839254"
    max_pos = 100.0  # mm
    
    def __init__(self):
        super().__init__(MotorType.TCUBE_BRUSHLESS)

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
        self.brusher = Brusher()
        self.flame_io = FlameIO()
        self.left_puller = LeftPuller()
        self.right_puller = RightPuller()
        
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
        
    def initialize_motor(self, motor_type: MotorTypes, serial: str="", poll_ms=100, simulate=False):
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
        
        motor.initing = True
        motor.connect(serial, poll_ms, simulate)
        motor.initialize()
        motor.initing = False
            
        return motor.ok