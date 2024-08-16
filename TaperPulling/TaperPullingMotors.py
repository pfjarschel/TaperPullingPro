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
        
    vel = 1.0
    max_vel = 1.0
    accel = 1.0
    max_accel = 1.0
    pos = 0.0
    initing = False
    homing = False
    homed = False
    connected = False
    ok = False
    error = False
    serial = ""
    moving = MoveDirection.STOPPED
    simulate = False
    sim_last_t = 0.0
    sim_last_pos = 0.0
    
    def __init__(self):
        pass
    
    def __del__(self):
        pass
    
    def connect(self, serial="", simulate=False):
        """
        Connect to motor
        
        Args:
            serial (str, optional): Serial number to connect. Defaults to "".
            simulate (bool, optional): Simulate this motor. Defaults to False.
        """
        self.simulate = simulate
        if serial != "":
            self.serial = serial
        if serial != "" and not simulate:
            # TODO: add specific code to connect to a ThorLabs motor
            self.connected = True
        
    def disconnect(self):
        """
        Disconnect to motor
        """
        # TODO: Code to disconnect
        self.connected = False
        self.ok = False
        
    def set_velocity(self, vel: float=-1.0, max_vel: float=-1.0):
        """
        Set motor velocity. If a value is not provided, it is not changed.

        Args:
            vel (float): Standard velocity. Defaults to -1.0, which does nothing.
            max_vel (float, optional): Maximum velocity. Defaults to -1.0, which does nothing.
        """
        if max_vel > 0.0:
            self.max_vel = max_vel
        if vel > 0.0:
            self.vel = vel
            
    def set_acceleration(self, accel: float=-1.0, max_accel: float=-1.0):
        """
        Set motor acceleration. If a value is not provided, it is not changed.

        Args:
            accel (float): Standard acceleration. Defaults to -1.0, which does nothing.
            max_accel (float, optional): Maximum acceleration. Defaults to -1.0, which sets standard value.
        """
        if max_accel > 0.0:
            self.max_accel = max_accel
        if accel > 0.0:
            self.accel = accel
    
    def home(self):
        """
        Perform homing procedure
        """
        if not self.get_homed() and self.connected:
            self.start_home()
            
            i = 0
            while not self.homed and i < 600:
                time.sleep(0.1)
                self.homed = self.get_homed()
                i += 1
                
    def start_home(self):
        if self.connected:
            self.homing = True
            # TODO: code to home motor
            pass
        elif self.simulate:
            self.homing = True
            self.go_to(10.0)
            self.move(self.MoveDirection.NEGATIVE)
    
    def go_to(self, pos:float):
        """
        Go to position
        
        Args:
            pos (float): target position
        """
        if self.ok:
            # TODO: code to go to position 
            pass
        elif self.simulate:
            self.pos = pos
            self.sim_last_pos = pos
            self.sim_last_t = time.time()
    
    def move(self, dir:MoveDirection):
        """
        Move continuously in a certain direction

        Args:
            dir (MoveDirection): Direction to move
        """
        if self.ok:
            # TODO: code to move
            pass
        elif self.simulate:
            self.pos = self.get_position()
            self.sim_last_pos = self.pos
            self.sim_last_t = time.time()
            self.moving = dir
    
    def stop(self):
        """
        Stop movement
        """
        if self.connected:
            # TODO: code to stop
            pass
        elif self.simulate:
            self.pos = self.get_position()
            self.sim_last_pos = self.pos
            self.sim_last_t = time.time()
            self.moving = self.MoveDirection.STOPPED
    
    def get_homed(self) -> bool:
        """
        Get the motor's homing status

        Returns:
            bool: Wheter the motor is homed
        """
        if self.connected:
            # TODO: code to get homed status
            pass
        elif self.simulate:
            if self.pos <= 0.0:
                self.go_to(0.0)
                self.homed = True
                self.homing = False
        return self.homed
    
    def get_position(self) -> float:
        """
        Get the motor's current position

        Returns:
            float: The position
        """
        if self.ok:
            # TODO: code to get position
            pass
        elif self.simulate:
            new_pos = self.sim_last_pos + self.vel*self.moving.value*(time.time() - self.sim_last_t)
            self.pos = new_pos
            self.sim_last_pos = self.pos
            self.sim_last_t = time.time()
        return self.pos
    
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
    
    def __init__(self):
        super().__init__()

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
    
    def __init__(self):
        super().__init__()
        
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
    
    def __init__(self):
        super().__init__()
        
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
    
    def __init__(self):
        super().__init__()

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
        
    class Loop(Timer):
        """
        Class that manages a threaded loop.
        """
        def run(self):
            while not self.finished.wait(self.interval):
                self.function(*self.args, **self.kwargs)
    
    def __init__(self):
        """
        This class contains the motors relevant to the taper pulling process.
        Initialization does not connect to any device.
        """
        self.brusher = Brusher()
        self.flame_io = FlameIO()
        self.left_puller = LeftPuller()
        self.right_puller = RightPuller()
        
        self.init_loop = self.Loop(0.1, self.init_loop_function)
        self.init_busy = False
        self.init_running = False
        
    def __del__(self):
        self.close()
        
    def close(self):
        """
        Shutdown procedures
        """
        # Stop thread
        if self.init_running:
            self.init_running = False
            self.init_loop.cancel()
        
    def initialize_motor(self, motor_type: MotorTypes, serial: str="", 
                         vel: float=-1.0, accel: float=-1.0,
                         max_vel: float=-1.0, max_accel: float=-1.0, 
                         simulate=False):
        """
        Connect to, configure, and home (if needed) a motor.
        If a parameter is not given, its current value (default at initialization) is used

        Args:
            motor_type (MotorTypes): Type of the motor to be initialized
            serial (str): Serial number. Defaults to empty.
            vel (float): Standard velocity. Defaults to -1.0.
            accel (float): Standard acceleration. Defaults to -1.0.
            max_vel (float, optional): Maximum velocity. Defaults to -1.0.
            max_accel (float, optional): Maximum acceleration. Defaults to -1.0.
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
        motor.connect(serial, simulate)
        motor.set_velocity(vel, max_vel)
        motor.set_acceleration(accel, max_accel)
        motor.home()
        motor.initing = False
        
        if (motor.connected or motor.simulate) and motor.homed:
            motor.ok = True
            
        return motor.ok
    
    def initialize_motor_async(self, motor_type: MotorTypes, serial: str="", 
                         vel: float=-1.0, accel: float=-1.0,
                         max_vel: float=-1.0, max_accel: float=-1.0, 
                         simulate=False):
        """
        Connect to, configure, and home (if needed) a motor.
        If a parameter is not given, its current value (default at initialization) is used

        Args:
            motor_type (MotorTypes): Type of the motor to be initialized
            serial (str): Serial number. Defaults to empty.
            vel (float): Standard velocity. Defaults to -1.0.
            accel (float): Standard acceleration. Defaults to -1.0.
            max_vel (float, optional): Maximum velocity. Defaults to -1.0.
            max_accel (float, optional): Maximum acceleration. Defaults to -1.0.
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
        motor.connect(serial, simulate)
        motor.set_velocity(vel, max_vel)
        motor.set_acceleration(accel, max_accel)
        if not self.init_running:
            self.init_running = True
            self.init_loop.start()
        
    def init_loop_function(self):
        if not self.init_busy:
            self.init_busy = True
            self.init_running = True
            
            motors = [self.brusher, self.flame_io, self.left_puller, self.right_puller]
            finish = [False]*len(motors)
            for i, motor in enumerate(motors):
                connected = (motor.connected or motor.simulate)
                if motor.initing and connected and not motor.homing:
                    motor.start_home()
                
                if motor.initing and connected and motor.homing and not motor.homed:
                    motor.get_homed()
                    
                if motor.initing and connected and motor.homed:
                    motor.ok = True
                    
                if motor.ok or motor.error:
                    finish[i] = True
                    motor.initing = False
                else:
                    finish[i] = False
            
            self.init_busy = False
            if all(finish):
                self.init_running = False
                self.init_loop.cancel()