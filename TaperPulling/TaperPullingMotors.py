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
    homed = False
    ok = False
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
        self.serial = serial
        self.simulate = simulate
        if serial != "" and not simulate:
            # TODO: add specific code to connect to a ThorLabs motor
            self.ok = False
        
    def disconnect(self):
        """
        Disconnect to motor
        """
        # TODO: Code to disconnect
        self.ok = False
        
    def set_parameters(self, vel: float, accel: float, max_vel: float=0.0, max_accel: float=0.0):
        """
        Set motor parameters. If max values are not provided, the standard value is used.

        Args:
            vel (float): Standard velocity
            accel (float): Standard acceleration
            max_vel (float, optional): Maximum velocity. Defaults to 0.0, which sets standard value.
            max_accel (float, optional): Maximum acceleration. Defaults to 0.0, which sets standard value.
        """
        self.vel = vel
        self.accel = accel
        if max_vel != 0.0:
            self.max_vel = max_vel
        else:
            self.max_vel = vel
        if max_accel != 0.0:
            self.max_accel = max_accel
        else:
            self.max_accel = accel
    
    def home(self):
        """
        Perform homing procedure
        """
        if not self.get_homed():
            if self.ok:
                # TODO: code to home motor 
                pass
            elif self.simulate:
                self.pos = 0.0
                self.homed = True
    
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
        if self.ok:
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
        if self.ok:
            # TODO: code to get homed status
            pass
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
    def __init__(self):
        super().__init__()

class FlameIO(GenericTLMotor):
    """
    Class for the flame in/out motor. Inherits the generic ThorLabs motors class.
    """
    def __init__(self):
        super().__init__()
        
class Puller(GenericTLMotor):
    """
    Class for the pullers. Inherits the generic ThorLabs motors class.
    """
    def __init__(self):
        super().__init__()

class TaperPullingMotors:
    """
    This class contains the motors relevant to the taper pulling process.
    """
    def __init__(self):
        """
        This class contains the motors relevant to the taper pulling process.
        Initialization does not connect to any device.
        """
        self.brusher = Brusher()
        self.flame_io = FlameIO()
        self.left_puller = Puller()
        self.right_puller = Puller()
        
    def initialize_brusher(self, serial:str, vel: float, accel: float, max_vel: float=0.0, max_accel: float=0.0, simulate=False):
        """
        Connect to, configure, and home (if needed) brusher motor.

        Args:
            serial (str): Serial number
            vel (float): Standard velocity
            accel (float): Standard acceleration
            max_vel (float, optional): Maximum velocity. Defaults to 0.0.
            max_accel (float, optional): Maximum acceleration. Defaults to 0.0.
            simulate (bool, optional): Simulate this motor. Defaults to False.
        """
        self.brusher.connect(serial, simulate)
        self.brusher.set_parameters(vel, accel, max_vel, max_accel)
        self.brusher.home()
        
    def initialize_flame_io(self, serial:str, vel: float, accel: float, max_vel: float=0.0, max_accel: float=0.0, simulate=False):
        """
        Connect to, configure, and home (if needed) flame in/out motor.

        Args:
            serial (str): Serial number
            vel (float): Standard velocity
            accel (float): Standard acceleration
            max_vel (float, optional): Maximum velocity. Defaults to 0.0.
            max_accel (float, optional): Maximum acceleration. Defaults to 0.0.
            simulate (bool, optional): Simulate this motor. Defaults to False.
        """
        self.flame_io.connect(serial, simulate)
        self.flame_io.set_parameters(vel, accel, max_vel, max_accel)
        self.flame_io.home()
        
    def initialize_left_puller(self, serial:str, vel: float, accel: float, max_vel: float=0.0, max_accel: float=0.0, simulate=False):
        """
        Connect to, configure, and home (if needed) left puller motor.

        Args:
            serial (str): Serial number
            vel (float): Standard velocity
            accel (float): Standard acceleration
            max_vel (float, optional): Maximum velocity. Defaults to 0.0.
            max_accel (float, optional): Maximum acceleration. Defaults to 0.0.
            simulate (bool, optional): Simulate this motor. Defaults to False.
        """
        self.left_puller.connect(serial, simulate)
        self.left_puller.set_parameters(vel, accel, max_vel, max_accel)
        self.left_puller.home()
        
    def initialize_right_puller(self, serial:str, vel: float, accel: float, max_vel: float=0.0, max_accel: float=0.0, simulate=False):
        """
        Connect to, configure, and home (if needed) right puller motor.

        Args:
            serial (str): Serial number
            vel (float): Standard velocity
            accel (float): Standard acceleration
            max_vel (float, optional): Maximum velocity. Defaults to 0.0.
            max_accel (float, optional): Maximum acceleration. Defaults to 0.0.
            simulate (bool, optional): Simulate this motor. Defaults to False.
        """
        self.right_puller.connect(serial, simulate)
        self.right_puller.set_parameters(vel, accel, max_vel, max_accel)
        self.right_puller.home()