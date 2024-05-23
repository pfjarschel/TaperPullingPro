# -*- coding: utf-8 -*-
# =============================================================================
# Created By   : Paulo Jarschel
# First release: May 31, 2024
# Gleb Wataghin Physics Institute (IFGW), University of Campinas
# =============================================================================
"""
This file contains the TaperPullingMotors class, along with the GenericTLMotor
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
        # TODO: add specific code to connect to a ThorLabs motor
        self.serial = serial
        self.simulate = simulate
        
    def disconnect(self):
        """
        Disconnect to motor
        """
        # TODO: Code to disconnect
        self.ok = False
    
    def home(self):
        """
        Perform homing procedure
        """
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
            # TODO: code to move
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
            new_pos = self.sim_last_pos + self.vel*self.moving*(time.time() - self.sim_last_t)
            self.pos = new_pos
            self.sim_last_pos = self.pos
            self.sim_last_t = time.time()
        return self.pos
    
class Brusher(GenericTLMotor):
    """
    Class for the flame brusher motor. Inherits the generic ThorLabs motors class.
    """
    def __init__(self):
        super.__init__()

class FlameIO(GenericTLMotor):
    """
    Class for the flame in/out motor. Inherits the generic ThorLabs motors class.
    """
    def __init__(self):
        super.__init__()
        
class Puller(GenericTLMotor):
    """
    Class for the pullers. Inherits the generic ThorLabs motors class.
    """
    def __init__(self):
        super.__init__()

class TaperPullingMotors:
    """
    This class contains the motors relevant to the taper pulling process.
    """
    def __init__(self):
        self.brusher = Brusher()
        self.flame_io = FlameIO()
        self.left_puller = Puller()
        self.right_puller = Puller()      