# -*- coding: utf-8 -*-
# =============================================================================
# Created By   : Paulo Jarschel
# First release: May 31, 2024
# Gleb Wataghin Physics Institute (IFGW), University of Campinas
# =============================================================================
"""
This module contains the TaperPullingCore class, responsible for the control of
the taper pulling actions and parameters. It centralizes all information and
control procedures during taper fabrication.
"""
# =============================================================================

import numpy as np
import time
from threading import Timer
from .TaperPullingMotors import TaperPullingMotors


class TaperPullingCore:
    """
    This class is the core/brain of the taper pulling process.
    It constantly updates current device status and provides useful functions
    to monitor them externally.
    It requires a hotzone function (hotzone size x pulled distance) for the
    process. This can be easily obtained from the TaperShape class.
    """
    
    class Loop(Timer):
        """
        Class that manages a threaded loop.
        """
        def run(self):
            while not self.finished.wait(self.interval):
                self.function(*self.args, **self.kwargs)
    
    # Flow control
    update_loop = None
    poll_interval = 1  # ms.
    busy = False
    running_process = False     # If pulling process started
    flame_approaching = False   # Stage 1
    flame_holding = False       # Stage 2
    pulling = False             # Stage 3
    stopping = False            # Stage 4
    standby = False             # Stage 5
    cleaving = False            # Opt. Stage 6
    looping = False             # Opt. Stage 6
    pos_check_precision = 0.1   # Precision expected when checking for positions (mm)
    
    # Devices to control
    motors = None
    all_motors_ok = False
    
    # Current data (for convenience)
    brusher_pos = 0.0
    flame_io_pos = 0.0
    puller_left_pos = 0.0
    puller_right_pos = 0.0
    
    # Movement directions
    brusher_dir0 = 1  # Position increases
    brusher_dir = 1  # Position increases
    flame_io_dir = 1  # Position increases
    puller_left_dir = -1  # Position decreases
    puller_right_dir = -1  # Position decreases
    
    # Pulling variables
    time_now = 0.0
    time_bef = 0.0
    time_init = 0.0
    time_hold0 = 0.0
    count = 0
    total_pulled = 0.0
    hotzone = 0.0
    rhz_edges = []
    flameIO_zeroed = False
    brusher_zeroed = False
    pullers_zeroed = False
    flameIO_backing = False
    current_profile_step = 0
    last_total_pulled = 0.0
    
    # Pulling parameters (and some defautls)
    hotzone_function = np.zeros((2, 11))  # Hotzone size x pulled distance
    auto_stop = True  # Stops when pulled distance reaches hz function end
    force_hz_edge = True  # Prevents two diameters
    
    brusher_x0 = 33.0  # mm
    brusher_min_span = 0.5  # mm, minimum brush span. Motor glitches if it's too low
    brusher_reverse = False  # Start brushing to more negative positions (v < 0)
    
    flame_io_x0 = 14.3  # mm
    flame_io_hold = 1.0  # s
    flame_io_moveback = False
    flame_io_mb_to = 12.0  # mm
    flame_io_mb_start = 0.01  # mm
    flame_io_mb_end = 200.0  # mm
    
    left_puller_x0 = 84.0  # mm
    right_puller_x0 = 84.0  # mm
    pullers_adaptive_vel = True  # Slows down pulling if flame span is too large and/or when brusher is slower
    brusher_enhance_edge = True  # Use acceleration information to improve HZ edges
    
    # Other vars
    cleave_dist = 5.0
    cleave_l = 0.0
    cleave_r = 0.0
    cleave_started = False
    cleave_ended = False
    pl_v0 = [0.0, 0.0]
    pl_a0 = [0.0, 0.0]
    pr_v0 = [0.0, 0.0]
    pr_a0 = [0.0, 0.0]
    loop_dist_bw = 1.0
    loop_dist_fw = 1.0
    loop_started = False
    loop_tensioned = False
    loop_looped = False
    loop_loosing = False
    loop_loosed = False
    
    def __init__(self):
        """
        "This class centralizes all devices controls."
        """        
        self.motors = TaperPullingMotors()
        
        self.update_loop = self.Loop(self.poll_interval/1000.0, self.update_function)
        self.update_loop.start()
        
    def __del__(self):
        self.close()
        
    def close(self):
        """
        Shutdown procedures
        """
        # Stop timer thread
        self.update_loop.cancel()
    
    def reset_pull_stats(self):
        """
        Resets all pulling variables to the default/starting values
        """
        # Flow
        self.running_process = False
        self.flame_approaching = False  # Stage 1
        self.flame_holding = False      # Stage 2
        self.pulling = False            # Stage 3
        self.stopping = False           # Stage 4
        self.standby = False            # Stage 5
        self.looping = False            # Opt. Stage 6
        self.cleaving = False           # Opt. Stage 6
        
        # Process variables
        self.time_now = 0.0
        self.time_bef = 0.0
        self.time_init = 0.0
        self.time_hold0 = 0.0
        self.count = 0
        self.total_pulled = 0.0
        self.hotzone = 0.0
        self.rhz_edges = []
        self.brusher_dir = self.brusher_dir0
        self.flameIO_zeroed = False
        self.brusher_zeroed = False
        self.pullers_zeroed = False
        self.flameIO_backing = False
        self.current_profile_step = 0
        self.last_total_pulled = 0.0
    
    def init_brusher_as_default(self, poll_ms=100, simulate=False):
        return self.motors.initialize_motor(self.motors.MotorTypes.BRUSHER, poll_ms, simulate=simulate)
    
    def init_flameio_as_default(self, poll_ms=100, simulate=False):
        return self.motors.initialize_motor(self.motors.MotorTypes.FLAME_IO, poll_ms, simulate=simulate)
    
    def init_puller_l_as_default(self, poll_ms=100, simulate=False):
        return self.motors.initialize_motor(self.motors.MotorTypes.LEFT_PULLER, poll_ms, simulate=simulate)
        
    def init_puller_r_as_default(self, poll_ms=100, simulate=False):
        return self.motors.initialize_motor(self.motors.MotorTypes.RIGHT_PULLER, poll_ms, simulate=simulate)
    
    def init_all_motors_as_default(self, poll_ms=100, simulate=False):
        bok = self.init_brusher_as_default(poll_ms, simulate)
        fok = self.init_flameio_as_default(poll_ms, simulate)
        lok = self.init_puller_l_as_default(poll_ms, simulate)
        rok = self.init_puller_r_as_default(poll_ms, simulate)
        self.all_motors_ok = bok and fok and lok and rok
        return self.all_motors_ok
    
    def check_all_motors_ok(self):
        bok = self.motors.brusher.ok
        fok = self.motors.flame_io.ok
        lok = self.motors.left_puller.ok
        rok = self.motors.right_puller.ok
        self.all_motors_ok = bok and fok and lok and rok
        return self.all_motors_ok
            
    def update_function(self):
        if not self.busy:
            self.busy = True
        
            # Get current time
            self.time_bef = self.time_now
            self.time_now = time.time()
            
            # Get current positions
            self.brusher_pos = self.motors.brusher.get_position()
            self.flame_io_pos = self.motors.flame_io.get_position()
            self.puller_left_pos = self.motors.left_puller.get_position()
            self.puller_right_pos = self.motors.right_puller.get_position()
            
            if self.running_process:
                # Process stages
                # Convert all stage switches to 7-bit number
                stage_array = [self.flame_approaching, self.flame_holding, self.pulling, 
                            self.stopping, self.standby, self.looping, self.cleaving]
                stage = sum(map(lambda x: x[1] << x[0], enumerate(stage_array)))
                
                if stage == 0:
                    # Stage 0, Sending motors to start positions
                    self.going_to_start()
                elif stage < 2:
                    # Stage 1: Flame approaching
                    self.check_brushing()
                    self.approach_flame()
                elif stage < 4:
                    # Stage 2: Flame holding
                    self.check_brushing()
                    self.hold_flame()
                elif stage < 8:
                    # Stage 3: Pulling
                    self.check_pulling()
                    self.check_brushing()
                elif stage < 16:
                    # Stage 4: Stopping
                    self.check_stopping()
                elif stage < 32:
                    # Stage 5: Stand-by, do nothing (for now?)
                    pass
                elif stage < 64:
                    # Opt. Stage 6: Looping
                    self.perform_loop()
                elif stage < 128:
                    # Opt. Stage 6: Cleaving
                    self.perform_cleave()
            
            self.busy = False
                
    def going_to_start(self):
        left = (self.puller_left_pos >= self.left_puller_x0 - self.pos_check_precision) and \
               (self.puller_left_pos <= self.left_puller_x0 + self.pos_check_precision)
        right = (self.puller_right_pos >= self.right_puller_x0 - self.pos_check_precision) and \
               (self.puller_right_pos <= self.right_puller_x0 + self.pos_check_precision)
        brusher = (self.brusher_pos >= self.brusher_x0 - self.pos_check_precision) and \
               (self.brusher_pos <= self.brusher_x0 + self.pos_check_precision)
        if left and right and brusher:
            self.motors.brusher.move(self.motors.brusher.MoveDirection(self.brusher_dir))
            self.flame_approaching = True
            print("starting positions ok")
            print("started brushing")
    
    def approach_flame(self):
        flame_in = (self.flame_io_pos >= self.flame_io_x0 - self.pos_check_precision) and \
                   (self.flame_io_pos <= self.flame_io_x0 + self.pos_check_precision)
        if flame_in:
            print("flame in")
            self.time_hold0 = time.time()
            self.flame_holding = True
    
    def hold_flame(self):
        time_hold = time.time() - self.time_hold0
        if time_hold >= self.flame_io_hold:
            self.motors.left_puller.move(self.motors.left_puller.MoveDirection(self.puller_left_dir))
            self.motors.right_puller.move(self.motors.right_puller.MoveDirection(self.puller_right_dir))
            self.pulling = True
            print("flame hold done")
            print("started pulling")
    
    def check_pulling(self):
        self.total_pulled = np.abs(self.left_puller_x0 - self.puller_left_pos) + \
                            np.abs(self.right_puller_x0 - self.puller_right_pos)
                            
        if self.auto_stop and self.total_pulled >= self.hotzone_function[0][-1]:
            self.stopping = True
            print("pulling end")
    
    def check_brushing(self):
        if self.motors.brusher.moving == self.motors.brusher.MoveDirection.STOPPED:
            self.motors.brusher.move(self.motors.brusher.MoveDirection(self.brusher_dir))
        hz = np.interp(self.total_pulled, self.hotzone_function[0], self.hotzone_function[1])
        if self.brusher_enhance_edge and False:
            pass
        else:
            l = self.brusher_x0 - hz/2.0
            r = self.brusher_x0 + hz/2.0
        if (self.brusher_pos < l and self.brusher_dir == -1) or \
            self.brusher_pos > r and self.brusher_dir == 1:
            self.rhz_edges.append(self.brusher_pos)
            self.brusher_dir = -1*self.brusher_dir
            self.motors.brusher.move(self.motors.brusher.MoveDirection(self.brusher_dir))
    
    def check_stopping(self):
        stop_ok = False
        if self.force_hz_edge:
            l = self.brusher_x0 - self.hotzone_function[1][-1]/2.0
            r = self.brusher_x0 + self.hotzone_function[1][-1]/2.0
            if (self.brusher_pos <= l  or self.brusher_pos >= r):
                stop_ok = True
        else:
            stop_ok = True
        
        if stop_ok:
            self.stop_pulling()
            self.standby = True
            print("stopped")
    
    def perform_cleave(self):
        if not self.cleave_started:
            self.pl_a0 = [self.motors.right_puller.accel, self.motors.right_puller.max_accel]
            self.pl_v0 = [self.motors.right_puller.vel, self.motors.right_puller.max_vel]
            self.pr_a0 = [self.motors.right_puller.accel, self.motors.right_puller.max_accel]
            self.pr_v0 = [self.motors.right_puller.vel, self.motors.right_puller.max_vel]
            
            self.motors.left_puller.set_acceleration(5000, 5000)
            self.motors.left_puller.set_velocity(500, 500)
            self.motors.right_puller.set_acceleration(5000, 5000)
            self.motors.right_puller.set_velocity(500, 500)
            
            self.motors.left_puller.move(self.motors.left_puller.MoveDirection(self.puller_left_dir))
            self.motors.right_puller.move(self.motors.right_puller.MoveDirection(self.puller_right_dir))
            
            self.cleave_l = self.puller_left_pos + self.cleave_dist*self.puller_left_dir
            self.cleave_r = self.puller_right_pos + self.cleave_dist*self.puller_right_dir
            
            self.cleave_started = True
        elif not self.cleave_ended:
            lok = False
            rok = False
            if (self.puller_left_pos - self.cleave_l)*self.puller_left_dir >= 0.0:
                self.motors.left_puller.stop()
                lok = True
            if (self.puller_right_pos - self.cleave_r)*self.puller_right_dir >= 0.0:
                self.motors.right_puller.stop()
                rok = True
            if lok and rok:
                self.cleave_ended = True
        elif self.cleave_ended:
            self.cleaving = False
            self.cleave_started = False
            self.cleave_ended = False
            self.motors.left_puller.set_acceleration(self.pl_a0[0], self.pl_a0[1])
            self.motors.left_puller.set_velocity(self.pl_v0[0], self.pl_v0[1])
            self.motors.right_puller.set_acceleration(self.pr_a0[0], self.pr_a0[1])
            self.motors.right_puller.set_velocity(self.pr_v0[0], self.pr_v0[1])
            
    def perform_loop(self):
        if not self.loop_started:
            self.loop_started = True
            self.motors.left_puller.go_to(self.puller_left_pos + self.puller_left_dir*self.loop_dist_bw)
            self.motors.right_puller.go_to(self.puller_right_pos + self.puller_right_dir*self.loop_dist_bw)
        elif not self.loop_tensioned:
            lok = False
            rok = False
            if self.motors.left_puller.moving == self.motors.left_puller.MoveDirection.STOPPED:
                lok = True
            if self.motors.right_puller.moving == self.motors.right_puller.MoveDirection.STOPPED:
                rok = True
            if lok and rok:
                self.loop_tensioned = True
        elif self.loop_looped and not self. loop_loosing:
            self.motors.left_puller.go_to(self.puller_left_pos - self.puller_left_dir*self.loop_dist_fw)
            self.motors.right_puller.go_to(self.puller_right_pos - self.puller_right_dir*self.loop_dist_fw)
            self.loop_loosing = True
        elif self.loop_loosing and not self.loop_loosed:
            lok = False
            rok = False
            if self.motors.left_puller.moving == self.motors.left_puller.MoveDirection.STOPPED:
                lok = True
            if self.motors.right_puller.moving == self.motors.right_puller.MoveDirection.STOPPED:
                rok = True
            if lok and rok:
                self.loop_loosed = True
        elif self.loop_loosed:
            self.looping = False
            self.loop_started = False
            self.loop_tensioned = False
            self.loop_looped = False
            self.loop_loosing = False
            self.loop_loosed = False
        
    def go_to_start(self):
        if self.check_all_motors_ok:
            self.motors.brusher.go_to(self.brusher_x0)
            self.motors.brusher.wait_for_movement()
            self.motors.left_puller.go_to(self.left_puller_x0)
            self.motors.right_puller.go_to(self.right_puller_x0)
            self.motors.left_puller.wait_for_movement()
            self.motors.right_puller.wait_for_movement()
            self.motors.flame_io.go_to(self.flame_io_x0)
    
    def start_process(self, hotzone_function: list[list[float]]|np.ndarray):
        """
        Starts the pulling process.
        
        Args:
            hotzone_function (np.ndarray): A 2 dimensional list or numpy array 
                containing the hotzone size vs. total pulled distance.
        """
        
        # Reset
        self.reset_pull_stats()
        
        if self.check_all_motors_ok:
            # Go to starting positions
            self.go_to_start()
            
            # Set hotzone function
            try:
                self.hotzone_function = np.array(hotzone_function)
            except:
                raise Exception("Error: hotzone_function must be a 2D list or numpy array.")
            if self.hotzone_function.shape[0] >= 2:
                self.running_process = True
            else:
                raise Exception("Error: hotzone_function must be a 2D list or numpy array.")
            
            # Set initial time
            self.time_init = time.time()
            self.time_now = time.time()
        else:
            print("Error: not all motors initialized correctly.")
        
    def stop_pulling(self):
        self.standby = True
        
        # Stop all motors
        self.motors.brusher.stop()
        self.motors.flame_io.stop()
        self.motors.left_puller.stop()
        self.motors.right_puller.stop()
        
        # Retract Flame I/O
        self.motors.flame_io.go_to(0.0)
    
        
        
        
            
        