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
from threading import Timer, Thread
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
    poll_interval = 10  # ms.
    busy = False
    running_process = False     # If pulling process started
    flame_approaching = False   # Stage 1
    flame_holding = False       # Stage 2
    pulling = False             # Stage 3
    stopping = False            # Stage 4
    standby = False             # Stage 5
    cleaving = False       # Opt. Stage 6
    looping = False             # Opt. Stage 6
    pos_check_precision = 0.1   # Precision expected when checking for positions (mm)
    print_lps = False
    loop_i = 0
    max_i = 100
    loop_time = 0.0
    
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
    flameIO_moving = False
    current_profile_step = 0
    last_total_pulled = 0.0
    
    # Pulling parameters (and some defaults)
    hotzone_function = np.zeros((2, 11))  # Hotzone size x pulled distance
    auto_stop = True  # Stops when pulled distance reaches hz function end
    force_hz_edge = True  # Prevents two diameters
    flame_size = 0.0  # Subtract this value from hotzone
    
    # Brushing control mode
    # 0: Checks for position at every loop, moves indefinitely. Less precision, faster
    # 0: Calculates stopping point, send brusher to that position. More precise, stopping procedure is slow
    brushing_control_mode = 0
    
    brusher_x0 = 31.0  # mm
    brusher_min_span = 0.5  # mm, minimum brush span. Motor glitches if it's too low
    brusher_reverse = False  # Start brushing to more negative positions (v < 0)
    brusher_going2x0 = False  # Brusher is going to x0
    
    flame_io_x0 = 14.3  # mm
    flame_io_hold = 1.0  # s
    flame_io_moveback = False
    flame_io_mb_to = 12.0  # mm
    flame_io_mb_start = 0.01  # mm
    flame_io_mb_end = 200.0  # mm
    
    left_puller_x0 = 84.0  # mm
    right_puller_x0 = 84.0  # mm
    left_puller_xinit = left_puller_x0
    right_puller_xinit = right_puller_x0
    pullers_adaptive_vel = True  # Slows down pulling if flame span is too large and/or when brusher is slower
    pullers_av_threshold = [5, 7]  # mm, threshold to decrease pulling velocity
    pullers_av_factor = 0.5  # will decrease velocity to this factor
    pullers_av_idx = 0  # Keep trac of speed changes
    brusher_enhance_edge = True  # Use acceleration information to improve HZ edges
    
    # Other vars
    cleave_dist = 2.0
    cleave_mode = "slow"
    cleave_ongoing = False
    fio_v0 = [0.0, 0.0]
    pl_v0 = [0.0, 0.0]
    pl_a0 = [0.0, 0.0]
    pr_v0 = [0.0, 0.0]
    pr_a0 = [0.0, 0.0]
    loop_dist_bw = 1.0
    loop_dist_fw = 1.0
    loop_started = False
    loop_loosened = False
    loop_looped = False
    loop_tensioning = False
    loop_tensioned = False
    emergency_stopped = False
    
    def __init__(self):
        """
        "This class centralizes all devices controls."
        """        
        self.motors = TaperPullingMotors()
        
        self.start_update()
        
    def __del__(self):
        self.close()
        
    def close(self):
        """
        Shutdown procedures
        """
        # Stop timer thread
        self.stop_update()
    
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
        self.flameIO_moving = False
        self.current_profile_step = 0
        self.last_total_pulled = 0.0
    
    def init_brusher_as_default(self, async_init=False, simulate=False):
        if async_init:
            init_thread = Thread(target=self.motors.initialize_motor, 
                                 kwargs={"motor_type": self.motors.MotorTypes.BRUSHER, "simulate": simulate})
            init_thread.start()
            return False
        else:
            return self.motors.initialize_motor(self.motors.MotorTypes.BRUSHER, simulate=simulate)
    
    def init_flameio_as_default(self, async_init=False, simulate=False):
        if async_init:
            init_thread = Thread(target=self.motors.initialize_motor, 
                                 kwargs={"motor_type": self.motors.MotorTypes.FLAME_IO, "simulate": simulate})
            init_thread.start()
            return False
        else:
            return self.motors.initialize_motor(self.motors.MotorTypes.FLAME_IO, simulate=simulate)
    
    def init_puller_l_as_default(self, async_init=False, simulate=False):
        if async_init:
            init_thread = Thread(target=self.motors.initialize_motor, 
                                 kwargs={"motor_type": self.motors.MotorTypes.LEFT_PULLER, "simulate": simulate})
            init_thread.start()
            return False
        else:
            return self.motors.initialize_motor(self.motors.MotorTypes.LEFT_PULLER, simulate=simulate)
        
    def init_puller_r_as_default(self, async_init=False, simulate=False):
        if async_init:
            init_thread = Thread(target=self.motors.initialize_motor, 
                                 kwargs={"motor_type": self.motors.MotorTypes.RIGHT_PULLER, "simulate": simulate})
            init_thread.start()
            return False
        else:
            return self.motors.initialize_motor(self.motors.MotorTypes.RIGHT_PULLER, simulate=simulate)
    
    def init_all_motors_as_default(self, async_init=False, simulate=False):
        bok = self.init_brusher_as_default(async_init, simulate)
        fok = self.init_flameio_as_default(async_init, simulate)
        lok = self.init_puller_l_as_default(async_init, simulate)
        rok = self.init_puller_r_as_default(async_init, simulate)
        self.all_motors_ok = bok and fok and lok and rok
        return self.all_motors_ok
    
    def check_all_motors_ok(self):
        bok = self.motors.brusher.ok
        fok = self.motors.flame_io.ok
        lok = self.motors.left_puller.ok
        rok = self.motors.right_puller.ok
        self.all_motors_ok = bok and fok and lok and rok
        return self.all_motors_ok
    
    def start_update(self):
        self.stop_update()
        self.loop_i = 0
        self.update_loop = None
        self.update_loop = self.Loop(self.poll_interval/1000.0, self.update_function)
        self.update_loop.start()
        
    def stop_update(self):
        try:
            self.update_loop.cancel()
        except:
            pass
            
    def update_function(self):
        if not self.busy:
            self.busy = True

            # Get current time
            self.time_bef = self.time_now
            self.time_now = time.time()
            
            if self.print_lps:
                if self.loop_i == 0:
                    self.loop_time = self.time_now
                elif self.loop_i >= self.max_i:
                    print("Loops per second:", int(self.max_i/(time.time() - self.loop_time)))
                    self.loop_i = 0
                    self.loop_time = self.time_now
                self.loop_i += 1
            
            # Get current positions
            self.brusher_pos = self.motors.brusher.get_position()
            self.flame_io_pos = self.motors.flame_io.get_position()
            self.puller_left_pos = self.motors.left_puller.get_position()
            self.puller_right_pos = self.motors.right_puller.get_position()
            
            # Assess if flame tip is in danger zone
            if self.check_all_motors_ok():
                fio_dz = self.motors.flame_io.danger_zone()
                br_dz = self.motors.brusher.danger_zone()
                lp_dz = self.motors.left_puller.danger_zone()
                rp_dz = self.motors.right_puller.danger_zone()
                
                is_in_danger = all([fio_dz, br_dz, lp_dz]) or all([fio_dz, br_dz, rp_dz])
                if is_in_danger:
                    if not self.emergency_stopped:
                        self.emergency_stopped = True
                        print("Flame tip was dangerously close to another motor! For safety, all movement ceased and all processes stopped.")
                        self.stop_pulling()
                elif self.emergency_stopped:
                    self.emergency_stopped = False
        
            if self.running_process:
                # Process stages
                # Convert all stage switches to a 7-bit number
                stage_array = [self.flame_approaching, self.flame_holding, self.pulling, self.stopping,
                               self.standby, self.looping, self.cleaving]
                stage = sum(map(lambda x: x[1] << x[0], enumerate(stage_array)))
                
                if stage == 0:
                    # Stage 0, Sending brusher to start position
                    self.going_to_start()
                elif stage < 2:
                    # Stage 1: Flame approaching
                    self.check_brushing(self.brushing_control_mode)
                    self.approach_flame()
                elif stage < 4:
                    # Stage 2: Flame holding
                    self.check_brushing(self.brushing_control_mode)
                    self.hold_flame()
                elif stage < 8:
                    # Stage 3: Pulling
                    self.check_pulling()
                    self.check_brushing(self.brushing_control_mode)
                    self.check_io_mb()
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
        brusher = (self.brusher_pos >= self.brusher_x0 - self.pos_check_precision) and \
               (self.brusher_pos <= self.brusher_x0 + self.pos_check_precision)
        if brusher:
            self.motors.flame_io.go_to(self.flame_io_x0)
            self.flame_approaching = True
            print("Flame approaching...")
            
            self.motors.brusher.move(self.motors.brusher.MoveDirection(self.brusher_dir))
            print("Started brushing")
    
    def get_pullers_xinit(self):
        self.left_puller_xinit = self.puller_left_pos
        self.right_puller_xinit = self.puller_right_pos
        self.motors.left_puller.set_velocity(self.motors.left_puller.pull_vel)
        self.motors.right_puller.set_velocity(self.motors.right_puller.pull_vel)
        self.pullers_av_idx = 0
        time.sleep(0.1)
    
    def approach_flame(self):
        flame_in = (self.flame_io_pos >= self.flame_io_x0 - self.pos_check_precision) and \
                   (self.flame_io_pos <= self.flame_io_x0 + self.pos_check_precision)
        if flame_in:
            print("Flame in")
            self.time_hold0 = time.time()
            self.flame_holding = True
    
    def hold_flame(self):
        time_hold = time.time() - self.time_hold0
        if time_hold >= self.flame_io_hold:
            brusher_centered = (self.brusher_pos >= self.brusher_x0 - self.pos_check_precision) and \
                      (self.brusher_pos <= self.brusher_x0 + self.pos_check_precision)
            if brusher_centered:
                self.motors.left_puller.move(self.motors.left_puller.MoveDirection(self.puller_left_dir))
                self.motors.right_puller.move(self.motors.right_puller.MoveDirection(self.puller_right_dir))
                self.pulling = True
                print("Flame hold done")
                print("Started pulling")
    
    def check_pulling(self):
        self.total_pulled = np.abs(self.left_puller_xinit - self.puller_left_pos) + \
                            np.abs(self.right_puller_xinit - self.puller_right_pos)
        
        if self.pullers_adaptive_vel:
            if self.pullers_av_idx < len(self.pullers_av_threshold):
                if self.total_pulled >= self.pullers_av_threshold[self.pullers_av_idx]:
                    self.pullers_av_idx += 1
                    self.motors.left_puller.stop(0)
                    self.motors.right_puller.stop(0)
                    new_v = self.pullers_av_factor*self.motors.left_puller.vel
                    self.motors.left_puller.set_velocity(new_v)
                    self.motors.right_puller.set_velocity(new_v)
                    self.motors.left_puller.move(self.motors.left_puller.MoveDirection(self.puller_left_dir))
                    self.motors.right_puller.move(self.motors.right_puller.MoveDirection(self.puller_right_dir))
        
        if self.auto_stop and self.total_pulled >= self.hotzone_function[0][-1]:
            self.stopping = True
            print("Pulling ending...")
    
    def check_brushing(self, mode=0):
        hz = np.interp(self.total_pulled, self.hotzone_function[0], self.hotzone_function[1])
        if mode == 0:
            if hz - self.flame_size >= self.motors.brusher.min_span:                
                dist_compensation = 1.66*self.motors.brusher.vel*self.poll_interval/1000.0
                l = self.brusher_x0 - hz/2.0 + dist_compensation + self.flame_size/2.0
                r = self.brusher_x0 + hz/2.0 - dist_compensation - self.flame_size/2.0
                beyond_limit = ((self.brusher_pos <= l and self.brusher_dir == -1) or 
                                (self.brusher_pos >= r and self.brusher_dir == 1))
                if self.brusher_enhance_edge:
                    if beyond_limit:
                        if self.motors.brusher.moving:
                            self.rhz_edges.append(self.brusher_pos + self.brusher_dir*self.flame_size/2.0)
                            self.motors.brusher.stop(1)
                            if self.pulling:
                                self.motors.left_puller.stop(0)
                                self.motors.right_puller.stop(0)
                        elif self.motors.brusher.motor_stopped():
                            self.brusher_dir = -1*self.brusher_dir
                            self.motors.brusher.move(self.motors.brusher.MoveDirection(self.brusher_dir))
                        # Brute force stop motor, if for some reason the stop() function doesn't do its job.
                        else:
                            # In the future, it might be important to set some flag here.
                            # Several stop commands in a row (before it actually stops), may harm performance
                            self.motors.brusher.stop(1)
                    elif self.brusher_pos >= l - dist_compensation and self.brusher_pos <= r + dist_compensation:
                        if self.pulling and not self.motors.left_puller.moving:
                            self.motors.left_puller.move(self.motors.left_puller.MoveDirection(self.puller_left_dir))
                        if self.pulling and not self.motors.right_puller.moving:
                            self.motors.right_puller.move(self.motors.right_puller.MoveDirection(self.puller_right_dir))
                    # Brute force move motor, if for some reason the move() function doesn't do its job.
                    elif self.motors.brusher.moving and self.motors.brusher.motor_stopped():
                        # In the future, it might be important to change the stop flag here.
                        # Several commands in a row may harm performance
                        self.motors.brusher.move(self.motors.brusher.MoveDirection(self.brusher_dir))
                else:
                    if beyond_limit:
                        self.brusher_dir = -1*self.brusher_dir
                        self.motors.brusher.stop(0)
                        self.rhz_edges.append(self.brusher_pos - self.brusher_dir*self.flame_size/2.0)
                        self.motors.brusher.move(self.motors.brusher.MoveDirection(self.brusher_dir))
            else:
                if (self.brusher_pos >= self.brusher_x0 + self.pos_check_precision or  
                    self.brusher_pos <= self.brusher_x0 - self.pos_check_precision):
                    if not self.brusher_going2x0:
                        self.brusher_going2x0 = True
                        self.motors.brusher.go_to(self.brusher_x0)
                else:
                    self.brusher_going2x0 = False
        else:
            if hz - self.flame_size >= self.motors.brusher.min_span:
                # This is a work in progressm, possibly useless
                if self.motors.brusher.movement == self.motors.brusher.MoveDirection.STOPPED:
                    # Interpolate hz function, discarding previous values
                    new_x = np.linspace(self.total_pulled,  self.hotzone_function[0][-1], 1001)
                    new_hz = np.interp(new_x, self.hotzone_function[0], self.hotzone_function[1])
                    
                    # Calculate intersection point from functions
                    v_x = self.motors.left_puller.pull_vel + self.motors.right_puller.pull_vel
                    v_f = self.motors.brusher.vel
                    difference = np.abs((new_x - self.total_pulled) - (new_hz*v_x/v_f))
                    delta_l = new_hz[difference.argmin()] - self.flame_size
                    
                    # Move
                    self.brusher_dir = -1*self.brusher_dir
                    self.rhz_edges.append(self.brusher_pos - self.brusher_dir*self.flame_size/2.0)
                    self.motors.brusher.go_to(self.brusher_x0 + delta_l*self.brusher_dir/2.0)
            else:
                if (self.brusher_pos >= self.brusher_x0 + self.pos_check_precision or  
                    self.brusher_pos <= self.brusher_x0 - self.pos_check_precision):
                    if not self.brusher_going2x0:
                        self.brusher_going2x0 = True
                        self.motors.brusher.go_to(self.brusher_x0)
                else:
                    self.brusher_going2x0 = False
                
    def check_io_mb(self):
        if self.flame_io_moveback and not self.flameIO_moving:
            if self.total_pulled >= self.flame_io_mb_start:
                to_pull = self.flame_io_mb_end - self.total_pulled
                dist = np.abs(self.flame_io_mb_to - self.flame_io_x0)
                v_pull = self.motors.left_puller.get_velocity()
                t = to_pull/(2*v_pull)
                self.fio_v0 = self.motors.flame_io.get_velocity()
                v_fmov = dist/t
                self.motors.flame_io.set_velocity(v_fmov)
                self.motors.flame_io.go_to(self.flame_io_mb_to)
                self.flameIO_moving = True                 
    
    def check_stopping(self):
        stop_ok = False
        hz = np.interp(self.total_pulled, self.hotzone_function[0], self.hotzone_function[1])
        if self.force_hz_edge and hz >= self.motors.brusher.min_span:
            l = self.brusher_x0 - self.hotzone_function[1][-1]/2.0
            r = self.brusher_x0 + self.hotzone_function[1][-1]/2.0
            if (self.brusher_pos <= l  or self.brusher_pos >= r):
                stop_ok = True
        else:
            stop_ok = True
        
        if stop_ok:
            self.stop_pulling()
            self.standby = True
            print("Stopped")
    
    def perform_cleave(self):
        if not self.cleave_ongoing:
            self.cleave_ongoing = True
            
            self.pl_a0 = self.motors.right_puller.get_acceleration()
            self.pl_v0 = self.motors.right_puller.get_velocity()
            self.pr_a0 = self.motors.right_puller.get_acceleration()
            self.pr_v0 = self.motors.right_puller.get_velocity()
            
            if self.cleave_mode == "fast":
                self.motors.left_puller.set_acceleration(900)
                self.motors.left_puller.set_velocity(100)
                self.motors.right_puller.set_acceleration(900)
                self.motors.right_puller.set_velocity(100)
            else:
                self.motors.left_puller.set_velocity(0.1)
                self.motors.right_puller.set_velocity(0.1)
            
            self.motors.left_puller.move_relative(-self.cleave_dist)
            self.motors.right_puller.move_relative(-self.cleave_dist)
        else:
            if (not self.motors.left_puller.moving) and (not self.motors.right_puller.moving):
                self.motors.left_puller.set_acceleration(self.pl_a0)
                self.motors.left_puller.set_velocity(self.pl_v0)
                self.motors.right_puller.set_acceleration(self.pr_a0)
                self.motors.right_puller.set_velocity(self.pr_v0)
            
                self.cleaving = False
                self.cleave_ongoing = False
            
    def perform_loop(self):
        if not self.loop_started:
            self.loop_started = True
            self.motors.left_puller.go_to(self.puller_left_pos - self.puller_left_dir*self.loop_dist_fw)
            self.motors.right_puller.go_to(self.puller_right_pos - self.puller_right_dir*self.loop_dist_fw)
        elif not self.loop_loosened:
            lok = False
            rok = False
            if self.motors.left_puller.movement == self.motors.left_puller.MoveDirection.STOPPED:
                lok = True
            if self.motors.right_puller.movement == self.motors.right_puller.MoveDirection.STOPPED:
                rok = True
            if lok and rok:
                self.loop_loosened = True
        elif self.loop_looped and not self. loop_tensioning:
            self.motors.left_puller.go_to(self.puller_left_pos + self.puller_left_dir*self.loop_dist_bw)
            self.motors.right_puller.go_to(self.puller_right_pos + self.puller_right_dir*self.loop_dist_bw)
            self.loop_tensioning = True
        elif self.loop_tensioning and not self.loop_tensioned:
            lok = False
            rok = False
            if self.motors.left_puller.movement == self.motors.left_puller.MoveDirection.STOPPED:
                lok = True
            if self.motors.right_puller.movement == self.motors.right_puller.MoveDirection.STOPPED:
                rok = True
            if lok and rok:
                self.loop_tensioned = True
        elif self.loop_tensioned:
            self.looping = False
            self.loop_started = False
            self.loop_loosened = False
            self.loop_looped = False
            self.loop_tensioning = False
            self.loop_tensioned = False
        
    def go_to_start(self):
        if self.check_all_motors_ok:
            print("Motors going to start...")
            self.motors.brusher.go_to(self.brusher_x0)
            self.motors.brusher.wait_for_movement()
            self.motors.left_puller.go_to(self.left_puller_x0)
            self.motors.right_puller.go_to(self.right_puller_x0)
            self.motors.left_puller.wait_for_movement()
            self.motors.right_puller.wait_for_movement()
            print("Starting positions OK")
    
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
            # Send brusher to starting position
            self.motors.brusher.go_to(self.brusher_x0)
            self.get_pullers_xinit()
            
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
        
        # Retract Flame I/O
        if self.flame_io_moveback:
            self.motors.flame_io.set_velocity(self.fio_v0)
            self.flameIO_moving = False
            time.sleep(0.1)
        self.motors.flame_io.go_to(0.0)
        
        # Stop all motors
        self.motors.brusher.stop()
        self.motors.left_puller.stop()
        self.motors.right_puller.stop()
        
        
        # Reset pullers velocity
        self.motors.left_puller.set_velocity(self.motors.left_puller.vel)
        self.motors.right_puller.set_velocity(self.motors.right_puller.vel)
        time.sleep(0.1)
        
        self.reset_pull_stats()