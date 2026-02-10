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
    poll_interval = 50  # ms.
    busy = False
    rel_pull = False            # Flame brusher doesn't move, brushing is performed on top of pulling movement.
    running_process = False     # If pulling process started
    flame_approaching = False   # Stage 1
    flame_holding = False       # Stage 2
    pulling = False             # Stage 3
    stopping = False            # Stage 4
    standby = False             # Stage 5
    cleaving = False            # Opt. Stage 6
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
    time_init_pull = 0.0
    time_brush_check_0 = 0.0
    time_brush_check_1 = 0.0
    fake_brusher_pos = 0.0
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
    # 1: Calculates stopping point, send brusher to that position. More precise, stopping procedure is slow
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
    pullers_av_threshold = [10, 20, 40, 60, 80, 100, 120, 140, 160, 180, 200]  # mm, thresholds to decrease pulling velocity
    pullers_av_factor = 0.75  # will decrease velocity to this factor
    pullers_av_idx = 0  # Keep track of speed changes
    pull_vel = 0.0
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
        
        # Velocity Tracking State
        self.last_pos_poll_time = 0.0
        self.tracking_kp = 1.5
        
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
        self.puller_left_pos = None
        self.puller_right_pos = None
        self.last_pos_poll_time = 0.0
        self.l_golden_pos = 0.0
        self.r_golden_pos = 0.0
        self.pull_started_init = False
        self.current_profile_step = 0
        self.last_total_pulled = 0.0
        self.l_last_cmd_v = 0.0
        self.r_last_cmd_v = 0.0
        self.l_last_cmd_dir = 0
        self.r_last_cmd_dir = 0
    
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

            try:
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
                
                # Get current positions (Throttled to 10Hz for stability)
                if (self.time_now - self.last_pos_poll_time) >= 0.100:
                    self.brusher_pos = self.motors.brusher.get_position()
                    self.flame_io_pos = self.motors.flame_io.get_position()
                    self.puller_left_pos = self.motors.left_puller.get_position()
                    self.puller_right_pos = self.motors.right_puller.get_position()
                    self.last_pos_poll_time = self.time_now
                
                # Guard: If still initializing (None), skip logic
                if self.puller_left_pos is None or self.puller_right_pos is None:
                    self.busy = False
                    return
                
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
            except Exception as e:
                print("CRITICAL ERROR in update_function:", e)
                import traceback
                traceback.print_exc()
                self.stop_pulling()
                self.busy = False
            
            self.busy = False
                
    def going_to_start(self):
        brusher = (self.brusher_pos >= self.brusher_x0 - self.pos_check_precision) and \
               (self.brusher_pos <= self.brusher_x0 + self.pos_check_precision)
        if brusher:
            self.fake_brusher_pos = 0.0
            self.motors.flame_io.go_to(self.flame_io_x0)
            self.flame_approaching = True
            print("Flame approaching...")
            
            if self.rel_pull:
                self.time_brush_check_0 = time.time()
                self.time_brush_check_1 = time.time()
            else:
                self.motors.brusher.move(self.motors.brusher.MoveDirection(self.brusher_dir))
            
            print("Started brushing")
    
    def get_pullers_xinit(self):
        # Force hardware poll to ensure we don't carry over None from reset_pull_stats
        self.puller_left_pos = self.motors.left_puller.get_position()
        self.puller_right_pos = self.motors.right_puller.get_position()

        self.left_puller_xinit = self.puller_left_pos
        self.right_puller_xinit = self.puller_right_pos
        self.pull_vel = self.motors.left_puller.pull_vel
        if self.rel_pull:
            self.motors.left_puller.set_velocity(self.motors.brusher.vel - self.brusher_dir*self.motors.left_puller.pull_vel)
            self.motors.right_puller.set_velocity(self.motors.brusher.vel + self.brusher_dir*self.motors.right_puller.pull_vel)
        else:
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
            if self.rel_pull:
                # In rel_pull, "brusher centered" means the virtual fake_brusher_pos is crossing 0.
                # Physical motors are oscillating, so checking their exact position against start is flaky.
                # We check if absolute value of fake_brusher_pos is within reasonable tolerance.
                brusher_centered = abs(self.fake_brusher_pos) < (self.pos_check_precision * 2.0)
                if brusher_centered:
                    # self.brusher_dir = self.brusher_dir0 # REMOVED: Interms with check_brushing logic
                    # CONFLICT FIX: Do not manually move motors here. 
                    # check_brushing() is running in parallel and handles the maintenance/centering.
                    
                    self.time_brush_check_0 = time.time()
                    self.time_brush_check_1 = time.time()
                    self.pulling = True
                    print("Flame hold done")
                    print("Started pulling")
                    self.time_init_pull = time.time()
            else:
                brusher_centered = (self.brusher_pos >= self.brusher_x0 - self.pos_check_precision) and \
                                   (self.brusher_pos <= self.brusher_x0 + self.pos_check_precision)
                self.motors.left_puller.move(self.motors.left_puller.MoveDirection(self.puller_left_dir))
                self.motors.right_puller.move(self.motors.right_puller.MoveDirection(self.puller_right_dir))
            
                self.pulling = True
                print("Flame hold done")
                print("Started pulling")
                self.time_init_pull = time.time()
    
    def check_pulling(self):
        if self.rel_pull:
            total_pull_vel = self.motors.left_puller.pull_vel + self.motors.right_puller.pull_vel
            pull_time = time.time() - self.time_init_pull
            self.total_pulled = total_pull_vel*pull_time
        else:
            self.total_pulled = np.abs(self.left_puller_xinit - self.puller_left_pos) + \
                                np.abs(self.right_puller_xinit - self.puller_right_pos)
        
        if self.pullers_adaptive_vel:
            if self.pullers_av_idx < len(self.pullers_av_threshold):
                if self.total_pulled >= self.pullers_av_threshold[self.pullers_av_idx]:
                    self.pullers_av_idx += 1
                    # Update the base pull_vel in the motor objects
                    # so that check_brushing follower picks up the new speed
                    self.motors.left_puller.pull_vel *= self.pullers_av_factor
                    self.motors.right_puller.pull_vel *= self.pullers_av_factor
                    
                    self.pull_vel = self.motors.left_puller.pull_vel
                    
                    if not self.rel_pull:
                        # Only handle discrete moves here if NOT in rel_pull mode
                        self.motors.left_puller.stop(0)
                        self.motors.right_puller.stop(0)
                        self.motors.left_puller.set_velocity(self.pull_vel)
                        self.motors.right_puller.set_velocity(self.pull_vel)
                        self.motors.left_puller.move(self.motors.left_puller.MoveDirection(self.puller_left_dir))
                        self.motors.right_puller.move(self.motors.right_puller.MoveDirection(self.puller_right_dir))
        
        if self.auto_stop and self.total_pulled >= self.hotzone_function[0][-1]:
            self.stopping = True
            print("Pulling ending...")
    
    def check_brushing(self, mode=0):
        # Predictive Relative Pulling Logic
        # Instead of reactive P-control on integrated position, we calculate
        # exact target positions for t and t+dt to derive feed-forward velocity.
        
        hz = np.interp(self.total_pulled, self.hotzone_function[0], self.hotzone_function[1])
        hz_l = self.brusher_x0 - hz/2
        hz_r = self.brusher_x0 + hz/2
        
        if self.rel_pull:
            # 1. Time housekeeping for integration (still needed for fake_brusher_pos)
            self.time_brush_check_0 = self.time_brush_check_1
            self.time_brush_check_1 = time.time()
            dt = self.time_brush_check_1 - self.time_brush_check_0
            if dt > 0.1: dt = 0.0 # Safety clamp
            
            # 2. Update Virtual Brusher State (The "Conductor")
            # This remains the source of truth for "where the flame should be relative to the fiber"
            
            # Current boundary check
            l = -hz/2.0 + self.flame_size/2.0
            r = hz/2.0 - self.flame_size/2.0
            
            if l >= r:
                 # Hotzone too small to brush. Keep centered.
                 self.fake_brusher_pos = 0.0
                 # Do not record edges, as there is no brushing oscillation
            else:
                # Predictive Move
                step = self.brusher_dir * self.motors.brusher.vel * dt
                next_pos = self.fake_brusher_pos + step
                
                # Check for boundary crossing BEFORE applying step
                if (self.brusher_dir == 1 and next_pos > r):
                    # We would overshoot. Stop at the wall and flip.
                    self.fake_brusher_pos = r
                    self.brusher_dir = -1
                    self.rhz_edges.append(self.fake_brusher_pos + self.flame_size/2.0) # Record exact limit (Right Edge)
                elif (self.brusher_dir == -1 and next_pos < l):
                    # We would undershoot. Stop at the wall and flip.
                    self.fake_brusher_pos = l
                    self.brusher_dir = 1
                    self.rhz_edges.append(self.fake_brusher_pos - self.flame_size/2.0) # Record exact limit (Left Edge)
                else:
                    # Safe to move
                    self.fake_brusher_pos = next_pos

            # 3. Predict Future State (Look-Ahead)
            # We want to know where the motors *should* be 100ms from now
            dt_pred = 0.1 
            
            # Predict Pulling Progress
            if self.pulling:
                # Assuming constant velocity for prediction window
                # Note: If adaptive velocity changes, it will be caught in next loop
                pull_vel_total = self.motors.left_puller.pull_vel + self.motors.right_puller.pull_vel
                fut_total_pulled = self.total_pulled + pull_vel_total * dt_pred
            else:
                fut_total_pulled = self.total_pulled
                
            # Predict Hotzone at t+dt
            fut_hz = np.interp(fut_total_pulled, self.hotzone_function[0], self.hotzone_function[1])
            fut_l = -fut_hz/2.0 + self.flame_size/2.0
            fut_r = fut_hz/2.0 - self.flame_size/2.0
            
            # Predict Virtual Brusher at t+dt
            # We must handle the reflection if it hits the wall during dt_pred
            # This is a simplified reflection: it assumes at most one bounce per dt_pred (valid for low speeds)
            fut_brusher_pos = self.fake_brusher_pos + self.brusher_dir * self.motors.brusher.vel * dt_pred
            
            # Check for reflection in prediction
            # (Note: We don't change actual direction here, just the predicted position)
            if (self.brusher_dir == 1 and fut_brusher_pos > fut_r):
                overshoot = fut_brusher_pos - fut_r
                fut_brusher_pos = fut_r - overshoot
            elif (self.brusher_dir == -1 and fut_brusher_pos < fut_l):
                overshoot = fut_l - fut_brusher_pos
                fut_brusher_pos = fut_l + overshoot
                
            # 4. Calculate Absolute Target Positions (t and t+dt)
            # Center of the "Taper" in machine coordinates moves to the LEFT (Puller L decreases, Puller R decreases)
            # We use total_pulled / 2.0 as the translation of the center frame (assuming symmetric pulling)
            
            curr_target_l = - (self.total_pulled / 2.0) - self.fake_brusher_pos
            curr_target_r = - (self.total_pulled / 2.0) + self.fake_brusher_pos
            
            fut_target_l = - (fut_total_pulled / 2.0) - fut_brusher_pos
            fut_target_r = - (fut_total_pulled / 2.0) + fut_brusher_pos
            
            # 5. Calculate Feed-Forward Velocity
            v_l_ideal = (fut_target_l - curr_target_l) / dt_pred
            v_r_ideal = (fut_target_r - curr_target_r) / dt_pred
            
            # 6. Update Golden Position (Hard Sync)
            # In the previous code, this was an integral that could drift.
            # Now we define it based on the Theoretical Function.
            # But we need an offset! The motor is at 80mm, not at "-TotalPulled".
            # We use the initial motor positions captured at start.
            
            if not hasattr(self, 'l_golden_pos') or self.l_golden_pos == 0.0:
                self.l_golden_pos = self.puller_left_pos
                self.r_golden_pos = self.puller_right_pos
                # Define offset based on first run
                self.l_golden_offset = self.puller_left_pos - curr_target_l
                self.r_golden_offset = self.puller_right_pos - curr_target_r

            if self.pulling and not getattr(self, 'pull_started_init', False):
                self.pull_started_init = True
                # Re-sync offsets when pulling starts to prevent "jump"
                self.l_golden_offset = self.puller_left_pos - curr_target_l
                self.r_golden_offset = self.puller_right_pos - curr_target_r

            # Calculate Absolute Golden Positions for P-Control
            abs_target_l = curr_target_l + getattr(self, 'l_golden_offset', 0)
            abs_target_r = curr_target_r + getattr(self, 'r_golden_offset', 0)
            
            self.l_golden_pos = abs_target_l
            self.r_golden_pos = abs_target_r
            
            # 7. P-Control Correction
            # V_cmd = V_ideal + Kp * (Target - Actual)
            v_l_cmd = v_l_ideal + self.tracking_kp * (self.l_golden_pos - self.puller_left_pos)
            v_r_cmd = v_r_ideal + self.tracking_kp * (self.r_golden_pos - self.puller_right_pos)
            
            # Clamp for safety
            v_l_cmd = np.clip(v_l_cmd, -12, 12)
            v_r_cmd = np.clip(v_r_cmd, -12, 12)
            
            # 8. Active Velocity Update (Throttled)
            l_v_abs = max(0.001, abs(v_l_cmd))
            r_v_abs = max(0.001, abs(v_r_cmd))
            
            l_dir = self.motors.left_puller.MoveDirection.POSITIVE if v_l_cmd > 0 else self.motors.left_puller.MoveDirection.NEGATIVE
            r_dir = self.motors.right_puller.MoveDirection.POSITIVE if v_r_cmd > 0 else self.motors.right_puller.MoveDirection.NEGATIVE
            
            # Check Left Motor
            l_v_diff = abs(l_v_abs - self.l_last_cmd_v)
            l_dir_change = (l_dir != self.l_last_cmd_dir)
            
            if l_v_diff > 0.005 or l_dir_change:
                self.motors.left_puller.set_velocity(l_v_abs)
                self.motors.left_puller.move(l_dir, stop=False)
                self.l_last_cmd_v = l_v_abs
                self.l_last_cmd_dir = l_dir
                
            # Check Right Motor
            r_v_diff = abs(r_v_abs - self.r_last_cmd_v)
            r_dir_change = (r_dir != self.r_last_cmd_dir)
            
            if r_v_diff > 0.005 or r_dir_change:
                self.motors.right_puller.set_velocity(r_v_abs)
                self.motors.right_puller.move(r_dir, stop=False)
                self.r_last_cmd_v = r_v_abs
                self.r_last_cmd_dir = r_dir
        else:
            # Fallback for Standard Mode (Not Relative)
            if not self.rel_pull and mode == 0:
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
            if not self.rel_pull and mode != 0:
                if hz - self.flame_size >= self.motors.brusher.min_span:
                    # This is a work in progress, possibly useless
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
                v_pull = self.pull_vel
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
            if self.rel_pull:
                self.time_brush_check_0 = self.time_brush_check_1
                self.time_brush_check_1 = time.time()
                t0 = self.time_brush_check_0
                t1 = self.time_brush_check_1
                self.fake_brusher_pos += self.brusher_dir*(t1 - t0)*self.motors.brusher.vel
                
                l = -self.hotzone_function[1][-1]/2.0
                r = self.hotzone_function[1][-1]/2.0
                if (self.fake_brusher_pos <= l  or self.fake_brusher_pos >= r):
                    stop_ok = True
            else:
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
        elif self.loop_looped and not self.loop_tensioning:
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
                
                # Verify shape and transpose if necessary (N, 2) -> (2, N)
                if self.hotzone_function.shape[0] > 2 and self.hotzone_function.shape[1] == 2:
                    self.hotzone_function = self.hotzone_function.T
                    
            except:
                raise Exception("Error: hotzone_function must be a 2D list or numpy array.")
            
            # Now expect shape[0] == 2 (2 rows: Distance, Span)
            if self.hotzone_function.shape[0] == 2:
                self.running_process = True
            else:
                raise Exception(f"Error: hotzone_function shape {self.hotzone_function.shape} invalid. Must be (2, N) or (N, 2).")
            
            # Set initial time
            self.time_init = time.time()
            self.time_now = time.time()
        else:
            print("Error: not all motors initialized correctly.")
        
    def stop_pulling(self):
        self.standby = True
        
        # Stop all motors
        self.motors.brusher.stop()
        self.motors.left_puller.stop()
        self.motors.right_puller.stop()
        
        # Retract Flame I/O
        if self.flame_io_moveback:
            self.motors.flame_io.set_velocity(self.fio_v0)
            self.flameIO_moving = False
            time.sleep(0.1)
        self.motors.flame_io.go_to(0.0)     
        
        # Reset pullers velocity
        self.motors.left_puller.set_velocity(self.motors.left_puller.vel)
        self.motors.right_puller.set_velocity(self.motors.right_puller.vel)
        time.sleep(0.1)
        
        # Reset all leftover stuff
        self.reset_pull_stats()
        
        # Go to standby mode
        self.running_process = True
        self.standby = True
        self.motors.flame_io.go_to(0.0)
        
    def get_time_left(self, total_to_pull=-1, total_pulled=-1): 
        if total_to_pull < 0:
            total_to_pull = self.hotzone_function[0][-1]
        if total_pulled < 0:
            total_pulled = self.total_pulled
        to_pull = total_to_pull - total_pulled
        v0 = (self.motors.left_puller.pull_vel + self.motors.right_puller.pull_vel)
        t = 0
        if self.pullers_adaptive_vel:
            i0 = 0
            if total_pulled > 0:
                i0 = self.pullers_av_idx
            if i0 < len(self.pullers_av_threshold):
                v = v0*(self.pullers_av_factor**i0)
                th = np.min([total_to_pull, self.pullers_av_threshold[i0]])
                x = th - total_pulled
                t += x/v
            for i in range(i0, len(self.pullers_av_threshold) - 1):
                if self.pullers_av_threshold[i] <= total_to_pull:
                    v = v0*(self.pullers_av_factor**(i + 1))
                    th = np.min([total_to_pull, self.pullers_av_threshold[i + 1]]) 
                    x = th - self.pullers_av_threshold[i]
                    t += x/v
            i1 = len(self.pullers_av_threshold) - 1
            if self.pullers_av_threshold[i1] < total_to_pull:
                v = v0*(self.pullers_av_factor**(i1 + 1))
                x = total_to_pull - np.max([self.pullers_av_threshold[i1], total_pulled])
                t += x/v
        else:
            x = to_pull
            t = x/v0
        
        return t