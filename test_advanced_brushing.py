import TaperPulling.TaperPullingMotors as TPM
import time
import sys
import matplotlib.pyplot as plt
import numpy as np
import os

def load_amplitude_profile(filename):
    """
    Loads the amplitude profile from the given filename.
    Format: Space-separated values.
    Col 1: Total Pulled Distance (mm)
    Col 2: Total Span (mm)
    
    Returns:
        times (np.array): Time points assuming 0.25 mm/s pulling speed.
        spans (np.array): Total span for each point.
    """
    try:
        data = np.loadtxt(filename)
        distances = data[:, 0]
        spans = data[:, 1]
        
        # Convert distance to time: Time = Dist / 0.25
        times = distances / 0.25
        return times, spans
    except Exception as e:
        print(f"Error loading profile: {e}")
        return None, None

def calculate_move_duration(distance, v=5.0, a=40.0):
    """
    Calculates time to move 'distance' with trapezoidal velocity profile.
    """
    if distance == 0:
        return 0.0
        
    # Time to reach max velocity: t_acc = v / a
    t_acc = v / a
    # Distance covered during accel: d_acc = 0.5 * a * t_acc^2
    d_acc = 0.5 * a * (t_acc**2)
    
    # Total distance for full triangular profile (accel + decel)
    d_tri = 2 * d_acc
    
    if distance >= d_tri:
        # Trapezoidal Profile (Hit max velocity)
        # Time at constant velocity
        d_const = distance - d_tri
        t_const = d_const / v
        return 2*t_acc + t_const
    else:
        # Triangular Profile (Did not reach max velocity)
        # distance = a * t_peak^2 (since d = 0.5*a*t^2 accel + 0.5*a*t^2 decel)
        # t_peak = sqrt(distance / a)
        # Total time = 2 * t_peak
        t_peak = np.sqrt(distance / a)
        return 2 * t_peak

def run_test():
    print("=== Advanced Brushing Synchronization Test ===")
    
    # Load Profile
    profile_file = "amp_list_test.txt"
    if not os.path.exists(profile_file):
        print(f"Error: {profile_file} not found!")
        return

    print(f"Loading profile from {profile_file}...")
    ref_times, ref_spans = load_amplitude_profile(profile_file)
    if ref_times is None:
        return
    
    print(f"Loaded {len(ref_times)} points. Total duration: {ref_times[-1]:.2f} s")

    print("Initializing motors...")
    
    # 1. Initialize Motors
    left_puller = TPM.LeftPuller()
    right_puller = TPM.RightPuller()
    
    left_puller.connect()
    right_puller.connect()
    
    if not left_puller.ok or not right_puller.ok:
        print("Failed to connect to motors.")
        sys.exit(1)

    # Full Initialization
    print("Performing full initialization...")
    left_puller.initialize()
    right_puller.initialize()
    
    # Set parameters (Higher Acceleration as requested)
    vel = 5.0 # mm/s
    accel = 40.0 # mm/s^2 
    
    left_puller.set_velocity(vel)
    left_puller.set_acceleration(accel)
    right_puller.set_velocity(vel)
    right_puller.set_acceleration(accel)
    
    print(f"Motors initialized. Vel: {vel} mm/s, Accel: {accel} mm/s^2")
    
    # 2. Setup Reference Positions
    center_pos = 50.0
    
    # Go to start position
    print(f"Moving both motors to center position: {center_pos} mm")
    left_puller.go_to(center_pos)
    right_puller.go_to(center_pos)
    
    # Wait for completion
    while left_puller.motor_moving() or right_puller.motor_moving():
        time.sleep(0.1)
        
    print("Center position reached. Configuring Triggers...")
    time.sleep(1.0)
    
    # 3. Configure Triggers
    left_puller.set_trigger_out_states(0) 
    left_puller.set_trigger_config(3, 0)
    right_puller.set_trigger_config(3, 0)
    
    # Data storage
    recorded_endpoints = [] # [time, span, l_pos, r_pos] - Only at end of moves
    recorded_trajectory = [] # [time, span, l_pos, r_pos] - High freq during moves
    
    current_direction = 1 # 1: Outward, -1: Inward
    
    # User-specified Pulling Velocity: 0.25 mm/s total -> 0.125 mm/s each motor
    v_pull_each = 0.125 
    
    start_time_global = time.time()
    current_sim_time = 0.0
    
    print("\nStarting Brushing Loop. Pulling Velocity: 0.25 mm/s total.")
    print("Each half-cycle includes a 0.3s pre-wait and a robust busy-wait.")

    try:
        # Loop until we exceed the profile time
        while current_sim_time < ref_times[-1]:
            # ITERATIVE TARGET PREDICTION
            # We need to find t_arrival such that:
            # t_arrival = t_now + t_wait + move_time(Target(t_arrival))
            
            t_now_loop = time.time() - start_time_global
            
            # 1. READ POSITIONS FIRST (Needed for direction check)
            l_curr = left_puller.get_position()
            r_curr = right_puller.get_position()
            
            # Initial Guess: Previous duration or default
            t_wait_overhead = 0.3 + 0.15 # 0.3 wait + 0.15 accel/latency
            
            # Determine Direction based on Current Position relative to Pulling Center
            # This self-corrects if we get out of sync.
            
            # Current Pulling Center at t_now
            pull_now = v_pull_each * t_now_loop
            c_l_now = center_pos - pull_now
            c_r_now = center_pos - pull_now # BOTH centers move towards 0 (Outer Limit)
            
            # Check L motor position relative to its center
            if l_curr < c_l_now:
                # Left motor is to the LEFT of its center (More Negative)
                # It must be at the "Outward" limit.
                # Next move should be INWARD (Return).
                current_direction = -1 
            else:
                # Left motor is to the RIGHT of its center (Less Negative)
                # It must be at the "Inward" limit (or center).
                # Next move should be OUTWARD.
                current_direction = 1
                
            # Override for first loop (t=0, pos=center)
            if t_now_loop < 0.5:
                current_direction = 1 # Force Outward First
            
            # First pass guess
            t_guess = t_now_loop + t_wait_overhead + 0.5 
                
            for _ in range(3): # Converge in 2-3 steps
                # 1. Lookup Params at Guess
                curr_span = np.interp(t_guess, ref_times, ref_spans)
                amp = curr_span / 2.0
                pull = v_pull_each * t_guess
                
                # 2. Calculate Tentative Targets
                # Pulling Center for both is (Start - Pull)
                c_l = center_pos - pull
                c_r = center_pos - pull
                
                if current_direction == 1:
                    # Outward: Away from Gap Center
                    # L (Left): moves Left (-) -> CenterL - Amp
                    # R (Right): moves Right (+) -> CenterR + Amp (Phase Flipped)
                    l_targ = c_l - amp
                    r_targ = c_r + amp
                else:
                    # Inward: Towards Gap Center
                    l_targ = c_l + amp
                    r_targ = c_r - amp
                    
                # 3. Calculate Move Distance (vs Current Physical Pos)
                # Note: We don't want to poll get_position() inside the calculation loop (slow).
                # We assume current pos is roughly where we left it (Target of previous move).
                # OR we read it once.
                # Let's read it once outside.
                
                # 4. Calculate Duration
                # Max dist / V
                # We need actual current positions for accurate duration.
                # We read them at start of loop (l_curr, r_curr).
                pass 

            # Read positions ONCE for calculation (ALREADY DONE AT START)
            # l_curr = left_puller.get_position()
            # r_curr = right_puller.get_position()
            
            # Iteration Loop
            t_pred = t_now_loop + 1.0 # Start with dummy
            
            for _ in range(3):
                # Lookup
                current_span = np.interp(t_pred, ref_times, ref_spans)
                amplitude = current_span / 2.0
                pull_offset = v_pull_each * t_pred
                
                # Targets
                # Center for both: Start - Pull
                c_l = center_pos - pull_offset
                c_r = center_pos - pull_offset
                
                if current_direction == 1:
                    # Simulation proved this order is stable/correct (Outward First)
                    # L goes Left (More Negative) -> (Center - Pull) - Amp
                    # R goes Right (More Positive) -> (Center + Pull) + Amp (OLD) -> (Center - Pull) + Amp (Phase Flipped)
                    l_try = c_l - amplitude
                    r_try = c_r + amplitude
                else:
                    # Inward (Return)
                    l_try = c_l + amplitude
                    r_try = c_r - amplitude
                
                # Time
                dist_l = abs(l_try - l_curr)
                dist_r = abs(r_try - r_curr)
                max_dist = max(dist_l, dist_r)
                
                # Use Kinematic Calculation
                duration = calculate_move_duration(max_dist, v=5.0, a=40.0)
                
                # Refine Prediction
                t_pred = t_now_loop + t_wait_overhead + duration
            
            # Final Targets
            left_target = l_try
            right_target = r_try
            
            # Pre-load Targets with error checking
            err_l = left_puller.set_move_absolute_position(left_target)
            err_r = right_puller.set_move_absolute_position(right_target)
            
            if err_l != 0 or err_r != 0:
                print(f"Warning: Set Target Failed (L:{err_l}, R:{err_r}) at t={current_sim_time:.2f}")

            # Fire Trigger (Pulse Train - Burst of 3)
            # This ensures that if the first is missed, the next catches it immediately.
            for _ in range(3):
                left_puller.set_trigger_out_states(1)
                time.sleep(0.01) # 10ms pulse
                left_puller.set_trigger_out_states(0)
                time.sleep(0.01) # 10ms gap
            
            # Wait for driver to register (Latency)
            time.sleep(0.3)
            
            # TRIGGER VERIFICATION & RETRY
            # Check if motors actually started. If not, re-fire.
            retry_count = 0
            while retry_count < 3:
                l_moving = left_puller.motor_moving()
                r_moving = right_puller.motor_moving()
                
                if l_moving and r_moving:
                    break # Both moving, good.
                
                # Identify who missed it
                missing = []
                if not l_moving: missing.append("LEFT")
                if not r_moving: missing.append("RIGHT")
                
                print(f"WARNING: Missed Trigger! {missing}. Retrying ({retry_count+1}/3)...")
                
                # Reset and Re-fire
                left_puller.set_trigger_out_states(0)
                time.sleep(0.05)
                left_puller.set_trigger_out_states(1)
                
                # Wait again for driver to register
                time.sleep(0.3)
                retry_count += 1
            
            while left_puller.motor_moving() or right_puller.motor_moving():
                # High-Freq Logging
                t_log = time.time() - start_time_global
                l_log = left_puller.get_position()
                r_log = right_puller.get_position()
                
                # Calculate metric for consistency
                m_pull_log = v_pull_each * t_log
                span_log = abs(l_log - (center_pos - m_pull_log)) + abs(r_log - (center_pos + m_pull_log))
                
                # Store: [Time, Span, LeftPos, RightPos]
                recorded_trajectory.append([t_log, span_log, l_log, r_log])
                time.sleep(0.05) 
            
            # Check for Premature Stop / Errors
            l_final = left_puller.get_position()
            r_final = right_puller.get_position()
            
            # Tolerances
            tol = 0.05
            if abs(l_final - left_target) > tol:
                stat_l = left_puller.get_status_bits()
                print(f"!!! LEFT MOTOR STOPPED EARLY: Pos={l_final:.3f}, Tgt={left_target:.3f}, Diff={l_final-left_target:.3f}, Status=0x{stat_l:08X}")
                
            if abs(r_final - right_target) > tol:
                stat_r = right_puller.get_status_bits()
                print(f"!!! RIGHT MOTOR STOPPED EARLY: Pos={r_final:.3f}, Tgt={right_target:.3f}, Diff={r_final-right_target:.3f}, Status=0x{stat_r:08X}")
            
            left_puller.set_trigger_out_states(0)
            time.sleep(0.05)
            
            # End of Move Analysis
            t_now_real = time.time() - start_time_global
            current_sim_time = t_now_real 
            
            # Debug Timing
            # time_err = t_now_real - t_pred
            # print(f"Time Err: {time_err:.3f}s") # User can check stdout if needed
            
            l_pos = left_puller.get_position()
            r_pos = right_puller.get_position()
            
            # Calculate Measured Span matches t_now_real
            m_pull = v_pull_each * t_now_real
            c_l_m = center_pos - m_pull
            c_r_m = center_pos - m_pull
            
            # Span = Sum of deviations from respective pulling centers
            m_span = abs(l_pos - c_l_m) + abs(r_pos - c_r_m)
            
            recorded_endpoints.append([t_now_real, m_span, l_pos, r_pos])
            # Also add to trajectory for continuity
            recorded_trajectory.append([t_now_real, m_span, l_pos, r_pos])
            
            # Flip direction (REMOVED: Now handled by state detection at start of loop)
            # current_direction *= -1

    finally:
        # Reset
        print("\nTest finished/interrupted. Resetting.")
        left_puller.set_trigger_config(0, 0)
        right_puller.set_trigger_config(0, 0)
        left_puller.disconnect()
        right_puller.disconnect()
    
    print("Generating plot...")
    print("Generating plot...")
    
    # 1. Endpoint Data (For Span Plot)
    ep_data = np.array(recorded_endpoints)
    if len(ep_data) > 0:
        t_ep = ep_data[:, 0]
        s_ep = ep_data[:, 1]
        dist_ep = t_ep * 0.25
    else:
        dist_ep = []
        s_ep = []
        
    # 2. Trajectory Data (For Motor Plot)
    tj_data = np.array(recorded_trajectory)
    if len(tj_data) > 0:
        t_tj = tj_data[:, 0]
        l_pos_tj = tj_data[:, 2]
        r_pos_tj = tj_data[:, 3]
        dist_tj = t_tj * 0.25
        
        # Calculate Reference Centers for Trajectory
        center_l_ref = center_pos - (t_tj * 0.125)
        center_r_ref = center_pos - (t_tj * 0.125) # Right also decreases
    else:
        dist_tj = []
        l_pos_tj = []
        r_pos_tj = []
        center_l_ref = []
        center_r_ref = []
        
    if len(ep_data) > 0:
        ref_dist = ref_times * 0.25
        
        plt.figure(figsize=(10, 10))
        
        # Subplot 1: Span vs Pulled Distance (Endpoints Only)
        plt.subplot(2, 1, 1)
        plt.plot(ref_dist, ref_spans, 'k-', label='Reference', alpha=0.5, linewidth=1)
        plt.plot(dist_ep, s_ep, 'r.', label='Measured Span (Endpoints)', markersize=4)
        plt.title("Advanced Brushing Test: Span vs Pulled Distance")
        plt.xlabel("Total Pulled Distance (mm)")
        plt.ylabel("Total Span (mm)")
        plt.legend()
        plt.grid(True)
        
        # Subplot 2: Motor Positions vs Pulled Distance (Full Trajectory)
        plt.subplot(2, 1, 2)
        plt.plot(dist_tj, l_pos_tj, 'b-', label='Left Motor', linewidth=1)
        plt.plot(dist_tj, r_pos_tj, 'g-', label='Right Motor', linewidth=1)
        
        # Add Reference Pulling Centers
        plt.plot(dist_tj, center_l_ref, 'b--', alpha=0.3, label='Left Center')
        plt.plot(dist_tj, center_r_ref, 'g--', alpha=0.3, label='Right Center')
        
        plt.title("Individual Motor Trajectories (Full History)")
        plt.xlabel("Total Pulled Distance (mm)")
        plt.ylabel("Position (mm)")
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()
        plot_file = "advanced_brushing_test.png"
        plt.savefig(plot_file)
        print(f"Plot saved to {plot_file}")
    else:
        print("No data recorded.")

if __name__ == "__main__":
    try:
        run_test()
    except Exception as e:
        print(f"An error occurred: {e}")
        import traceback
        traceback.print_exc()
