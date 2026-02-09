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
    recorded_points = [] # [time, measured_span]
    
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
            # At what time will the motor REACH the next target?
            # Roughly: t_now + 0.3s (wait) + move_time (~0.5s) + overhead
            # The oscillation was ~0.5mm, implying ~1.0s lag.
            # Let's target t_now + 0.9s
            t_predict = (time.time() - start_time_global) + 0.9
            
            # Interpolate required span for the predicted time
            current_span = np.interp(t_predict, ref_times, ref_spans)
            amplitude = current_span / 2.0
            
            # Calculate Pulling Drift for the TARGET time
            # We must target where the center WILL BE.
            pull_offset = v_pull_each * t_predict
            
            # Calculate targets: Center + Drift +/- Amplitude
            # Standard Taper Pulling: Motors move APART.
            # Assuming L:0..50..100 and R:0..50..100 is not valid, usually independent coordinates.
            # If Left Puller (0-50) is at 50, Pulling means going to 0 (Negative).
            # If Right Puller (0-50) is at 50, Pulling means going to 0? Or is it 50..100?
            # Let's Stick to the "Expanding" logic relative to a virtual center if that's what we tested.
            # But the previous test had L=C+P (Increasing), R=C-P (Decreasing).
            # If C=50, L->60, R->40. They move TOWARDS each other (Collision risk).
            # I will SWAP THIS to be Safe/Standard: L moves -, R moves +.
            # L = C - P. R = C + P.
            
            if current_direction == 1:
                left_target = center_pos - pull_offset + amplitude
                right_target = center_pos + pull_offset - amplitude
            else:
                left_target = center_pos - pull_offset - amplitude
                right_target = center_pos + pull_offset + amplitude
                
            # Pre-load Targets with error checking
            err_l = left_puller.set_move_absolute_position(left_target)
            err_r = right_puller.set_move_absolute_position(right_target)
            
            if err_l != 0 or err_r != 0:
                print(f"Warning: Set Target Failed (L:{err_l}, R:{err_r}) at t={current_sim_time:.2f}")

            # Fire Trigger (Pulse High)
            left_puller.set_trigger_out_states(1)
            
            # Robust Busy-Wait Sequence:
            # 1. Wait long enough for the driver/hardware to register "Moving"
            #    We use 0.3s as specified by user/previous experience.
            time.sleep(0.3)
            
            # 2. Wait for it to FINISH (IDLE state)
            #    If it never became busy, this returns immediately.
            while left_puller.motor_moving() or right_puller.motor_moving():
                time.sleep(0.05) 
            
            # 3. Reset Trigger Pulse and add a tiny delay to ensure the next 
            #    transition is seen as a new edge.
            left_puller.set_trigger_out_states(0)
            time.sleep(0.05)
            
            # Record Data at the end of the stroke
            t_now_real = time.time() - start_time_global
            current_sim_time = t_now_real 
            
            l_pos = left_puller.get_position()
            r_pos = right_puller.get_position()
            
            # Calculate Measured Span: $|L - (Center - Pull)| + |R - (Center + Pull)|$
            # Matching the corrected direction above (L-, R+).
            m_pull = v_pull_each * t_now_real
            m_span = abs(l_pos - (center_pos - m_pull)) + abs(r_pos - (center_pos + m_pull))
            
            recorded_points.append([t_now_real, m_span])
            
            # Flip direction
            current_direction *= -1

    finally:
        # Reset
        print("\nTest finished/interrupted. Resetting.")
        left_puller.set_trigger_config(0, 0)
        right_puller.set_trigger_config(0, 0)
        left_puller.disconnect()
        right_puller.disconnect()
    
    print("Generating plot...")
    path_data = np.array(recorded_points)
    
    if len(path_data) > 0:
        times_rec = path_data[:, 0]
        spans_rec = path_data[:, 1]
        
        plt.figure(figsize=(12, 6))
        
        # Plot Reference Profile
        plt.plot(ref_times, ref_spans, 'k-', label='Reference Profile', alpha=0.5, linewidth=1)
        
        # Plot Measured Data
        plt.plot(times_rec, spans_rec, 'r.', label='Measured Span', markersize=4)
        
        plt.title("Advanced Brushing Test: Dynamic Amplitude")
        plt.xlabel("Time (s)")
        plt.ylabel("Total Span (mm)")
        plt.legend()
        plt.grid(True)
        
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
