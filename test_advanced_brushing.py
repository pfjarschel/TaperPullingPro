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
    
    start_time_global = time.time()
    current_sim_time = 0.0
    
    try:
        # Loop until we exceed the profile time
        while current_sim_time < ref_times[-1]:
            
            # Interpolate current required span
            current_span = np.interp(current_sim_time, ref_times, ref_spans)
            amplitude = current_span / 2.0
            
            # Calculate targets
            if current_direction == 1:
                left_target = center_pos + amplitude
                right_target = center_pos - amplitude
            else:
                left_target = center_pos - amplitude
                right_target = center_pos + amplitude
                
            # Pre-load Targets
            left_puller.set_move_absolute_position(left_target)
            right_puller.set_move_absolute_position(right_target)
            
            # Fire Trigger
            left_puller.set_trigger_out_states(1)
            
            # Minimal Wait: 100ms then check moving
            time.sleep(0.1)
            
            # Wait for move to complete
            while left_puller.motor_moving() or right_puller.motor_moving():
                # We can do a tight poll here if we wanted to be super precise, 
                # but "motor_moving" check usually has ~10ms latency anyway.
                time.sleep(0.01) 
            
            # End Pulse
            left_puller.set_trigger_out_states(0)
            
            # Record Data ONCE at the end of the stroke (as requested)
            t_now_real = time.time() - start_time_global
            current_sim_time = t_now_real # Update simulation time based on real elapsed time
            
            l_pos = left_puller.get_position()
            r_pos = right_puller.get_position()
            
            # Calculate "Real Amplitude" (Sum of absolute diffs from center)
            # This represents the total span achieved at this point
            measured_span = abs(l_pos - center_pos) + abs(r_pos - center_pos)
            
            recorded_points.append([t_now_real, measured_span])
            
            # print(f"T: {t_now_real:.2f}s | Target Span: {current_span:.3f} | Measured: {measured_span:.3f}")
            
            # Flip direction
            current_direction *= -1
            
            # Small safety sleep? Maybe unneeded if we want max speed. 
            # User said "Minimize the time the motors remain stopped".
            # So no extra sleep.

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
