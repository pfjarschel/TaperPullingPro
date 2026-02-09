import TaperPulling.TaperPullingMotors as TPM
import time
import sys
import matplotlib.pyplot as plt
import numpy as np

def run_test():
    print("=== Brushing Synchronization Test ===")
    print("Initializing motors...")
    
    # 1. Initialize Motors
    # LEFT is MASTER (Out) -> Connected to BOTH Inputs
    # RIGHT is SLAVE (In)
    left_puller = TPM.LeftPuller()
    right_puller = TPM.RightPuller()
    
    left_puller.connect()
    right_puller.connect()
    
    if not left_puller.ok or not right_puller.ok:
        print("Failed to connect to motors.")
        sys.exit(1)

    # Full Initialization as requested
    print("Performing full initialization...")
    left_puller.initialize()
    right_puller.initialize()
    
    # Set safe standard parameters
    vel = 5.0 # mm/s
    accel = 5.0 # mm/s^2
    
    left_puller.set_velocity(vel)
    left_puller.set_acceleration(accel)
    right_puller.set_velocity(vel)
    right_puller.set_acceleration(accel)
    
    print("Motors connected and initialized.")
    
    # 2. Setup Reference Positions
    center_pos = 50.0
    amplitude = 2.5 # Half of 5mm total swing
    num_cycles = 3
    
    # Go to start position
    print(f"Moving both motors to start position: {center_pos} mm")
    left_puller.go_to(center_pos)
    right_puller.go_to(center_pos)
    
    # Wait for completion
    while left_puller.motor_moving() or right_puller.motor_moving():
        time.sleep(0.1)
        
    print("Start position reached.")
    time.sleep(1.0)
    
    # 3. Configure Triggers
    # Mode 3 (Standard Absolute) is mapped to Mode 5 internally for BMC
    # Polarity 0 (Rising Edge/High)
    print("Configuring triggers...")
    left_puller.set_trigger_out_states(0) # Ensure low initially
    
    left_cfg = left_puller.set_trigger_config(3, 0)
    right_cfg = right_puller.set_trigger_config(3, 0)
    
    print(f"Trigger Config Results - Left: {left_cfg}, Right: {right_cfg}")
    
    # Data storage for plotting
    # Structure: [time, left_pos, right_pos]
    recorded_data = []
    
    current_direction = 1 # 1: Outward, -1: Inward
    
    start_time_global = time.time()
    
    for i in range(num_cycles * 2):
        print(f"\n--- Half-Cycle {i+1}/{num_cycles*2} ---")
        
        # Calculate targets
        # Left moves +delta, Right moves -delta (or vice versa)
        # Verify directions:
        # If "Outward": Left goes <, Right goes > ? Or opposite?
        # Let's assume:
        #  - Left moves Positive
        #  - Right moves Negative
        
        if current_direction == 1:
            left_target = center_pos + amplitude
            right_target = center_pos - amplitude
            print(f"Direction: POSITIVE (Left -> {left_target}, Right -> {right_target})")
        else:
            left_target = center_pos - amplitude
            right_target = center_pos + amplitude
            print(f"Direction: NEGATIVE (Left -> {left_target}, Right -> {right_target})")
            
        # 4. Pre-load Targets
        left_puller.set_move_absolute_position(left_target)
        right_puller.set_move_absolute_position(right_target)
        
        # Give a small delay for comms
        time.sleep(0.1)
        
        # Check current positions before trigger
        l_curr = left_puller.get_position()
        r_curr = right_puller.get_position()
        print(f"Pre-Trigger Pos - Left: {l_curr:.4f}, Right: {r_curr:.4f}")
        
        # 5. Fire Trigger
        print("Firing trigger...")
        t_fire = time.time() - start_time_global
        
        # Pulse: High -> Delay -> Low
        left_puller.set_trigger_out_states(1)
        # Note: Triggers usually fire on edge. 
        # We need to be fast here to capture initial movement.
        
        # 6. High-Speed Polling Loop
        # Poll for ~ 2 seconds (enough for 2.5mm move at 5mm/s + accel)
        timeout = 2.0 
        t_start_loop = time.time()
        
        temp_data = []
        while (time.time() - t_start_loop) < timeout:
            t_now = time.time() - start_time_global
            l_pos = left_puller.get_position()
            r_pos = right_puller.get_position()
            
            # recorded_data.append([t_now, l_pos, r_pos])
            temp_data.append([t_now, l_pos, r_pos])
            
            # Try to poll as fast as possible, but yield slightly
            # If the driver is slow, this won't help much, but we try.
            # time.sleep(0.001) 
        
        recorded_data.extend(temp_data)
            
        # End Pulse
        left_puller.set_trigger_out_states(0)
        
        # Check final positions
        l_end = left_puller.get_position()
        r_end = right_puller.get_position()
        print(f"End Pos - Left: {l_end:.4f}, Right: {r_end:.4f}")
        
        # Validate roughly
        if abs(l_end - left_target) > 0.1 or abs(r_end - right_target) > 0.1:
            print("WARNING: Targets not reached! Sync might be broken.")
            
        # Flip direction
        current_direction *= -1
        time.sleep(0.5)

    # 7. Reset and Plot
    print("\nTest finished. Resetting triggers...")
    left_puller.set_trigger_config(0, 0)
    right_puller.set_trigger_config(0, 0)
    left_puller.disconnect()
    right_puller.disconnect()
    
    print("Generating plot...")
    data = np.array(recorded_data)
    if len(data) > 0:
        times = data[:, 0]
        l_pos = data[:, 1]
        r_pos = data[:, 2]
        
        plt.figure(figsize=(10, 6))
        plt.plot(times, l_pos, label='Left Motor', marker='.', markersize=2)
        plt.plot(times, r_pos, label='Right Motor', marker='.', markersize=2)
        
        plt.title("Brushing Synchronization Test")
        plt.xlabel("Time (s)")
        plt.ylabel("Position (mm)")
        plt.legend()
        plt.grid(True)
        
        plot_file = "brushing_sync_test.png"
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
