
import TaperPulling.TaperPullingMotors as TPM
import time
import sys

def run_test():
    print("Initializing motors...")
    # LEFT is MASTER (Out)
    # RIGHT is SLAVE (In)
    left_puller = TPM.LeftPuller()
    right_puller = TPM.RightPuller()
    
    left_puller.connect()
    right_puller.connect()
    
    # Initialize basic settings
    left_puller.initialize()
    left_puller.set_velocity(left_puller.vel)
    left_puller.set_acceleration(left_puller.accel)
    
    right_puller.initialize()
    right_puller.set_velocity(right_puller.vel)
    right_puller.set_acceleration(right_puller.accel)
    
    if not left_puller.ok or not right_puller.ok:
        print("Failed to connect to motors.")
        sys.exit(1)
        
    print("Motors connected.")
    
    # Using positions closer to the ones we know work (50 area)
    start_pos = 50.0
    target_pos = 53.0
    intermediate_pos = 45.0
    
    # Hypothesis:
    # If "Fake Absolute":
    #   At 50, Config Target 53 -> Calculates Delta (+3.0).
    #   Move to 45.
    #   Trigger -> Moves +3.0 -> Lands at 48.
    # If "True Absolute":
    #   At 50, Config Target 53.
    #   Move to 45.
    #   Trigger -> Moves to 53 -> Lands at 53.

    # 1. Move to START POS (50.0)
    print(f"\n1. Moving to Start Position: {start_pos}...")
    current = right_puller.get_position()
    print(f"   Current position before move: {current:.3f}")
    right_puller.go_to(start_pos)
    
    timeout = 10.0
    t0 = time.time()
    while right_puller.motor_moving(): 
        time.sleep(0.1)
        if time.time() - t0 > timeout:
            print("   TIMEOUT: Motor still thinks it is moving but took too long!")
            break
            
    print(f"   Reached: {right_puller.get_position():.3f}")

    # 2. Configure Trigger (Target 73.0)
    print(f"\n2. Configuring Trigger for Target: {target_pos}...")
    right_puller.set_move_absolute_position(target_pos)
    # WE USE MODE 3 (Standard Abs) here, relying on our class fix to map it to 5 if needed.
    right_puller.set_trigger_config(3, 0)
    print("   Trigger Configured.")
    
    # Check if config persists
    switches = right_puller.get_trigger_switches()
    print(f"   Switches: 0x{switches:02X}")

    # 3. Move to INTERMEDIATE POS (45.0) - The "Trap"
    print(f"\n3. Moving to Intermediate Position: {intermediate_pos}...")
    right_puller.go_to(intermediate_pos)
    
    t0 = time.time()
    while right_puller.motor_moving(): 
        time.sleep(0.1)
        if time.time() - t0 > timeout:
            print("   TIMEOUT: Motor still thinks it is moving but took too long!")
            break
            
    print(f"   Reached: {right_puller.get_position():.3f}")
    
    # Check if config persisted after move
    switches_after = right_puller.get_trigger_switches()
    print(f"   Switches after move: 0x{switches_after:02X}")
    if (switches_after & 0x07) == 0:
        print("   WARNING: Move command cleared the Trigger Config! Re-enabling...")
        right_puller.set_trigger_config(3, 0)
    
    # 4. Fire Trigger
    print("\n4. Firing Trigger...")
    left_puller.set_trigger_out_states(1)
    time.sleep(0.2)
    left_puller.set_trigger_out_states(0)
    
    print("   Waiting for move...")
    time.sleep(3.0) # Give it time to travel 13mm (from 60 to 73)
    
    final_pos = right_puller.get_position()
    print(f"\nFINAL POSITION: {final_pos:.3f}")
    
    print("\n--- CONCLUSION ---")
    if abs(final_pos - target_pos) < 0.2:
        print(f"PASSED: True Absolute Mode! (Moved to {target_pos})")
    elif abs(final_pos - (intermediate_pos + (target_pos - start_pos))) < 0.2:
        # Expected: 60 + 3 = 63
        print(f"FAILED: Fake Absolute (Calculated Relative). Landed at {final_pos} (Start + Delta).")
    else:
        print(f"INCONCLUSIVE: Landed at {final_pos}. Neither Target nor simple Delta.")

    # Cleanup
    print("\nDisconnecting...")
    right_puller.set_trigger_config(0, 0)
    left_puller.disconnect()
    right_puller.disconnect()

if __name__ == "__main__":
    try:
        run_test()
    except Exception as e:
        print(f"An error occurred: {e}")
        import traceback
        traceback.print_exc()
