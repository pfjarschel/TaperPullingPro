
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
    
    # 0. RESET STATES (Critical!)
    print("\n0. Resetting states...")
    left_puller.set_trigger_out_states(0)
    print(f"   Clearing Trigger Config: {right_puller.set_trigger_config(0, 0)}")
    time.sleep(1.0)
    print(f"   Current Switches: 0x{right_puller.get_trigger_switches():02X}")

    # Using distinct positions:
    # BASE = 50.0
    # START = 55.0
    # TARGET = 65.0
    # INTERMEDIATE = 45.0
    
    start_pos = 55.0
    target_pos = 65.0
    intermediate_pos = 45.0
    
    # 1. Move to START POS (55.0)
    print(f"\n1. Moving to Start Position: {start_pos}...")
    status = right_puller.get_status_bits()
    print(f"   Status Bits before move: 0x{status:08X}")
    current = right_puller.get_position()
    print(f"   Current position before move: {current:.3f}")
    
    right_puller.go_to(start_pos)
    # Give the controller a moment to set the 'moving' bit
    time.sleep(0.5) 
    
    timeout = 15.0
    t0 = time.time()
    while right_puller.motor_moving(): 
        time.sleep(0.1)
        if time.time() - t0 > timeout:
            print("   TIMEOUT: Motor didn't stop moving!")
            break
            
    print(f"   Reached: {right_puller.get_position():.3f}")
    print(f"   Status Bits after reaching: 0x{right_puller.get_status_bits():08X}")

    # 2. Configure Trigger for Target 65.0
    print(f"\n2. Configuring Trigger for Target: {target_pos}...")
    # Add a small delay for safety
    time.sleep(0.5)
    right_puller.set_move_absolute_position(target_pos)
    right_puller.set_trigger_config(3, 0) # Maps to 5 internally
    print("   Trigger Configured.")
    print(f"   Switches: 0x{right_puller.get_trigger_switches():02X}")

    # 3. Move to INTERMEDIATE POS 45.0
    # THEORY: This call will BECOME the new armed target and deferred!
    print(f"\n3. Moving to Intermediate Position: {intermediate_pos} (deferred?)...")
    right_puller.go_to(intermediate_pos)
    time.sleep(1.0) # Check if it moved
    
    curr = right_puller.get_position()
    print(f"   Position after 1s: {curr:.3f}")
    if abs(curr - intermediate_pos) < 0.2:
        print("   -> Motor moved IMMEDIATELY. Trigger Mode does NOT capture standard MoveTo commands.")
    else:
        print("   -> Motor did NOT move. Trigger Mode IS capturing standard MoveTo commands!")

    # 4. Fire Trigger
    print("\n4. Firing Trigger...")
    left_puller.set_trigger_out_states(1)
    time.sleep(0.2)
    left_puller.set_trigger_out_states(0)
    
    print("   Waiting for move...")
    time.sleep(4.0) 
    
    final_pos = right_puller.get_position()
    print(f"\nFINAL POSITION: {final_pos:.3f}")
    
    print("\n--- CONCLUSION ---")
    if abs(final_pos - target_pos) < 0.2:
        print(f"PASSED: True Absolute Mode and standard calls are NOT deferred to trigger (Final={target_pos}).")
    elif abs(final_pos - intermediate_pos) < 0.2:
        print(f"OBSERVATION: Motor moved to the INTERMEDIATE target ({intermediate_pos}). It seems ANY absolute move command is deferred to trigger.")
    else:
        print(f"UNKNOWN: Landed at {final_pos}.")

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
