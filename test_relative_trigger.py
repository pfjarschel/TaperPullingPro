
import TaperPulling.TaperPullingMotors as TPM
import time
import sys

def run_test():
    print("Initializing motors...")
    # LEFT is MASTER (Out), RIGHT is SLAVE (In)
    left_puller = TPM.LeftPuller()
    right_puller = TPM.RightPuller()
    
    left_puller.connect()
    right_puller.connect()
    
    if not left_puller.ok or not right_puller.ok:
        print("Failed to connect to motors.")
        sys.exit(1)
        
    print("Motors connected.")
    
    # 1. Initialize Left (MASTER) as GPO, State Low (Polarity 1)
    print("Initializing Master (Left) Trigger Out to LOW (Inactive)...")
    left_puller.set_trigger_out_states(0) 
    left_puller.set_trigger_config(0, 0) # Input Mode None on Master
    
    print(f"Left (Master) Switches: 0x{left_puller.get_trigger_switches():02X}")
    
    # 2. Start testing modes for Right (SLAVE)
    modes_to_test = [2, 3, 4]
    
    center_pos = 50.0
    test_delta = 3.0
    
    print(f"Moving BOTH to start position: {center_pos} mm")
    left_puller.go_to(center_pos)
    right_puller.go_to(center_pos)
    while left_puller.motor_moving() or right_puller.motor_moving(): time.sleep(0.1)
    
    for mode in modes_to_test:
        print(f"\n--- TESTING SLAVE (RIGHT) TRIGGER MODE: {mode} ---")
        
        # Reset position for clean test
        print(f"Centering Slave (Right) at {center_pos}...")
        right_puller.go_to(center_pos)
        while right_puller.motor_moving(): time.sleep(0.1)
        
        # Configure Slave (Right)
        # Pol 0 = Active High / Rising Edge
        right_puller.set_trigger_config(mode, 0) 
        print(f"Right (Slave) Switches Configured: 0x{right_puller.get_trigger_switches():02X}")
        
        target = center_pos + test_delta
        print(f"Arming Slave (Right) to position {target:.3f}...")
        right_puller.move_absolute(target)
        
        # Check if it moved immediately
        print("Checking if Slave (Right) stayed put...")
        time.sleep(1.0)
        pos = right_puller.get_position()
        status = right_puller.get_status_bits()
        print(f"Post-Arm: Pos={pos:.3f}, StatusBits=0x{status:08X}")
        
        if abs(pos - target) < 0.1:
            print(f"Mode {mode} FAILED: Moved immediately upon arming.")
        else:
            print(f"Mode {mode} successfully ARMED (did not move).")
            print("Firing trigger pulse from Master (Left)...")
            left_puller.set_trigger_out_states(1) # State 1 = High
            time.sleep(0.1)
            left_puller.set_trigger_out_states(0) # State 0 = Low
            
            # Check if it moved
            print("Checking if Slave (Right) moved now...")
            time.sleep(1.0)
            pos = right_puller.get_position()
            print(f"Post-Trigger: Pos={pos:.3f}")
            if abs(pos - target) < 0.1:
                print(f"*** SUCCESS: Mode {mode} worked with hardware trigger! ***")
            else:
                print(f"Mode {mode} FAILED: Did not move after trigger pulse.")
        
        # Invert delta for next iteration
        test_delta *= -1

    print("\nSearch Finished.")
    left_puller.disconnect()
    right_puller.disconnect()

if __name__ == "__main__":
    try:
        run_test()
    except Exception as e:
        print(f"An error occurred: {e}")
        import traceback
        traceback.print_exc()
