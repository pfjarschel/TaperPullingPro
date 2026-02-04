
import TaperPulling.TaperPullingMotors as TPM
import time
import sys

def get_di_state(motor):
    status = motor.get_status_bits()
    # Bit 20 (0-indexed) is Digital Input 1
    return (status >> 20) & 1

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
    
    # 1. Initialize Left (MASTER) as GPO
    print("\n--- GPO STATE DIAGNOSTICS ---")
    
    # Set Low
    print("Setting Master Out to LOW...")
    left_puller.set_trigger_out_states(0) 
    time.sleep(0.5)
    di_state = get_di_state(right_puller)
    print(f"Master=LOW -> Slave DI1 Read: {di_state} (Should be 0)")
    
    # Set High
    print("Setting Master Out to HIGH...")
    left_puller.set_trigger_out_states(1) 
    time.sleep(0.5)
    di_state = get_di_state(right_puller)
    print(f"Master=HIGH -> Slave DI1 Read: {di_state} (Should be 1)")
    
    # Reset to Low
    print("Resetting Master Out to LOW...")
    left_puller.set_trigger_out_states(0)
    time.sleep(0.5)
    
    # 2. Test Modes: 2 (Rel) and 3 (Abs)
    # Mode 2 is usually Relative Move. Mode 3 is Absolute.
    
    center_pos = 50.0
    test_delta = 3.0
    
    # Ensure centered
    right_puller.go_to(center_pos)
    while right_puller.motor_moving(): time.sleep(0.1)

    # Test Cases: (Mode, UseRelativeCommand)
    test_cases = [
        (2, True, "Mode 2 (Relative) + MoveRelative"),
        (3, False, "Mode 3 (Absolute) + MoveAbsolute")
    ]
    
    for mode, use_rel, desc in test_cases:
        print(f"\n--- TESTING {desc} ---")
        
        # Reset position
        right_puller.go_to(center_pos)
        while right_puller.motor_moving(): time.sleep(0.1)
        
        # Configure Slave: Mode=mode, Polarity=0 (High/Rising)
        right_puller.set_trigger_config(mode, 0) 
        print(f"Right (Slave) Switches: 0x{right_puller.get_trigger_switches():02X}")
        
        if use_rel:
            print(f"Arming Slave (Relative {test_delta}mm)...")
            right_puller.move_relative(test_delta)
        else:
            target = center_pos + test_delta
            print(f"Arming Slave (Absolute {target:.3f}mm)...")
            right_puller.move_absolute(target)
        
        # Check immediate movement
        time.sleep(1.0)
        pos = right_puller.get_position()
        status = right_puller.get_status_bits() & 0xFFFFFFFF # Ensure unsigned
        
        if abs(pos - center_pos) > 0.1: # Moved from center?
            print(f"FAILED: Moved immediately. (Pos: {pos:.3f})")
        else:
            print(f"ARMED (Pos: {pos:.3f}, Status: 0x{status:08X})")
            print("Firing TRIGGER PULSE (0.2s)...")
            
            # Pulse
            left_puller.set_trigger_out_states(1)
            time.sleep(0.2) # Longer pulse
            left_puller.set_trigger_out_states(0)
            
            # Check move
            time.sleep(1.5) # Wait for move
            pos = right_puller.get_position()
            status = right_puller.get_status_bits() & 0xFFFFFFFF
            
            dist = pos - center_pos
            print(f"Post-Trigger: Pos={pos:.3f} (Delta={dist:.3f}), Status: 0x{status:08X}")
            
            if abs(dist) > 0.1:
                print(f"*** SUCCESS! Triggered Move Completed. ***")
                break 
            else:
                print(f"FAILED: Did not move.")

    print("\nTest Finished.")
    left_puller.disconnect()
    right_puller.disconnect()

if __name__ == "__main__":
    try:
        run_test()
    except Exception as e:
        print(f"An error occurred: {e}")
        import traceback
        traceback.print_exc()
