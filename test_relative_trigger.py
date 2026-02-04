
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
    
    # 2. Test Mode 3 (Abs) with Polarity Variations
    # Previously Mode 3 + Pol 0 moved immediately.
    # Let's test Pol 0 and Pol 1.
    
    center_pos = 50.0
    test_delta = 3.0
    
    # Ensure centered
    right_puller.go_to(center_pos)
    while right_puller.motor_moving(): time.sleep(0.1)

    polarities = [0, 1]
    
    for pol in polarities:
        mode = 3 # Absolute
        print(f"\n--- TESTING MODE 3 (ABS) with POLARITY {pol} ---")
        
        # Reset position
        right_puller.go_to(center_pos)
        while right_puller.motor_moving(): time.sleep(0.1)
        
        # Configure Slave
        # Pol 0 = Active High?, Pol 1 = Active Low?
        right_puller.set_trigger_config(mode, pol) 
        print(f"Right (Slave) Switches: 0x{right_puller.get_trigger_switches():02X}")
        
        target = center_pos + test_delta
        print(f"Arming Slave to {target:.3f}...")
        right_puller.move_absolute(target)
        
        # Check immediate movement
        time.sleep(1.0)
        pos = right_puller.get_position()
        
        if abs(pos - target) < 0.1:
            print(f"FAILED: Moved immediately. (Current Position: {pos:.3f})")
        else:
            print(f"ARMED SUCCESSFULLY! (Position stuck at {pos:.3f})")
            print("Firing TRIGGER PULSE (Master Low -> High -> Low)...")
            
            # Pulse
            left_puller.set_trigger_out_states(1)
            time.sleep(0.05)
            left_puller.set_trigger_out_states(0)
            
            # Check move
            time.sleep(1.0)
            pos = right_puller.get_position()
            
            if abs(pos - target) < 0.1:
                print(f"*** SUCCESS! Triggered Move Completed. ***")
                break # Found it!
            else:
                print(f"FAILED: Did not move after trigger. (Pos: {pos:.3f})")

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
