
import TaperPulling.TaperPullingMotors as TPM
import time
import sys

def run_scan():
    print("Initializing motors...")
    # Right Puller will be the DUT (Device Under Test)
    right_puller = TPM.RightPuller()
    # Left Puller used to keep line Low
    left_puller = TPM.LeftPuller()
    
    left_puller.connect()
    right_puller.connect()
    
    if not right_puller.ok:
        print("Failed to connect to Right motors.")
        sys.exit(1)
        
    print("Motors connected.")
    
    # Ensure Line is LOW
    if left_puller.ok:
        left_puller.set_trigger_out_states(0)
    
    center_pos = 50.0
    scan_target = 50.5 # Small move
    
    print(f"Moving to start position: {center_pos} mm")
    right_puller.go_to(center_pos)
    while right_puller.motor_moving(): time.sleep(0.1)
    
    candidates = []
    
    print("Starting Brute Force Scan of Trigger Bits (0-255)...")
    print("Looking for settings where the motor DOES NOT move immediately.")
    
    for i in range(256):
        # Reset
        if abs(right_puller.get_position() - center_pos) > 0.05:
            right_puller.go_to(center_pos)
            while right_puller.motor_moving(): time.sleep(0.05)
            
        print(f"\rTesting Bits: 0x{i:02X} ({i})", end="")
        
        # Set Trigger Bits
        # Note: We must use the raw set method or helper
        # Since TaperPullingMotors uses bit logic, let's use the raw call via eval hack 
        # or add a raw method. We'll verify what we have available.
        # TPM has `set_trigger_switches_raw`!
        right_puller.set_trigger_switches_raw(i)
        
        # Command Move
        # Try Relative Move this time
        right_puller.move_relative(0.5)
        
        # Wait small amount to see if it moves
        time.sleep(0.2)
        
        pos = right_puller.get_position()
        
        if abs(pos - center_pos) < 0.01:
            # It stayed put! (or failed to move for other reasons)
            # Verify it's not just broken by sending a Stop
            right_puller.stop(1)
            print(f" -> CANDIDATE FOUND: 0x{i:02X}")
            candidates.append(i)
        else:
            # It moved. Stop it.
            right_puller.stop(1)
            
    print("\nScan Finished.")
    print("Candidates (Values that waited):", [f"0x{c:02X}" for c in candidates])
    
    right_puller.disconnect()
    if left_puller.ok: left_puller.disconnect()

if __name__ == "__main__":
    try:
        run_scan()
    except Exception as e:
        print(f"An error occurred: {e}")
        import traceback
        traceback.print_exc()
