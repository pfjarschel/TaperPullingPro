
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
    
    print("Starting Brute Force Scan with Safety Reset...")
    
    # Force Disable Triggers Initially
    right_puller.set_trigger_switches_raw(0)
    
    for i in range(256):
        # 0. Safety Reset of Trigger Config (Crucial!)
        right_puller.set_trigger_switches_raw(0x00)
        time.sleep(0.05)
        
        # 1. Return to Center
        # Check current pos first
        cur_pos = right_puller.get_position()
        if abs(cur_pos - center_pos) > 0.05:
            # print(f"\rResetting... (Pos: {cur_pos:.3f})", end="")
            right_puller.go_to(center_pos)
            
            # Timeout wait
            t0 = time.time()
            while right_puller.motor_moving():
                if time.time() - t0 > 5.0:
                    print(f"\nFATAL: Timeout resetting to center. Status: 0x{right_puller.get_status_bits():08X}")
                    sys.exit(1)
                time.sleep(0.05)
                
        print(f"\rTesting Bits: 0x{i:02X} ({i}) ", end="")
        
        # 2. Set Trigger Bits
        right_puller.set_trigger_switches_raw(i)
        
        # 3. Command Move (Relative)
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
