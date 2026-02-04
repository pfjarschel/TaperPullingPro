
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
        # Always return to center first
        right_puller.go_to(center_pos)
        while right_puller.motor_moving(): time.sleep(0.05)
            
        print(f"\rTesting Bits: 0x{i:02X} ({i})", end="")
        
        # Set Trigger Bits using c_ubyte
        # We need to modify set_trigger_switches_raw to use c_ubyte or cast here?
        # Let's import explicit types to be safe
        from ctypes import c_ubyte
        # We can't easily modify the TPM class method argument type on the fly
        # unless we modify the TPM file or subclass.
        # But wait, TaperPullingMotors.py uses c_byte. c_byte is signed (-128 to 127).
        # 0-255 will overflow c_byte if > 127.
        # We MUST fix TaperPullingMotors.py to use c_ubyte or handle the conversion.
        
        # For this script, let's just hack the call directly if possible, or
        # better yet, let's fix the main class first.
        
        # Temporary workaround for this script:
        # Cast to signed byte for the library call if it expects 'char' (which is signed mostly)
        # But SetTriggerSwitches expects 'byte' which is usually unsigned char in Thorlabs.
        # Let's Try passing it as an int and let ctypes handle it, or force unsigned.
        
        # Actually, let's just update the main file in the next step.
        # For now, let's assume the user will apply the fix I'm about to make to TPM.
        right_puller.set_trigger_switches_raw(i)
        
        # Command Move
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
