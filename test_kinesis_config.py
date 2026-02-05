
import TaperPulling.TaperPullingMotors as TPM
import time
import sys

def run_test():
    print("Initializing motors (CONNECT ONLY)...")
    # LEFT is MASTER (Out) - We control this one to fire the trigger
    # RIGHT is SLAVE (In) - We expect this one to be configured by Kinesis
    left_puller = TPM.LeftPuller()
    right_puller = TPM.RightPuller()
    
    # Connect loads settings from the device (which Kinesis should have saved)
    left_puller.connect()
    right_puller.connect()
    
    if not left_puller.ok or not right_puller.ok:
        print("Failed to connect to motors.")
        sys.exit(1)
        
    print("Motors connected.")
    
    # 1. READ CONFIGURATION
    print("\n--- Reading Current Configuration (Right Puller) ---")
    
    # Read Trigger Switches
    switches = right_puller.get_trigger_switches()
    print(f"Trigger Switches: 0x{switches:02X}")
    # Decode for user convenience
    # bits 0-2: Input Mode
    input_mode = switches & 0x07
    print(f"  -> Input Mode: {input_mode} (0=None, 1=GPI, 2=Rel, 3=Abs)")
    
    # Read Absolute Position Target
    abs_target = right_puller.get_move_absolute_position()
    print(f"Stored Absolute Target: {abs_target:.3f}")
    
    # Read Relative Distance
    rel_dist = right_puller.get_move_relative_distance()
    print(f"Stored Relative Distance: {rel_dist:.3f}")
    
    current_pos = right_puller.get_position()
    print(f"Current Position: {current_pos:.3f}")
    
    print("--------------------------------------------------\n")

    # 2. Fire Trigger
    # We assume the user has positioned the motor where they want it (Start Pos)
    # or that the 'Absolute Move' will go to the target regardless of where it is.
    
    print("Firing Trigger Pulse (Master Left Puller High -> Low)...")
    # Ensure Master Trigger is set to output mode (if not already)
    # We'll just toggle the bit which works if it's in GPO mode? 
    # Actually, we should probably safely ensure 'left_puller' is in GPO mode.
    # But usually we rely on 'set_trigger_out_states' doing the right thing for manual firing.
    # Although previous script worked with just set_trigger_out_states.
    
    left_puller.set_trigger_out_states(1)
    time.sleep(0.2)
    left_puller.set_trigger_out_states(0)
    
    # 3. Check for move
    print("Waiting for move (2.0s)...")
    time.sleep(2.0)
    
    final_pos = right_puller.get_position()
    delta = final_pos - current_pos
    
    print(f"\nFinal Position: {final_pos:.3f} (Delta: {delta:.3f})")
    
    # Diagnostics
    if abs_target > 0 and abs(final_pos - abs_target) < 0.1:
        print("SUCCESS: Moved to Absolute Target!")
    elif rel_dist > 0 and abs(delta - rel_dist) < 0.01:
        print(f"RESULT: Moved Relative Distance ({rel_dist:.3f}). Mode {input_mode} behaving as Relative.")
    elif abs(delta) > 0.001:
        print(f"RESULT: Moved {delta:.3f}mm (Unknown mode).")
    else:
        print("RESULT: Did not move.")

    # Cleanup
    print("\nDisconnecting...")
    left_puller.disconnect()
    right_puller.disconnect()

if __name__ == "__main__":
    try:
        run_test()
    except Exception as e:
        print(f"An error occurred: {e}")
        import traceback
        traceback.print_exc()
