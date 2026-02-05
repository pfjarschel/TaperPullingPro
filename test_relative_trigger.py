
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
    
    print("Initializing settings...")
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
    
    center_pos = 50.0
    target_pos = 53.0
    
    # 1. Reset everything
    print("Resetting states...")
    left_puller.set_trigger_out_states(0) # Ensure Master is Low
    
    # Clear Slave Config first
    print(f"Clearing Trigger Config: {right_puller.set_trigger_config(0, 0)}")
    
    # Move to Start
    print(f"Moving to Start: {center_pos}")
    right_puller.go_to(center_pos)
    while right_puller.motor_moving(): time.sleep(0.1)
    
    # 2. Set Target Position (BUT DO NOT CALL MOVE)
    print(f"Pre-Setting Target Position to {target_pos}...")
    err = right_puller.set_move_absolute_position(target_pos)
    print(f"SetAbsPos Return Code: {err}")
    
    # Verify Absolute Position
    stored_target = right_puller.get_move_absolute_position()
    print(f"Stored Target Position: {stored_target:.3f}")
    
    # Set defined Relative Distance to rule out/in relative mode
    test_rel_dist = 0.5
    print(f"Setting Relative Distance to {test_rel_dist}mm to test mode hypothesis...")
    err = right_puller.set_move_relative_distance(test_rel_dist)
    print(f"SetRelDist Return Code: {err}")
    stored_rel = right_puller.get_move_relative_distance()
    print(f"Stored Relative Distance: {stored_rel:.3f}")
    
    # 3. Enable Trigger Mode
    # We now expect Mode 3 (Standard Abs) to be automatically mapped to Mode 5 (BMC Abs) by the class.
    print("Enabling Trigger Mode (Mode 3=Abs, Polarity 0)...")
    err = right_puller.set_trigger_config(3, 0)
    print(f"SetTriggerConfig Return Code: {err}")
    
    print(f"Slave Switches: 0x{right_puller.get_trigger_switches():02X}")
    
    # 4. Check if it waits (Armed State)
    print("Checking for immediate movement (Waiting 2s)...")
    time.sleep(2.0)
    
    pos = right_puller.get_position()
    if abs(pos - center_pos) > 0.1:
        print(f"FAILED: Moved immediately upon setting config! Pos: {pos:.3f}")
        # Disable trigger to stop weirdness
        right_puller.set_trigger_config(0, 0)
    else:
        print(f"ARMED SUCCESSFULLY! (Pos: {pos:.3f})")
        
        # 5. Fire Trigger
        print("Firing Trigger Pulse (Master High -> Low)...")
        left_puller.set_trigger_out_states(1)
        time.sleep(0.2)
        left_puller.set_trigger_out_states(0)
        
        # 6. Check for move
        print("Waiting for move...")
        time.sleep(2.0)
        pos = right_puller.get_position()
        
        dist = pos - center_pos
        print(f"Final Position: {pos:.3f} (Delta: {dist:.3f})")
        
        if abs(pos - target_pos) < 0.1:
            print("SUCCESS! Motor moved to target after trigger (Absolute Mode Working!).")
        elif abs(pos - (center_pos + test_rel_dist)) < 0.1:
             print("RESULT: Still moved Relative 0.5mm (Absolute Mode failed).")
        else:
             print(f"RESULT: Movement didn't match Rel ({test_rel_dist}) or Abs ({target_pos}).")

    # Cleanup
    print("Disabling Triggers and Disconnecting...")
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
