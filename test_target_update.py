import TaperPulling.TaperPullingMotors as TPM
import time
import sys

def run_test():
    print("=== Target Update Verification Test ===")
    print("Goal: Check if 'set_move_absolute_position' updates the target of an ONGOING triggered move.")
    
    # Initialize Right Motor (Slave/Input) for simplicity
    motor = TPM.RightPuller()
    motor.connect()
    
    if not motor.ok:
        print("Failed to connect.")
        sys.exit(1)
        
    motor.initialize()
    motor.set_velocity(5.0)
    motor.set_acceleration(10.0)
    
    # Move to Start
    start_pos = 50.0
    print(f"Moving to Start: {start_pos}")
    motor.go_to(start_pos)
    while motor.motor_moving():
        time.sleep(0.1)
    time.sleep(1.0)
    
    # Configure Trigger (Standard / Mode 3 -> 5)
    motor.set_trigger_config(3, 0)
    
    # Set Initial Target A
    target_A = 55.0
    target_B = 60.0
    
    print(f"Pre-loading Target A: {target_A}")
    motor.set_move_absolute_position(target_A)
    
    # Fire Trigger (Simulate by calling the move? No, we need hardware trigger usually)
    # But wait, Right Puller Input is connected to Left Puller Output.
    # We need Left Puller to fire the trigger.
    
    left = TPM.LeftPuller()
    left.connect()
    if not left.ok:
        print("Left motor needed for trigger source.")
        sys.exit(1)
    left.set_trigger_out_states(0)
    
    print("Firing Trigger (Start Move to A)...")
    left.set_trigger_out_states(1)
    
    # Wait a tiny bit for move to start
    time.sleep(0.2)
    if not motor.motor_moving():
        print("Error: Motor did not start moving!")
    else:
        print("Motor is moving...")
        
        # ACT: Update Target to B while moving AND RE-FIRE TRIGGER
        print(f"!!! UPDATING TARGET TO {target_B} AND RE-FIRING TRIGGER !!!")
        motor.set_move_absolute_position(target_B)
        time.sleep(0.05) # Small delay for comms
        
        # Reset Trigger first (pulse needs edge)
        left.set_trigger_out_states(0)
        time.sleep(0.05)
        
        # Fire Trigger AGAIN
        left.set_trigger_out_states(1)
        
        # Wait for completion
        while motor.motor_moving():
            pos = motor.get_position()
            # print(f"Pos: {pos:.2f}")
            time.sleep(0.1)
            
        final_pos = motor.get_position()
        print(f"Final Position: {final_pos:.4f}")
        
        if abs(final_pos - target_A) < 0.1:
            print("RESULT: Update IGNORED. Stopped at Target A.")
        elif abs(final_pos - target_B) < 0.1:
            print("RESULT: Update ACCEPTED. Stopped at Target B.")
        else:
            print("RESULT: Indeterminate.")

    # Cleanup
    left.set_trigger_out_states(0)
    motor.set_trigger_config(0, 0)
    motor.disconnect()
    left.disconnect()

if __name__ == "__main__":
    run_test()
