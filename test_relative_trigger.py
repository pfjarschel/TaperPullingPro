
import TaperPulling.TaperPullingMotors as TPM
import time
import sys

def run_test():
    print("Initializing motors...")
    # Initialize motors
    # Note: These classes attempt to connect upon initialization if not simulated
    left_puller = TPM.LeftPuller()
    right_puller = TPM.RightPuller()
    
    left_puller.connect()
    right_puller.connect()
    
    if not left_puller.ok or not right_puller.ok:
        print("Failed to connect to motors.")
        sys.exit(1)
        
    print("Motors connected.")
    print(f"Left: {left_puller.serial}, Right: {right_puller.serial}")

    # Configuration Constants (Matches Chat Export)
    # KMOT_TriggerPortMode: 0x02 = Absolute Move on Trigger
    # KMOT_TriggerPortPolarity: 0x01 = Active High / Rising Edge
    TRIG_MODE_ABS = 0x02
    TRIG_POL_HIGH = 0x01
    
    print("Configuring Triggers...")
    left_puller.set_trigger_config(TRIG_MODE_ABS, TRIG_POL_HIGH)
    right_puller.set_trigger_config(TRIG_MODE_ABS, TRIG_POL_HIGH)
    
    # Configure Output (Optional, but good practice if using Master's out)
    # For manual firing via SetTriggerOutStates, config might not matter as much, 
    # but let's ensure it's not set to something conflicting.
    # 0x01 = Move Complete (we overide this with manual state control anyway)
    # right_puller.set_trigger_out_config(0x01, 0x01) 

    # Test Parameters
    center_pos = 10.0 # mm (Safe spot)
    brush_amp = 1.0 # mm
    cycles = 6
    
    # Go to start position normally first
    print(f"Moving to start position: {center_pos} mm")
    left_puller.go_to(center_pos)
    right_puller.go_to(center_pos)
    
    # Wait for completion
    while left_puller.motor_moving() or right_puller.motor_moving():
        time.sleep(0.1)
        
    print("Start position reached. Starting Triggered Loop...")
    
    current_pos_l = center_pos
    current_pos_r = center_pos
    
    for i in range(cycles):
        # Determine direction
        direction = 1 if i % 2 == 0 else -1
        delta = brush_amp * direction
        
        target_l = current_pos_l + delta
        target_r = current_pos_r + delta # Both move same direction for simpler 'brushing' test
        # Note: real pulling involves them moving apart, but for sync test, parallel motion is fine.
        
        print(f"Cycle {i+1}: Preparing move to L={target_l:.3f}, R={target_r:.3f}")
        
        # 1. Pre-load positions
        left_puller.set_move_absolute_position(target_l)
        right_puller.set_move_absolute_position(target_r)
        
        # 2. Arm Triggers (simultaneous via hardware, armed via software)
        # Using move_absolute(wait=False) which acts as the 'Arm' command in Trigger Mode
        left_puller.move_absolute(target_l, wait=False)
        right_puller.move_absolute(target_r, wait=False)
        
        # Short wait to ensure both are armed
        time.sleep(0.1)
        
        # 3. Fire Trigger (Master - Right Puller)
        print("Fring trigger...")
        # Pulse Pin 10 (Digital Out 1) High
        # 0x01 = Pin 1 High (assuming Pin 1 maps to the DO used for Trigger Out)
        # Check TBD001 manual or trial: usually DO1 is used for Trig Out.
        right_puller.set_trigger_out_states(0x01)
        time.sleep(0.010) # 10ms pulse
        right_puller.set_trigger_out_states(0x00)
        
        # 4. Wait for move completion
        # We can poll the status bits or position
        print("Waiting for move completion...")
        timeout = 5.0
        start_wait = time.time()
        while True:
            l_moving = left_puller.motor_moving()
            r_moving = right_puller.motor_moving()
            
            if not l_moving and not r_moving:
                break
                
            if time.time() - start_wait > timeout:
                print("Timeout waiting for move!")
                break
            time.sleep(0.05)
            
        print(f"Move Complete. L={left_puller.get_position():.4f}, R={right_puller.get_position():.4f}")
        
        current_pos_l = left_puller.get_position()
        current_pos_r = right_puller.get_position()
        
        time.sleep(0.5)

    print("Test Finished.")
    left_puller.disconnect()
    right_puller.disconnect()

if __name__ == "__main__":
    try:
        run_test()
    except Exception as e:
        print(f"An error occurred: {e}")
        import traceback
        traceback.print_exc()
