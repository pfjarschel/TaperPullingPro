
import TaperPulling.TaperPullingMotors as TPM
import time
import sys
import ctypes

def get_di_state(motor):
    status = motor.get_status_bits()
    # Bit 20 (0-indexed) is Digital Input 1
    # Note: Python integers have infinite precision, so no need to mask specifically for sign
    # But let's mask to 32-bit just in case:
    status &= 0xFFFFFFFF
    return (status >> 20) & 1

def run_strict_test():
    print("Initializing motors...")
    # LEFT is MASTER (Out)
    # RIGHT is SLAVE (In)
    left_puller = TPM.LeftPuller()
    right_puller = TPM.RightPuller()
    
    left_puller.connect()
    right_puller.connect()
    
    if not left_puller.ok or not right_puller.ok:
        print("Failed to connect to motors.")
        sys.exit(1)
        
    print("Motors connected.")

    center_pos = 50.0
    target_pos = 53.0
    
    # 0. Preparation
    print(f"Moving BOTH to start position: {center_pos} mm")
    left_puller.go_to(center_pos)
    right_puller.go_to(center_pos)
    while left_puller.motor_moving() or right_puller.motor_moving(): time.sleep(0.1)
    
    # 1. Ensure Line is LOW
    print("Setting Master Out to LOW...")
    left_puller.set_trigger_out_states(0) 
    left_puller.set_trigger_config(0, 0) # Clear Master Config
    
    print("Waiting for Slave DI1 to read LOW...")
    for _ in range(50):
        if get_di_state(right_puller) == 0:
            print("Line confirmed LOW.")
            break
        time.sleep(0.1)
    else:
        print("FATAL: Line did not settle to LOW.")
        sys.exit(1)
        
    # 2. Configure Slave: Mode 3 (Abs), Polarity 0 (Active High?)
    print("\n--- CONFIGURING SLAVE: Mode 3, Polarity 0 ---")
    right_puller.set_trigger_config(3, 0)
    switches = right_puller.get_trigger_switches()
    print(f"Right (Slave) Switches: 0x{switches:02X}")
    
    # 3. Arm
    print(f"Arming Slave to {target_pos:.3f}...")
    right_puller.move_absolute(target_pos)
    
    # 4. Check Wait State
    print("Checking if Waiting...")
    time.sleep(1.0)
    pos = right_puller.get_position()
    status = right_puller.get_status_bits() & 0xFFFFFFFF
    print(f"Post-Arm Status: Pos={pos:.3f}, Status=0x{status:08X}, DI1={get_di_state(right_puller)}")
    
    if abs(pos - center_pos) > 0.1:
        print("FAILED: Moved immediately.")
        sys.exit(0)
    
    print("Unit is WAITING (Success so far). Now Firing Trigger...")
    
    # 5. Fire and HOLD High
    print("Setting Master Out to HIGH (Holding)...")
    left_puller.set_trigger_out_states(1)
    
    # Monitor for movement while High
    start_time = time.time()
    moved = False
    while time.time() - start_time < 3.0:
        pos = right_puller.get_position()
        if abs(pos - center_pos) > 0.1:
            print(f"*** MOVEMENT DETECTED! Pos={pos:.3f} ***")
            moved = True
            break
        time.sleep(0.1)
        
    status = right_puller.get_status_bits() & 0xFFFFFFFF
    print(f"Status while HIGH: 0x{status:08X}, DI1={get_di_state(right_puller)}")
    
    print("Setting Master Out to LOW...")
    left_puller.set_trigger_out_states(0)
    
    if moved:
        print("TEST PASSED: Mode 3/Pol 0 triggers on HIGH level.")
    else:
        print("TEST FAILED: Timed out waiting for move while Trigger was HIGH.")

    left_puller.disconnect()
    right_puller.disconnect()

if __name__ == "__main__":
    try:
        run_strict_test()
    except Exception as e:
        print(f"An error occurred: {e}")
        import traceback
        traceback.print_exc()
