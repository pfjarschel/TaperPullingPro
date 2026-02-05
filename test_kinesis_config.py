
import TaperPulling.TaperPullingMotors as TPM
import time
import sys

def run_test():
    print("Initialize Left Puller (Trigger Source) ONLY...")
    # LEFT is MASTER (Out) - We control this one to fire the trigger
    left_puller = TPM.LeftPuller()
    
    left_puller.connect()
    
    if not left_puller.ok:
        print("Failed to connect to Left Puller.")
        sys.exit(1)
        
    print("Left Puller connected.")
    print("Right Puller should be open in Kinesis software.")
    print("Ready to fire trigger...")
    time.sleep(1.0)

    # 2. Fire Trigger
    print("Firing Trigger Pulse (Master Left Puller High -> Low)...")
    left_puller.set_trigger_out_states(1)
    time.sleep(0.2)
    left_puller.set_trigger_out_states(0)
    
    print("Trigger Fired!")
    print("Check Kinesis software to see if Right Puller moved to Target (Absolute) or +Distance (Relative).")

    # Cleanup
    print("\nDisconnecting...")
    left_puller.disconnect()

if __name__ == "__main__":
    try:
        run_test()
    except Exception as e:
        print(f"An error occurred: {e}")
        import traceback
        traceback.print_exc()
