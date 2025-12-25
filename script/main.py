import sys
import time
import select

# Import core control module (assumes sf_can_controller.py is in the same directory)
from sf_can_controller import MotorController 

# --- Core Configuration ---
IFACE = "can0"        
MOTOR_ID = 1         # <- Change ID to control different motors
UPDATE_RATE_HZ = 100.0 
PRINT_EVERY = 2     
INITIAL_TARGET_DEG = 0.0

# --- Main Control Loop ---
def run_simple_test() -> None:
    """Run a simplified position control loop."""
    
    # 1. Initialization
    update_period = 1.0 / UPDATE_RATE_HZ
    target_rad = INITIAL_TARGET_DEG
    
    KP, KD = 0.5, 0.3  # Default MIT parameters
    
    controller = MotorController(interface=IFACE, motor_id=MOTOR_ID)
    print(f"--- SF Motor Test Start ---")
    print(f"Interface: {IFACE}, ID: {MOTOR_ID}, Rate: {UPDATE_RATE_HZ} Hz")
    
    # 2. Enable motor
    controller.enable()
    
    last_send_time = time.perf_counter()
    print_counter = 0

    inputCheckCount = 0

    # 3. Main loop
    while True:
        controller.poll_rx()
        current_state = controller.get_motor_state()
        
        now = time.perf_counter()
        
        # --- Periodic input check (every 500 loops) ---
        inputCheckCount += 1
        if inputCheckCount >= 500:
            inputCheckCount = 0
            
            # Blocking I/O waiting for user input (this will pause the control loop)
            # Note: If the input is not a number, a ValueError will be raised.
            line = input("Please enter target joint angle: ").strip()
            if line:
                angle_deg = float(line)
                target_rad = angle_deg
                print(f"Target joint angle updated: {angle_deg:.3f} deg")
        
        # Periodically send MIT command
        if now - last_send_time >= update_period:
            last_send_time = now
            
            # Send target position command
            controller.send_mit_command(
                pos=target_rad,
                vel=0.0,
                kp=KP,
                kd=KD,
                tor=0.0
            )

            # Print motor state
            print_counter += 1
            if print_counter >= PRINT_EVERY:
                print_counter = 0
                print(
                    f"Cmd={target_rad:.2f} | "
                    f"Pos={current_state.pos:.2f} (Vel={current_state.vel:.2f})"
                )
        
        time.sleep(0.001)
            

if __name__ == "__main__":
    # Run test
    run_simple_test()