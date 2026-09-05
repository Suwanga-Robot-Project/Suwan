import time
from gpiozero import OutputDevice, Button

# --- Hardware Setup ---
LIFT_PIN_1 = 17
LIFT_PIN_2 = 27
LIMIT_TOP_PIN = 23
LIMIT_BOTTOM_PIN = 24

lift_dir_1 = OutputDevice(LIFT_PIN_1, initial_value=False)
lift_dir_2 = OutputDevice(LIFT_PIN_2, initial_value=False)

limit_top = Button(LIMIT_TOP_PIN, pull_up=True)
limit_bottom = Button(LIMIT_BOTTOM_PIN, pull_up=True)

def stop_lift():
    lift_dir_1.off()
    lift_dir_2.off()
    print("[STATUS] Lift stopped.")

print("===========================================")
print(" Lift Mechanism Test Script")
print("===========================================")
print(" Pins: DIR1=GPIO17, DIR2=GPIO27")
print(" Limits: TOP=GPIO23, BOTTOM=GPIO24")
print("-------------------------------------------")
print(" Commands:")
print("   'u' : Ascend (Up)")
print("   'd' : Descend (Down)")
print("   's' : Stop")
print("   's_top' : Read Top Limit Switch")
print("   's_bot' : Read Bottom Limit Switch")
print("   'q' : Quit Script")
print("===========================================")

try:
    while True:
        cmd = input("\nEnter command (u / d / s / s_top / s_bot / q): ").strip().lower()

        if cmd == 'u':
            if limit_top.is_pressed:
                print("[WARNING] Top limit switch is already pressed! Cannot ascend.")
            else:
                stop_lift()
                time.sleep(0.05)
                lift_dir_1.on()
                print("[STATUS] Ascending (Up)...")

        elif cmd == 'd':
            if limit_bottom.is_pressed:
                print("[WARNING] Bottom limit switch is already pressed! Cannot descend.")
            else:
                stop_lift()
                time.sleep(0.05)
                lift_dir_2.on()
                print("[STATUS] Descending (Down)...")

        elif cmd == 's':
            stop_lift()

        elif cmd == 's_top':
            state = "PRESSED (Triggered)" if limit_top.is_pressed else "RELEASED (Open)"
            print(f"[LIMIT] Top Switch: {state}")

        elif cmd == 's_bot':
            state = "PRESSED (Triggered)" if limit_bottom.is_pressed else "RELEASED (Open)"
            print(f"[LIMIT] Bottom Switch: {state}")

        elif cmd == 'q':
            print("Exiting test script...")
            break

        else:
            print("[ERROR] Invalid command.")

except KeyboardInterrupt:
    print("\n[WARNING] Interrupted by user.")

finally:
    stop_lift()
    lift_dir_1.close()
    lift_dir_2.close()
    limit_top.close()
    limit_bottom.close()
    print("Cleanup complete. Pins released.")
