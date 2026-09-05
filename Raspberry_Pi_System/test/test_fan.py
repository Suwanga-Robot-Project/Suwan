import time
from gpiozero import OutputDevice

# --- Hardware Setup ---
FAN_PIN = 26

# active_high=True: 1 is ON, 0 is OFF
# initial_value=False: Default state is Pull-down (0)
fan_relay = OutputDevice(FAN_PIN, active_high=True, initial_value=False)

print("===========================================")
print(" Cooling Fan (12V Relay) Test Script")
print("===========================================")
print(" Commands:")
print("   '1' : Turn Fan ON (Boost)")
print("   '0' : Turn Fan OFF")
print("   'q' : Quit Script")
print("===========================================")

try:
    while True:
        cmd = input("Enter command (1 / 0 / q): ").strip()
        
        if cmd == '1':
            fan_relay.on()
            print("[STATUS] Relay activated (Fan ON).")
        elif cmd == '0':
            fan_relay.off()
            print("[STATUS] Relay deactivated (Fan OFF).")
        elif cmd.lower() == 'q':
            print("Exiting test script...")
            break
        else:
            print("[ERROR] Invalid input. Please enter 1, 0, or q.")
            
except KeyboardInterrupt:
    print("\n[WARNING] Test interrupted by user (Ctrl+C).")

finally:
    # Safely turn off and release the GPIO pin before exiting
    fan_relay.off()
    fan_relay.close()
    print("Cleanup complete. Fan is turned OFF.")
