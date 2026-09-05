import time
from gpiozero import Button

# --- Pin Setup ---
LIMIT_TOP_PIN = 23
LIMIT_BOTTOM_PIN = 24

# Initialize limit switches with internal pull-up enabled
limit_top = Button(LIMIT_TOP_PIN, pull_up=True)
limit_bottom = Button(LIMIT_BOTTOM_PIN, pull_up=True)

# Event Callbacks
def top_pressed():
    print("[EVENT] TOP Limit Switch -> PRESSED (Triggered)")

def top_released():
    print("[EVENT] TOP Limit Switch -> RELEASED")

def bottom_pressed():
    print("[EVENT] BOTTOM Limit Switch -> PRESSED (Triggered)")

def bottom_released():
    print("[EVENT] BOTTOM Limit Switch -> RELEASED")

# Attach Callback Functions
limit_top.when_pressed = top_pressed
limit_top.when_released = top_released
limit_bottom.when_pressed = bottom_pressed
limit_bottom.when_released = bottom_released

print("===========================================")
print(" Limit Switch Signal Test Script")
print("===========================================")
print(f" TOP Limit Pin   : GPIO {LIMIT_TOP_PIN}")
print(f" BOTTOM Limit Pin: GPIO {LIMIT_BOTTOM_PIN}")
print("-------------------------------------------")
print(" Press the limit switches manually with your hand.")
print(" Real-time events will be displayed below.")
print(" Press Ctrl+C to stop.")
print("===========================================\n")

try:
    # Print initial status
    print("Initial Status:")
    print(f" - Top Switch    : {'PRESSED' if limit_top.is_pressed else 'RELEASED'}")
    print(f" - Bottom Switch : {'PRESSED' if limit_bottom.is_pressed else 'RELEASED'}")
    print("\nWaiting for switch input events...\n")
    
    # Keep running to listen for events
    while True:
        time.sleep(1)

except KeyboardInterrupt:
    print("\n[INFO] Test stopped by user.")

finally:
    limit_top.close()
    limit_bottom.close()
    print("[INFO] Cleanup complete. GPIO pins released.")
