from gpiozero import AngularServo
from time import sleep
import signal
import sys

def signal_handler(sig, frame):
    print("\nSafely shutting down...")
    servo.detach()
    sys.exit(0)

# Register signal handler for Ctrl+C
signal.signal(signal.SIGINT, signal_handler)

try:
    # Map 30°–150° to -60°–+60°
    servo = AngularServo(12, min_angle=-60, max_angle=60, 
                        min_pulse_width=0.0006, max_pulse_width=0.0023)

    while True:
        print("Move to 150°")
        servo.angle = 60   # corresponds to physical 150°
        sleep(2)

        print("Move to 30°")
        servo.angle = -60  # corresponds to physical 30°
        sleep(2)

except Exception as e:
    print(f"Error: {e}")
finally:
    if 'servo' in locals():
        servo.detach()