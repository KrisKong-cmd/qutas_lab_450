#!/usr/bin/env python3
import pigpio
import time

# Initialize pigpio
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("Could not connect to pigpio daemon")

# Servo GPIO pin (change as needed, BCM numbering)
SERVO_PIN = 12

# Servo pulse‐width settings for a 180° servo (in microseconds)
MIN_PULSE = 500    # -90°
MAX_PULSE = 2500   # +90°
NEUTRAL_PULSE = 1500  # 0° (90° in the original scale)

def set_servo_angle(angle):
    """
    Move the servo to the requested angle (-90° to +90°).
    Clamps out‐of‐range values, computes the pulse width,
    and sends it to the servo.
    """
    # Ensure angle is within valid range
    angle = max(-90, min(90, angle))
    # Calculate pulse width
    pulse_width = NEUTRAL_PULSE + (angle / 90.0) * (MAX_PULSE - NEUTRAL_PULSE)
    # Send command to servo
    pi.set_servo_pulsewidth(SERVO_PIN, pulse_width)

def main():
    try:
        print("Servo control (-90° to +90°). Press Ctrl+C to exit.")
        while True:
            # Get user input
            try:
                raw = input("Enter angle (-90 to +90): ")
                angle = float(raw)
                if -90 <= angle <= 90:
                    set_servo_angle(angle)
                    print(f"Servo moved to {angle}°")
                else:
                    print("Angle must be between -90 and +90 degrees")
            except ValueError:
                print("Please enter a valid number")
            # Small delay for stability
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopping servo control")
        # Stop sending pulses
        pi.set_servo_pulsewidth(SERVO_PIN, 0)
        pi.stop()

if __name__ == "__main__":
    main()