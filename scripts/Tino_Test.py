#!/usr/bin/env python3
import pigpio
import time

# Initialize pigpio
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("Could not connect to pigpio daemon")

# Servo GPIO pin (change as needed, BCM numbering)
SERVO_PIN = 12

# Servo pulse-width settings for Surpass Hobby S0002P (180° operating angle)
# Based on specs: 180°±10° (500~2500μs pulse width range)
MIN_PULSE = 500    # 0° position
MAX_PULSE = 2500   # 180° position
NEUTRAL_PULSE = 1500  # 90° position (middle)

# The servo can rotate 360° mechanically but operates within 180°
OPERATING_ANGLE_RANGE = 180  # degrees

# Global variable to track current servo position
current_angle = 90.0  # Start at center position
DEFAULT_SPEED = 30  # degrees per second (adjustable)

def get_current_pulse_width():
    """Get the current pulse width based on current angle."""
    return MIN_PULSE + (current_angle / 180.0) * (MAX_PULSE - MIN_PULSE)

def angle_to_pulse(angle):
    """Convert angle to pulse width."""
    return MIN_PULSE + (angle / 180.0) * (MAX_PULSE - MIN_PULSE)

def pulse_to_angle(pulse):
    """Convert pulse width to angle."""
    return (pulse - MIN_PULSE) / (MAX_PULSE - MIN_PULSE) * 180.0

def move_servo_smooth(target_angle, speed_deg_per_sec):
    """
    Move servo smoothly from current position to target angle.
    
    Args:
        target_angle: Target angle (0-180°)
        speed_deg_per_sec: Speed in degrees per second
    """
    global current_angle
    
    target_angle = max(0, min(180, target_angle))
    angle_diff = target_angle - current_angle
    
    if abs(angle_diff) < 0.5:  # Already close enough
        return angle_to_pulse(target_angle)
    
    # Calculate step size and delay
    step_time = 0.02  # 20ms steps for smooth movement
    angle_step = speed_deg_per_sec * step_time
    
    # Determine direction
    if angle_diff > 0:
        angle_step = abs(angle_step)
    else:
        angle_step = -abs(angle_step)
    
    # Move in small increments
    steps = int(abs(angle_diff) / abs(angle_step))
    
    for i in range(steps):
        current_angle += angle_step
        pulse_width = angle_to_pulse(current_angle)
        pi.set_servo_pulsewidth(SERVO_PIN, pulse_width)
        time.sleep(step_time)
    
    # Final position adjustment
    current_angle = target_angle
    final_pulse = angle_to_pulse(current_angle)
    pi.set_servo_pulsewidth(SERVO_PIN, final_pulse)
    
    return final_pulse

def set_servo_angle(angle, speed=None):
    """
    Move the servo to the requested angle (0° to 180°).
    0° = MIN_PULSE (500μs)
    90° = NEUTRAL_PULSE (1500μs) 
    180° = MAX_PULSE (2500μs)
    
    Args:
        angle: Target angle (0-180°)
        speed: Movement speed in degrees/second (None for instant movement)
    """
    # Ensure angle is within valid operating range
    target_angle = max(0, min(180, angle))
    target_pulse = MIN_PULSE + (target_angle / 180.0) * (MAX_PULSE - MIN_PULSE)
    
    if speed is None:
        # Instant movement (original behavior)
        pi.set_servo_pulsewidth(SERVO_PIN, target_pulse)
        return target_pulse
    else:
        # Smooth movement at specified speed
        return move_servo_smooth(target_angle, speed)

def set_servo_pulse(pulse_width):
    """
    Directly set servo pulse width (500-2500μs range).
    Useful for fine-tuning or testing specific pulse values.
    """
    # Ensure pulse width is within valid range
    pulse_width = max(MIN_PULSE, min(MAX_PULSE, pulse_width))
    
    # Send command to servo
    pi.set_servo_pulsewidth(SERVO_PIN, pulse_width)
    
    return pulse_width

def main():
    global current_angle
    
    try:
        # Initialize servo to center position
        print("Initializing servo to center position (90°)...")
        pi.set_servo_pulsewidth(SERVO_PIN, NEUTRAL_PULSE)
        current_angle = 90.0
        time.sleep(1)  # Give servo time to reach position
        
        print("Surpass Hobby S0002P Servo Control")
        print("Operating range: 0° to 180°")
        print("Pulse width range: 500-2500μs")
        print(f"Default speed: {DEFAULT_SPEED}°/sec")
        print("Commands:")
        print("  - Enter angle (0-180): moves to that angle at default speed")
        print("  - Enter angle,speed (e.g., '45,10'): moves to angle at specified speed")
        print("  - Enter 'fast,angle': instant movement to angle")
        print("  - Enter 'speed,X': change default speed to X degrees/second")
        print("  - Enter 'p' followed by pulse width (500-2500): sets direct pulse")
        print("  - Enter 'status': shows current position")
        print("  - Enter 'q' to quit")
        print("Press Ctrl+C to exit.\n")
        
        current_speed = DEFAULT_SPEED
        
        while True:
            try:
                raw_input = input(f"Enter command (current: {current_angle:.1f}° @ {current_speed}°/s): ").strip().lower()
                
                if raw_input == 'q' or raw_input == 'quit':
                    break
                elif raw_input == 'status':
                    pulse = get_current_pulse_width()
                    print(f"Current position: {current_angle:.1f}° (pulse: {pulse:.0f}μs)")
                elif raw_input.startswith('speed,'):
                    try:
                        new_speed = float(raw_input.split(',')[1])
                        if 1 <= new_speed <= 300:  # Reasonable speed limits
                            current_speed = new_speed
                            print(f"Default speed changed to {current_speed}°/s")
                        else:
                            print("Speed must be between 1 and 300 degrees/second")
                    except (ValueError, IndexError):
                        print("Invalid speed format. Use: speed,30")
                elif raw_input.startswith('fast,'):
                    try:
                        angle = float(raw_input.split(',')[1])
                        if 0 <= angle <= 180:
                            actual_pulse = set_servo_angle(angle, speed=None)  # Instant movement
                            current_angle = angle
                            print(f"Servo moved instantly to {angle}° (pulse: {actual_pulse:.0f}μs)")
                        else:
                            print("Angle must be between 0 and 180 degrees")
                    except (ValueError, IndexError):
                        print("Invalid format. Use: fast,90")
                elif raw_input.startswith('p'):
                    # Direct pulse width control
                    try:
                        pulse_str = raw_input[1:].strip()
                        pulse_width = float(pulse_str)
                        if MIN_PULSE <= pulse_width <= MAX_PULSE:
                            actual_pulse = set_servo_pulse(pulse_width)
                            current_angle = pulse_to_angle(actual_pulse)
                            print(f"Servo set to {actual_pulse}μs pulse width (~{current_angle:.1f}°)")
                        else:
                            print(f"Pulse width must be between {MIN_PULSE} and {MAX_PULSE}μs")
                    except ValueError:
                        print("Invalid pulse width. Use format: p1500")
                elif ',' in raw_input:
                    # Angle with custom speed
                    try:
                        parts = raw_input.split(',')
                        angle = float(parts[0])
                        speed = float(parts[1])
                        if 0 <= angle <= 180 and 1 <= speed <= 300:
                            print(f"Moving to {angle}° at {speed}°/s...")
                            actual_pulse = set_servo_angle(angle, speed=speed)
                            print(f"Servo reached {angle}° (pulse: {actual_pulse:.0f}μs)")
                        else:
                            print("Angle: 0-180°, Speed: 1-300°/s")
                    except ValueError:
                        print("Invalid format. Use: angle,speed (e.g., 45,15)")
                else:
                    # Angle control with default speed
                    try:
                        angle = float(raw_input)
                        if 0 <= angle <= 180:
                            print(f"Moving to {angle}° at {current_speed}°/s...")
                            actual_pulse = set_servo_angle(angle, speed=current_speed)
                            print(f"Servo reached {angle}° (pulse: {actual_pulse:.0f}μs)")
                        else:
                            print("Angle must be between 0 and 180 degrees")
                    except ValueError:
                        print("Please enter a valid command")
                        
            except ValueError:
                print("Invalid input format")
            
            # Small delay for stability
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopping servo control")
    finally:
        # Stop sending pulses and cleanup
        pi.set_servo_pulsewidth(SERVO_PIN, 0)
        pi.stop()
        print("Servo control stopped")

if __name__ == "__main__":
    main()