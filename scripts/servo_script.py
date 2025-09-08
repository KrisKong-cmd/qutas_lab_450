import RPi.GPIO as GPIO
import time

servo_pin = 11  # Replace with actual GPIO pin number

GPIO.setmode(GPIO.BOARD)
GPIO.setup(servo_pin, GPIO.OUT)

pwm = GPIO.PWM(servo_pin, 50)  # 50 Hz
pwm.start(0)

try:
    while True:
        print("Rotate to 150° (clockwise)")
        pwm.ChangeDutyCycle(10.8)
        time.sleep(1)

        print("Rotate to 30° (anticlockwise)")
        pwm.ChangeDutyCycle(4.2)
        time.sleep(1)

except KeyboardInterrupt:
    pwm.stop()
    GPIO.cleanup()