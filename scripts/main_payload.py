#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Bool
import time

# GPIO Setup
servo_pin = 12  # Servo control pin
chan_list = [12, 13]  # Other actuator pins
GPIO.setmode(GPIO.BOARD)
GPIO.setup(servo_pin, GPIO.OUT)
GPIO.setup(chan_list, GPIO.OUT)

# Initialize PWM
pwm = GPIO.PWM(servo_pin, 50)  # 50 Hz frequency
pwm.start(0)

# Function to convert angle to duty cycle
def angle_to_duty_cycle(angle):
    return (angle / 18.0) + 2.5

def callback_a(msg_in):
    if msg_in.data:
        rospy.loginfo("Moving servo sequence and setting output high!")
        # Move 30 degrees clockwise
        pwm.ChangeDutyCycle(angle_to_duty_cycle(30))
        time.sleep(1)
        
        # Move 60 degrees counterclockwise from current position
        # Current position is 30, so moving to -30 degrees (30 - 60 = -30)
        pwm.ChangeDutyCycle(angle_to_duty_cycle(-30))
        time.sleep(1)
        
        GPIO.output(chan_list, GPIO.HIGH)
    else:
        rospy.loginfo("Resetting servo and setting output low!")
        pwm.ChangeDutyCycle(0)  # Stop servo
        GPIO.output(chan_list, GPIO.LOW)

def shutdown():
    if 'sub_a' in globals():
        sub_a.unregister()
    pwm.stop()
    GPIO.cleanup()

if __name__ == '__main__':
    try:
        # Setup the ROS node
        rospy.init_node('actuator_controller', anonymous=True)
        
        # Setup the subscriber
        sub_a = rospy.Subscriber('/actuator_control/actuator_a', Bool, callback_a)
        
        # Register shutdown hook
        rospy.on_shutdown(shutdown)
        
        # Keep the node running
        rospy.spin()
        
    except KeyboardInterrupt:
        shutdown()