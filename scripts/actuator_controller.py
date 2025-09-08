#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Bool

sub_a = None
chan_list = [11, 12]  # Update as needed for your GPIO setup

def callback_a(msg_in):
    if msg_in.data:
        rospy.loginfo("Setting output HIGH!")
        GPIO.output(chan_list, GPIO.HIGH)
    else:
        rospy.loginfo("Setting output LOW!")
        GPIO.output(chan_list, GPIO.LOW)

def shutdown():
    global sub_a
    if sub_a is not None:
        sub_a.unregister()
    GPIO.cleanup()

if __name__ == '__main__':
    rospy.init_node('actuator_controller', anonymous=True)

    GPIO.setmode(GPIO.BOARD)  # Use BOARD pin numbering
    GPIO.setup(chan_list, GPIO.OUT)

    sub_a = rospy.Subscriber('/actuator_control/actuator_a', Bool, callback_a)
    rospy.on_shutdown(shutdown)
    rospy.spin()