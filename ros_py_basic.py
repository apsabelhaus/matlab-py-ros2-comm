#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
import random

def control_values_callback(data):
    rospy.loginfo("Received from /control_values: %s", data.data)

def bending_publisher():
    rospy.init_node('bending_node', anonymous=True)

    # Publisher
    pub = rospy.Publisher('/bending_angles', Float64MultiArray, queue_size=10)

    # Subscriber
    rospy.Subscriber('/control_values', Float64MultiArray, control_values_callback)

    rate = rospy.Rate(10)  # 10 Hz

    rospy.loginfo("Publishing random bending angles to /bending_angles at 10Hz...")

    while not rospy.is_shutdown():
        msg = Float64MultiArray()
        msg.data = [random.random(), random.random()]  # 2x1 random vector
        pub.publish(msg)
        rospy.loginfo("Published to /bending_angles: %s", msg.data)
        rate.sleep()

if __name__ == '__main__':
    try:
        bending_publisher()
    except rospy.ROSInterruptException:
        pass