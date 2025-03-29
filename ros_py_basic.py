#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray

# for demonstration
import random

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        subtopic = 'control_values'
        self.subscription = self.create_subscription(
            Float64MultiArray,
            subtopic,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        print('Created subscription to ', subtopic)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        print('I heard: ' + str(msg.data))

# publisher and subscriber in same module for ease

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        pubtopic = 'bending_angles'
        self.publisher_ = self.create_publisher(Float64MultiArray, pubtopic, 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Float64MultiArray()
        # msg.data = 'Hello World: %d' % self.i
        msg.data = [random.random(), random.random()]  # 2x1 random vector
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


# def main(args=None):
#     rclpy.init(args=args)

#     minimal_publisher = MinimalPublisher()

#     rclpy.spin(minimal_publisher)

#     # Destroy the node explicitly
#     # (optional - otherwise it will be done automatically
#     # when the garbage collector destroys the node object)
#     minimal_publisher.destroy_node()
#     rclpy.shutdown()

def main(args=None):
    print('Python publisher/subscriber ROS2 minimal demo.')
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_subscriber) # ends when matlab ends
    rclpy.spin(minimal_publisher) # ends when we end in python

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    minimal_publisher.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()

# import rospy
# from std_msgs.msg import Float64MultiArray
# import random

# def control_values_callback(data):
#     rospy.loginfo("Received from /control_values: %s", data.data)

# def bending_publisher():
#     rospy.init_node('bending_node', anonymous=True)

#     # Publisher
#     pub = rospy.Publisher('/bending_angles', Float64MultiArray, queue_size=10)

#     # Subscriber
#     rospy.Subscriber('/control_values', Float64MultiArray, control_values_callback)

#     rate = rospy.Rate(10)  # 10 Hz

#     rospy.loginfo("Publishing random bending angles to /bending_angles at 10Hz...")

#     while not rospy.is_shutdown():
#         msg = Float64MultiArray()
#         msg.data = [random.random(), random.random()]  # 2x1 random vector
#         pub.publish(msg)
#         rospy.loginfo("Published to /bending_angles: %s", msg.data)
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         bending_publisher()
#     except rospy.ROSInterruptException:
#         pass