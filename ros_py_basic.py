#!/usr/bin/env python3.10

# TO-DO specify python version more cleanly, otherwise get library not found error.
# On Drew's laptop, this is python3.10

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray

# for demonstration
import random

class ezloophwPyPubSub(Node):

    def __init__(self):
        super().__init__('ezloophwpy_pubsub')
        subtopic = 'control_values'
        pubtopic = 'bending_angles'
        self.subscription = self.create_subscription(
            Float64MultiArray,
            subtopic,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Float64MultiArray, pubtopic, 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        print('Created subscription to ', subtopic)
        print('Created publisher for topc ', pubtopic)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        print('I heard: ' + str(msg.data))
    
    def timer_callback(self):
        msg = Float64MultiArray()
        # msg.data = 'Hello World: %d' % self.i
        msg.data = [random.random(), random.random()]  # 2x1 random vector
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    print('Python publisher/subscriber ROS2 minimal demo.')
    rclpy.init(args=args)

    ezloophw_ros2_node = ezloophwPyPubSub()

    # for a cleaner shutdown...
    try:
        rclpy.spin(ezloophw_ros2_node)
    except KeyboardInterrupt:
        print('Caught keyboard interrupt, shutting down python ros2 node...')
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        ezloophw_ros2_node.destroy_node()
        # shutdown will happen automatically if this is the last node (ie MATLAB has already closed), so to avoid confusing errors...
        try:
            rclpy.shutdown()
        except:
            print('ROS2 already shut down.')

if __name__ == '__main__':
    main()