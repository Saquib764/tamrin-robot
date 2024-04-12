#!/usr/bin/env python

import rclpy

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('my_pkg_test_node')
    node.get_logger().info('Hello ROS2')
    rclpy.spin(node)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()

