#!/usr/bin/env python3
# ^ linux style: run env program to find python3 in PATH 

import rclpy 
from rclpy.node import Node

# Node Class (class my_class(parent_class))
class MyNode(Node): 
    def __init__(self, node_name):
        super().__init__(node_name) # call parent ctor to set name
        self.get_logger().info("Initializing " + node_name)

        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("timer callback message")


def main(args=None):
    rclpy.init(args=args)       # Initialize ROS2 communication
    
    node = MyNode("my_node")    # Create actual node
    rclpy.spin(node)            # keep node running with callbacks enabled

    rclpy.shutdown()            # End of module -> shutdown 


if __name__ == '__main__':
    main()
