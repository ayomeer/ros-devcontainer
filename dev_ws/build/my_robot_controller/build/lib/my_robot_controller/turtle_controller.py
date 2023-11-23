import rclpy 
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist 

import numpy as np

centerPoint = np.array([5,5])

class Controller(Node): 
    def __init__(self):
        super().__init__("turtle_controller") # call parent ctor to set name
        self.get_logger().info("Initializing turtle_controller")
        self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10) 
        self.cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def pose_callback(self, pose: Pose):
        cmd = Twist() # prep publish variable

        # "controller"
        p = np.array([pose.x, pose.y])
        r = np.linalg.norm(p-centerPoint) 
        if r > 3: # far from center
            cmd.linear.x = 1.0
            cmd.angular.z = 0.8
        
        else: # close to center
            cmd.linear.x = 2.0
            cmd.angular.z = 0.0

        # publish pose to /turtle1/cmd_vel for turtlesim to read
        self.cmd_vel_publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)         
    node = Controller()
    rclpy.spin(node)           
    rclpy.shutdown()            