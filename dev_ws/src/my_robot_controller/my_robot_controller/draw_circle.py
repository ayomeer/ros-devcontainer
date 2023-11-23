# Put interpreter line back if problems come up

# Dependencies (add to package.xml!)
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist 


class DrawCircleNode(Node):
    def __init__(self):
        super().__init__("DrawCircle_Node")
        self.get_logger().info("DrawCircle_Node Start")

        # publisher as member 
        self.cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10) # 10 ~queue size
        self.timer = self.create_timer(2.0, self.send_vel_cmd) # call send_vel_cmd every (create_timer from Node superclass)

    def send_vel_cmd(self):
        msg = Twist()
        msg.linear.x =  2.0
        msg.angular.z = 1.0
        self.cmd_vel_publisher.publish(msg)


def main():
    rclpy.init()
    node = DrawCircleNode()
    rclpy.spin(node)

    rclpy.shutdown()

    