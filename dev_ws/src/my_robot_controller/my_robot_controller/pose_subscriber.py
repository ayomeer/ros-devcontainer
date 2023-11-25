import rclpy 
from rclpy.node import Node
from turtlesim.msg import Pose

class PoseSubscriberNode(Node): 
    def __init__(self):
        super().__init__("pose_subscriber") # call parent ctor to set name
        self.get_logger().info("Initializing pose_subscriber")
        self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10) 

    def pose_callback(self, msg: Pose):
        self.get_logger().info("x: {:.2f}, y: {:.2f}".format(msg.x, msg.y))


def main(args=None):
    rclpy.init(args=args)       
    
    node = PoseSubscriberNode()
    rclpy.spin(node)          

    rclpy.shutdown()            