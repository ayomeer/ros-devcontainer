import rclpy
from rclpy import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("test node")
        self.get_logger().info("meep")


def main(args=None):
    rclpy.init(args=args)
    node=MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()