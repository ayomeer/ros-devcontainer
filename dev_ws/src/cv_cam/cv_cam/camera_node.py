#!/usr/bin/env python3
# ^ linux style: run env program to find python3 in PATH 

import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image # publishing msg_type: sensor_msgs/msg/Image
import cv2 
from cv_bridge import CvBridge # openCV <-> ROS bridge


class ImagePublisher(Node): 
    def __init__(self):
        super().__init__("camera_node") 
        self.get_logger().info("Initializing camera_node")

        self.create_publisher(Image, 'image_raw', 10)
        self.get_logger().info("Create publisher")

        # VideoCapture(const String &filename, int apiPreference)
        self.cap = cap = cv2.VideoCapture(
            """nvarguscamerasrc ! video/x-raw(memory:NVMM),format=NV12,
            width=640,height=480,framerate=30/1 ! nvvidconv ! 
            video/x-raw,format=BGRx ! videoconvert ! 
            video/x-raw,format=BGR ! appsink drop=1""", cv2.CAP_GSTREAMER) 
        
        # self.cv_bridge = CvBridge()
        

    #     self.create_timer(1.0, self.timer_callback)

    # def timer_callback(self):
    #     self.get_logger().info("Publish video frame")
    #     ret, frame = self.cap.read()                                  
        
    #     if ret == True:                                                
    #         self.publisher.publish(
    #             self.cv_bridge.cv2_to_imgmsg(frame, 'bgr8'))



def main(args=None):
    rclpy.init(args=args)       # Initialize ROS2 communication
    
    node = ImagePublisher()    # Create actual node
    rclpy.spin(node)            # keep node running with callbacks enabled
    rclpy.shutdown()            # End of module -> shutdown 


if __name__ == '__main__':
    main()
