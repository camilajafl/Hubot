import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
import numpy as np
import cv2
from robcomp_interfaces.msg import DetectionArray, Detection
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

"""
ros2 launch my_package first_node.launch.py
"""

class WebCamNode(Node):

    def __init__(self):
        super().__init__('WebCamNode')

        self.img_pub = self.create_publisher(Image, 'webCam', QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.timer = self.create_timer(0.25, self.control)

    def control(self):
        webcam = cv2.VideoCapture(0)
        while True:
            val, image = webcam.read()
            if val:
                cv2.imshow("WebCam", image)
                imagem = CvBridge().cv2_to_imgmsg(image, "bgr8")
                self.img_pub.publish(imagem)
            if cv2.waitKey(1) == 27:
                break

def main(args=None):
    rclpy.init(args=args)
    first_node = WebCamNode()

    rclpy.spin(first_node)

    first_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()