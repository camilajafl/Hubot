import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from rclpy.qos import ReliabilityPolicy, QoSProfile
from std_msgs.msg import String
import cv2
from robcomp_util.module_aruco import Aruco3d
from robcomp_interfaces.msg import DetectionArray, Detection

class ArucoDetector(Node): # Mude o nome da classe

    def __init__(self):
        super().__init__('aruco_node')
        self.running = True

        # Subscribers
        ## Coloque aqui os subscribers
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            CompressedImage, # or CompressedImage
            '/camera/image_raw/compressed', # or '/camera/image_raw/compressed'
            self.image_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        self.flag_sub = self.create_subscription(
            String,
            '/vision/image_flag', # Mude o nome do tópico
            self.flag_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        # Publishers
        self.detections_pub = self.create_publisher(DetectionArray, 'aruco_detection', 10)
        self.Arucos = Aruco3d()

    def flag_callback(self, msg):
        if msg.data.lower() == 'false':
            self.running = False
        elif msg.data.lower() == 'true':
            self.running = True
        print(self.running )

    def image_callback(self, qualquer_coisa_aqui):
        if self.running:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(qualquer_coisa_aqui, "bgr8") # if CompressedImage
            cv_image, results = self.Arucos.detectaAruco(cv_image)

            dections = DetectionArray()
            for result in results:
                cv_image = self.Arucos.drawAruco(cv_image, result)

                detection = Detection()
                detection.classe = str(result['id'][0])
                detection.cx = float(result['centro'][0])
                detection.cy = float(result['centro'][1])

                dections.deteccoes.append(detection)
            
            self.detections_pub.publish(dections)

            cv2.imshow('Image', cv_image)
            cv2.waitKey(1)
            
            # Faça aqui o processamento da imagem
            # ou chame uma classe ou função que faça isso
        else:
            print('Image processing is paused')

    
def main(args=None):
    rclpy.init(args=args)
    ros_node = ArucoDetector() # Mude o nome da classe

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()