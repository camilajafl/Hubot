import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from rclpy.qos import ReliabilityPolicy, QoSProfile
from std_msgs.msg import String
import cv2
from robcomp_util.module_net import MobileNetDetector
from robcomp_interfaces.msg import DetectionArray, Detection

# https://codeshare.io/mobilenet3

class MobilenetDetectorNode(Node): # Mude o nome da classe

    def __init__(self):
        super().__init__('mobilenet_node')
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
        
        self.Detector = MobileNetDetector(CONFIDENCE = 0.7)


        # Publishers
        ## Coloque aqui os publishers
        self.detection_pub = self.create_publisher(DetectionArray, "mobilenet_detection", 10)

    def flag_callback(self, msg):
        if msg.data.lower() == 'false':
            self.running = False
        elif msg.data.lower() == 'true':
            self.running = True
        print(self.running )

    def image_callback(self, robcomp_eh_legal):
        if self.running:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(robcomp_eh_legal, "bgr8") # if CompressedImage
            image, resultados = self.Detector.detect(cv_image)

            detection_array = DetectionArray()
            for resultado in resultados:
                detection = Detection()
                detection.classe = resultado['classe']
                x1, y1, x2, y2 = resultado['bbox']
                detection.cx = (x1 + x2) / 2
                detection.cy = (y1 + y2) / 2
                detection_array.deteccoes.append(detection)
            
            self.detection_pub.publish(detection_array)



            cv2.imshow('Image', image)
            cv2.waitKey(1)
            
            # Faça aqui o processamento da imagem
            # ou chame uma classe ou função que faça isso
        else:
            print('Image processing is paused')

    
def main(args=None):
    rclpy.init(args=args)
    ros_node = MobilenetDetectorNode() # Mude o nome da classe

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()