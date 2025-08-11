import rclpy
import cv2
import pickle
import face_recognition
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from rclpy.qos import ReliabilityPolicy, QoSProfile
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import os


class FacialNode(Node):

    def __init__(self):
        super().__init__('FacialNode')

        # Publishers
        self.user_pub = self.create_publisher(String, 'recognized_user', 10)
        self.primeiro_pub = self.create_publisher(String, 'primeiro_user', 10)
        self.face_location_pub = self.create_publisher(String, 'primeiro_face_location', 10) 
        local = ''

        self.running = True
        self.bridge = CvBridge()

        # Carregar encodings usando caminho relativo ao pacote
        share_dir = get_package_share_directory('entregavel_3')
        encodings_path = os.path.join(share_dir, 'encodings.pickle')

        with open(encodings_path, 'rb') as f:
            self.known_data = pickle.load(f)

        self.currentname = "Unknown"
        self.unknown_encodings = {}
        self.tracked_names = []
        self.unknown_id_counter = 1
        self.name_map = {}

        # Subscriber
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

    def image_callback(self, msg):
        local = "indefinido" 
        if not self.running:
            return

        # Converter imagem
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        frame_center_x = cv_image.shape[1] // 2 

        # Localizar rostos
        boxes = face_recognition.face_locations(cv_image)
        encodings = face_recognition.face_encodings(cv_image, boxes)
        names = []
        names_in_frame = []

        if not encodings:
            name = "Unknown"

        # Reconhecimento facial
        for encoding in encodings:
            matches = face_recognition.compare_faces(self.known_data["encodings"], encoding)
            name = "Unknown"

            if True in matches:
                matchedIdxs = [i for (i, b) in enumerate(matches) if b]
                counts = {}
                for i in matchedIdxs:
                    name = self.known_data["names"][i]
                    counts[name] = counts.get(name, 0) + 1
                name = max(counts, key=counts.get)

                if self.currentname != name:
                    self.currentname = name
                    self.get_logger().info(f"Identificado: {name}")

            else:
                match_found = False
                tolerance = 0.5
                for unknown_name, prev_encoding in self.unknown_encodings.items():
                    dist = face_recognition.face_distance([prev_encoding], encoding)[0]
                    if dist < tolerance:
                        name = unknown_name
                        match_found = True
                        break
                if not match_found:
                    name = f"Unknown{self.unknown_id_counter}"
                    self.unknown_encodings[name] = encoding
                    self.unknown_id_counter += 1

            names.append(name)
            names_in_frame.append(name)
            if name not in self.tracked_names:
                self.tracked_names.append(name)

        self.tracked_names = [n for n in self.tracked_names if n in names_in_frame]

        # Publicar nome reconhecido
        recognized = next((n for n in self.tracked_names if not n.startswith("Unknown")), "Unknown")
        self.user_pub.publish(String(data=recognized))

        # Publicar primeiro usuário e localização da caixa
        if self.tracked_names:
            first_name = self.tracked_names[0]
            # Localizar a caixa correspondente
            for idx, name in enumerate(names):
                if name == first_name:
                    (top, right, bottom, left) = boxes[idx]
                    face_center_x = (left + right) // 2  # Centro horizontal do rosto
                    face_location_str = f"{top},{right},{bottom},{left}"
                    self.face_location_pub.publish(String(data=face_location_str))

                    # Verificar posição em relação ao centro da imagem
                    if face_center_x < frame_center_x - 200:  # Margem para "esquerda"
                        local = "esquerda"
                        print("esquerda")
                    elif face_center_x > frame_center_x + 200:  # Margem para "direita"
                        local = "direita"
                        print("direita")
                    else:
                        local = "centralizado"
                        print("centralizado")
                    break
        else:
            first_name = "Unknown"
            self.face_location_pub.publish(String(data="-1,-1,-1,-1"))

        self.primeiro_pub.publish(String(data=first_name))
        self.face_location_pub.publish(String(data=local))

        # Desenhar boxes na imagem
        for ((top, right, bottom, left), name) in zip(boxes, names):
            cv2.rectangle(cv_image, (left, top), (right, bottom), (0, 255, 225), 2)
            y = top - 15 if top - 15 > 15 else top + 15
            cv2.putText(cv_image, name, (left, y), cv2.FONT_HERSHEY_SIMPLEX, .8, (0, 255, 255), 2)

        # Desenhar linhas verticais para a zona central
        cv2.line(cv_image, (frame_center_x - 200, 0), (frame_center_x - 200, cv_image.shape[0]), (255, 0, 0), 2)
        cv2.line(cv_image, (frame_center_x + 200, 0), (frame_center_x + 200, cv_image.shape[0]), (0, 0, 255), 2)


        cv2.imshow("Reconhecimento Facial", cv_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = FacialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
