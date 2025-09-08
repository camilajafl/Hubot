import os, pickle, threading, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import face_recognition

class EnrollNode(Node):
    def __init__(self):
        super().__init__('enroll_node')

        # onde fica sua base gravável
        self.encodings_path = os.path.expanduser('~/.ros/encodings.pickle')
        os.makedirs(os.path.dirname(self.encodings_path), exist_ok=True)

        self.bridge = CvBridge()
        self.last_frame_bgr = None
        self.lock = threading.Lock()

        # assina câmera (mesmo tópico do FacialNode)
        self.create_subscription(CompressedImage, '/image_raw/compressed',
                                 self._img_cb, 10)

        # assina pedidos de cadastro (nome)
        self.create_subscription(String, 'captura_nome', self._captura_cb, 10)

        # publica status opcional
        self.status_pub = self.create_publisher(String, 'enroll_status', 10)

        # cliente p/ recarregar encodings no FacialNode
        self.reload_cli = self.create_client(Empty, 'reload_encodings')

        self.num_photos = int(os.getenv('ENROLL_NUM_PHOTOS', '8'))  # override por env
        self.get_logger().info(f'EnrollNode pronto. num_photos={self.num_photos}')

    def _img_cb(self, msg: CompressedImage):
        try:
            self.last_frame_bgr = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f'Falha na conversão da imagem: {e}')

    def _read_pickle(self):
        if os.path.exists(self.encodings_path):
            with open(self.encodings_path, 'rb') as f:
                return pickle.load(f)
        return {"encodings": [], "names": []}

    def _atomic_write_pickle(self, data):
        tmp = self.encodings_path + '.tmp'
        with open(tmp, 'wb') as f:
            pickle.dump(data, f)
        os.replace(tmp, self.encodings_path)

    def _status(self, txt):
        self.get_logger().info(txt)
        self.status_pub.publish(String(data=txt))

    def _wait_for_face(self, timeout=4.0):
        """retorna um encoding quando houver exatamente 1 face no frame; senão None"""
        end = time.time() + timeout
        while time.time() < end:
            frame = self.last_frame_bgr
            if frame is None:
                rclpy.spin_once(self, timeout_sec=0.05); continue
            # (opcional) redimensionar para estabilidade e velocidade
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            boxes = face_recognition.face_locations(rgb, model='hog')
            if len(boxes) == 1:
                encs = face_recognition.face_encodings(rgb, boxes)
                if encs:
                    return encs[0]
            rclpy.spin_once(self, timeout_sec=0.05)
        return None

    def _captura_cb(self, msg: String):
        name = msg.data.strip()
        if not name:
            self._status('Nome vazio. Abortando.')
            return

        n = max(5, min(self.num_photos, 20))
        self._status(f'Iniciando cadastro de "{name}" com {n} amostras…')

        encs = []
        tentativas = 0
        while len(encs) < n and tentativas < n*3:
            tentativas += 1
            enc = self._wait_for_face(timeout=4.0)
            if enc is not None:
                encs.append(enc)
                self._status(f'OK {len(encs)}/{n}')
            else:
                self._status('Frame inválido (sem face única). Tentando novamente…')

        if not encs:
            self._status('Falha: não foi possível obter amostras válidas.')
            return

        with self.lock:
            data = self._read_pickle()
            data['encodings'].extend(encs)
            data['names'].extend([name]*len(encs))
            self._atomic_write_pickle(data)

        # pede pro FacialNode recarregar
        if self.reload_cli.wait_for_service(timeout_sec=2.0):
            fut = self.reload_cli.call_async(Empty.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)

        self._status(f'Usuário "{name}" cadastrado com {len(encs)} amostras.')

def main(args=None):
    rclpy.init(args=args)
    node = EnrollNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
