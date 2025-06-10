import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist, Point
import numpy as np
import time
from robcomp_util.odom import Odom
from robcomp_interfaces.msg import Conversation

class Teseu(Node, Odom): # Mude o nome da classe
    def __init__(self, point: Point = Point()):
        Node.__init__(self, 'quadrado_node') # Mude o nome do nó
        Odom.__init__(self) # Mude o nome do nó
        time.sleep(1)
        self.sequencia = {
            'baixo': [(-2,-2), (-0.5,-2),(-0.5,2),(0.5,2), (0.5,0.5), (0.5,2), (-2,2), (-2,0)]
        }

        # Inicialização de variáveis
        self.twist = Twist()
        self.threshold = np.pi/180
        self.kp_linear = 0.6
        self.kp_angular = 0.6
        self.max_vel = 0.3
        self.point = point
        self.index = 0

        self.robot_state = 'admin'
        self.state_machine = {
            'center': self.center,
            'goto': self.goto,
            'stop': self.stop,
            "admin": self.admin,
        }

        self.timer = self.create_timer(0.25, self.control)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.handler_pub = self.create_publisher(Conversation, '/handler', 10)
        msg = Conversation()
        msg.message = "Robo: Estou pronto para explorar"
        self.handler_pub.publish(msg)

        # Subscriber
        self.handler_sub = self.create_subscription(
            Conversation,
            '/handler',
            self.handler_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

    def handler_callback(self, robcomp_eh_muito_legal):
        print(robcomp_eh_muito_legal)
    def get_angular_error(self):
        x = self.point.x - self.x
        y = self.point.y - self.y
        theta = np.arctan2(y , x)

        self.distance = np.sqrt(x**2 + y**2)
        erro = theta - self.yaw
        self.erro = np.arctan2(np.sin(erro), np.cos(erro))

        print('Erro: ', self.erro)
        self.twist.angular.z = self.erro * self.kp_angular

    def center(self):
        self.get_angular_error()

        if abs(self.erro) < np.deg2rad(3):
            self.robot_state = 'goto'

    def goto(self):
        self.get_angular_error()

        if self.distance > 0.01:
            linear_x = self.distance * self.kp_linear
            self.twist.linear.x = min(linear_x, self.max_vel)

        else:
            self.robot_state = 'stop'
    
    def stop(self):
        self.twist = Twist()
        self.robot_state = 'admin'

    def admin(self):
        self.robot_state = 'center'
        self.point = Point()
        self.point.x = float(self.sequencia['baixo'][self.index][0])
        self.point.y = float(self.sequencia['baixo'][self.index][1])
        self.index += 1

    def control(self):
        self.twist = Twist()
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()

        self.cmd_vel_pub.publish(self.twist)
        
            
def main(args=None):
    rclpy.init(args=args)
    ros_node = Teseu(Point( x = -2., y = 0., z = 0.))

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()