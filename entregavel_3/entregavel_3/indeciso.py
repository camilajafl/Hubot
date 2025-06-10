import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from robcomp_util.odom import Odom
from robcomp_util.laser import Laser
# Adicione aqui os imports necessários

class Indeciso_node(Node, Odom, Laser): # Mude o nome da classe

    def __init__(self):
        super().__init__('indeciso_node') # Mude o nome do nó
        Odom.__init__(self)
        Laser.__init__(self)
        self.timer = self.create_timer(0.25, self.control)

        self.robot_state = 'forward'
        self.state_machine = {
            'forward': self.forward,
            'backward': self.backward,
            'stop': self.stop
        }

        # Inicialização de variáveis
        self.twist = Twist()
        self.vel = 0.1

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        ## Coloque aqui os publishers

    def forward(self):
         self.twist.linear.x = self.vel
    
    def backward(self):
        self.twist.linear.x = -self.vel

    def stop(self):
        self.twist.linear.x = 0.0
    
    def check_danger(self):
       pass 

    def control(self):
        frente = min(self.front)
       
        if (frente) < 0.95:
            self.robot_state = 'backward'
        elif (frente) > 1.05:
            self.robot_state = 'forward'
        elif (frente) >= 0.95 and (frente) <=1.05:
            self.robot_state = 'stop'

        self.state_machine[self.robot_state]()

        print(f'Distancia: {frente})')
        print(f'Estado: {self.robot_state}')

        self.cmd_vel_pub.publish(self.twist)
        

def main(args=None):
    rclpy.init(args=args)
    ros_node = Indeciso_node() # Mude o nome da classe

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 