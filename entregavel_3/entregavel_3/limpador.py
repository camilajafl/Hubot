# entregavel_3/limpador.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String                     # <<-- importe String
from geometry_msgs.msg import Twist
from robcomp_util.odom import Odom
from robcomp_util.laser import Laser
import numpy as np

class Limpador(Node, Laser, Odom):
    def __init__(self):
        super().__init__('limpador_node')
        Laser.__init__(self)
        Odom.__init__(self)

        # publisher de direção para os olhos
        self.eyes_pub = self.create_publisher(String, 'eye_direction', 10)

        self.timer = self.create_timer(0.25, self.control)
        self.robot_state = 'forward'
        self.state_machine = {
            'turn': self.turn,
            'forward': self.forward
        }
        self.twist = Twist()
        self.goal_yaw = 0
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def forward(self):
        self.twist.linear.x = 0.2
        print(min(self.front))
        if min(self.front) < 0.5:
            self.twist.linear.x = 0.0
            self.robot_state = 'turn'
            self.goal_yaw = (self.yaw + 5*np.pi/4)

    def turn(self):
        self.twist.angular.z = 0.4
        erro = np.arctan2(np.sin(self.goal_yaw - self.yaw), np.cos(self.goal_yaw - self.yaw))
        print(erro)

        if abs(erro) < np.deg2rad(8):
            self.twist.angular.z = 0.0
            self.robot_state = 'forward'

    def control(self):
        # roda lógica de movimento
        self.state_machine[self.robot_state]()
        self.cmd_vel_pub.publish(self.twist)
        print(self.robot_state)

        # decide direção dos olhos
        if self.twist.angular.z > 0:
            direction = 'right'
        elif self.twist.angular.z < 0:
            direction = 'left'
        else:
            direction = 'forward'

        # publica no tópico
        msg = String()
        msg.data = direction
        self.eyes_pub.publish(msg)

        self.get_logger().info(f'Direction -> {direction}')

def main(args=None):
    rclpy.init(args=args)
    node = Limpador()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
