import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from robcomp_util.odom import Odom
from robcomp_util.laser import Laser
import numpy as np

class Limpador(Node, Laser, Odom):
    def __init__(self):
        super().__init__('limpador_node')
        Laser.__init__(self)
        Odom.__init__(self)
        
        
        self.timer = self.create_timer(0.25, self.control)

        self.robot_state = 'forward'

        self.state_machine = {
            'turn': self.turn,
            'forward' : self.forward
        }

        self.twist = Twist()
        self.goal_yaw = 0
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def forward(self):
        self.twist.linear.x = 0.2
        print(f"front sample: {self.front[:5]}  min: {min(self.front):.2f}")


        if min(self.front) < 0.5:
            self.twist.linear.x = 0.0
            self.robot_state = 'turn'
            self.goal_yaw = (self.yaw +  5*np.pi/ 4)
    
    def turn(self):
        self.twist.angular.z = 0.4

        erro_posicao = np.arctan2(np.sin(self.goal_yaw - self.yaw), np.cos(self.goal_yaw - self.yaw))

        if abs(erro_posicao) < np.deg2rad(8):
            self.twist.angular.z = 0.0
            self.robot_state = 'forward'
        
    
    def control(self):
        self.state_machine[self.robot_state]()
        print(self.twist)
        self.cmd_vel_pub.publish(self.twist)
        print(f'Estado Atual: {self.robot_state}')

    
def main(args=None):
    rclpy.init(args=args)
    ros_node = Limpador()

    
    rclpy.spin(ros_node)
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    

    






