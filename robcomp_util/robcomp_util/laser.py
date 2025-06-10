import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
import numpy as np

class Laser():

    def __init__(self):
        self.opening = 5
        
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
      
        rclpy.spin_once(self) # Roda pelo menos uma vez para pegar os valores de x, y e front

    def laser_callback(self, msg: Odometry):
        self.laser_msg = np.array(msg.ranges).round(decimals=2)
        self.laser_msg[self.laser_msg == 0] = np.inf
        self.laser_msg = list(self.laser_msg)

        self.left = self.laser_msg[90-self.opening:90+self.opening]
        self.right = self.laser_msg[270-self.opening:270+self.opening]
        self.back = self.laser_msg[180-self.opening:180+self.opening]
        self.front = self.laser_msg[-self.opening:] + self.laser_msg[:self.opening]

