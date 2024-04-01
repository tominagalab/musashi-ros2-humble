import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv2
import neoapi

class Musashi_Camera(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        self.declare_parameter('interval', 0.1)


        interval = self.get_parameter('interval').as_double()     
        self.timer = self.create_timer(interval, self.timer_callback)
        
    def timer_callback(self):
        self.get_logger().info('timer callback')

def main():
    rclpy.init(args=args)
    node = Musashi_Camera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
