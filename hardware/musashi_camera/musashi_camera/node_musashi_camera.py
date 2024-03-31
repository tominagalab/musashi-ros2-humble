import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv2
import neoapi

class Camera(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.timer = self.create_timer(0.03, self.timer_callback)
        
    def timer_callback(self):
        pass

def main():
    rclpy.init(args=args)
    camera = Camera()
    rclpy.spin(camera)
    camera.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
