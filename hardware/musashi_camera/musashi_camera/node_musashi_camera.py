import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2
import neoapi

class Musashi_Camera(Node):
    def __init__(self):
        super().__init__('node_musashi_camera')
        
        # declare publisher
        self.image_pub = self.create_publisher(Image, '/raw_image', 3)
        
        # declariation prameters
        self.declare_parameter('interval', 0.1)

        # load parameters
        interval = self.get_parameter('interval').get_parameter_value().double_value 

        # neoapi
        # create camera instance & connect
        self.camera = neoapi.Cam()
        self.camera.Connect()

	# cv_bridge
        self.bridge = CvBridge()

        # start timer_callback        
        self.timer = self.create_timer(interval, self.timer_callback)
        
    def timer_callback(self):
        self.get_logger().info('timer callback')
        
        # get image from camera
        image = self.camera.GetImage()

        if not image.IsEmpty():        
            # 
            mat = image.GetNPArray().reshape(image.GetHeight(), image.GetWidth())
            mat = cv2.cvtColor(mat, cv2.COLOR_BayerRG2BGR)
            img_msg = self.bridge.cv2_to_imgmsg(mat, 'bgr8')
            self.image_pub.publish(img_msg)

        
def main(args=None):
    rclpy.init(args=args)
    node = Musashi_Camera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
