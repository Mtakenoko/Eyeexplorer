import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

from test import Test

class Predict(Node):

    def __init__(self):
        super().__init__('depth_predict')

        self.bridge = CvBridge()

        self.pub = self.create_publisher(Image, '/endoscope/image/depth', 10)
        self.sub = self.create_subscription(Image, '/endoscope/image/color', self.image_callback, 10)

        self.test = Test()

    def image_callback(self, msg : Image):
        print("received image stamp {}".format(msg._header.frame_id))
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        self.test.load_test_images()
        self.test.depth_predict()
        self.cv2image_publish(self.cv_image)

    def cv2image_publish(self):
        self.pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image))


def main(args=None):
    rclpy.init(args=args)

    node = Predict()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()

        