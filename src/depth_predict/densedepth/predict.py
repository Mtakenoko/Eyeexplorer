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
        self.sub = self.create_subscription(Image, '/endoscope_image', self.image_callback, 10)

        self.test = Test()

    def image_callback(self, msg : Image):
        print("received image stamp {}".format(msg._header.frame_id))
        cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        self.test.input_image(cv_image)
        outputs = self.test.depth_predict()
        self.cv2image_publish(outputs[0])
        self.test.display_images(outputs)

    def cv2image_publish(self, image):
        self.pub.publish(self.bridge.cv2_to_imgmsg(image))

def main(args=None):
    rclpy.init(args=args)

    node = Predict()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()