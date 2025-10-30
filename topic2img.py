"""
Commands:
    source /opt/ros/humble/setup.bash
    python3 topic2img.py
"""
import os

from cv_bridge import CvBridge
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.subscription = self.create_subscription(
            Image,
            '/image',  # current publishing topic
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.frame_id = 0

        self.save_dir = "./dataset"  # save path
        os.makedirs(self.save_dir, exist_ok=True)

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")  # ROS image -> BGR8
            filename = os.path.join(self.save_dir, f"frame_{self.frame_id:06d}.jpg")
            cv2.imwrite(filename, cv_image)

            self.get_logger().info(f"Saved {filename}")
            self.frame_id += 1

        except Exception as e:
            self.get_logger().error(f"Error saving frame: {e}")

# entry
def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
