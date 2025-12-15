from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PIL import Image as PILImage
import cv2

class ImageBuffer:
    def __init__(self, node: Node, topic="/camera/image_raw"):
        self.node = node
        self.bridge = CvBridge()
        self.last_pil = None
        self.sub = node.create_subscription(Image, topic, self.cb, 10)

    def cb(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        self.last_pil = PILImage.fromarray(rgb)

    def get_latest(self):
        return self.last_pil
