from rclpy.node import Node
import rclpy
from std_msgs.msg import String
from lagra_core.vlm.blip2 import BLIP2Captioner
from lagra_core.perception.image_buffer import ImageBuffer

class VLMNode(Node):
    def __init__(self):
        super().__init__("vlm_node")
        self.img = ImageBuffer(self, "/camera/image_raw")
        self.captioner = BLIP2Captioner()
        self.pub = self.create_publisher(String, "/lagra/scene_caption", 10)

        self.create_subscription(String, "/lagra/caption_request", self.on_request, 10)

    def on_request(self, _msg):
        frame = self.img.get_latest()
        if frame is None:
            self.get_logger().warn("No image yet.")
            return
        caption = self.captioner.caption(frame)
        self.pub.publish(String(data=caption))
        self.get_logger().info(f"BLIP-2: {caption}")

def main():
    rclpy.init()
    node = VLMNode()
    rclpy.spin(node)
    rclpy.shutdown()

