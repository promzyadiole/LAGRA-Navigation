from rclpy.node import Node
import rclpy

class LLMNode(Node):
    def __init__(self):
        super().__init__("llm_node")

    def parse_intent(self, text):
        return {
            "intent": "NAVIGATE_AND_DESCRIBE",
            "target": "garage"
        }


def main():
    rclpy.init()
    node = LLMNode()
    rclpy.spin(node)
    rclpy.shutdown()
