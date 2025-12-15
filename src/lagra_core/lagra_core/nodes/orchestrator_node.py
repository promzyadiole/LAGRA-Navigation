import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from lagra_core.navigation.nav2_controller import Nav2Controller


class OrchestratorNode(Node):
    def __init__(self):
        super().__init__("orchestrator_node")

        self.nav = Nav2Controller()
        self.nav.initialize()

        self.subscription = self.create_subscription(
            String,
            "/lagra/user_command",
            self.command_callback,
            10
        )

        self.get_logger().info("Orchestrator Node started.")

    def command_callback(self, msg):
        text = msg.data.lower()

        if "garage" in text:
            self.get_logger().info("Navigating to garage and capturing scene.")
            self.nav.go_to_pose(3.2, -5.5, -1.57)
            # Trigger BLIP-2 caption here

        elif "parlour" in text:
            self.nav.go_to_pose(-3.0, 0.0, -1.57)
            # Trigger BLIP-2 caption here


def main():
    rclpy.init()
    node = OrchestratorNode()
    rclpy.spin(node)
    rclpy.shutdown()
