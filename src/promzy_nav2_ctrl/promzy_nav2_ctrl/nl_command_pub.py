import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class NLCommandPublisher(Node):
    def __init__(self):
        super().__init__('nl_command_publisher')
        self.publisher_ = self.create_publisher(String, '/nl_commands', 10)
        self.timer = self.create_timer(1.0, self.publish_command)
        self.counter = 0

    def publish_command(self):
        msg = String()
        msg.data = "Go to the kitchen"
        self.publisher_.publish(msg)
        self.counter += 1
        self.get_logger().info(f'Publishing #{self.counter}: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = NLCommandPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
