import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class TurtleScannerNode(Node):
    def __init__(self):
        super().__init__('turtle_scanner_node')

        self.pose_scanner = None
        self.pose_target  = None

        self.create_subscription(Pose, '/turtle1/pose', self.cb_scanner, 10)
        self.create_subscription(Pose, '/turtle_target/pose', self.cb_target, 10)

        self.get_logger().info('Nœud scanner démarré !')

    def cb_scanner(self, msg):
        self.pose_scanner = msg

    def cb_target(self, msg):
        self.pose_target = msg

def main():
    rclpy.init()
    node = TurtleScannerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
