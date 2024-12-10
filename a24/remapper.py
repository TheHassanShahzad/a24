import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelRemapper(Node):
    def __init__(self):
        super().__init__('cmd_vel_remapper')
        
        # Subscriber to the /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Publisher to the /diff_cont/cmd_vel_unstamped topic
        self.publisher = self.create_publisher(
            Twist,
            '/diff_cont/cmd_vel_unstamped',
            10)
        
    def cmd_vel_callback(self, msg):
        # Directly publish the received message to the new topic
        self.publisher.publish(msg)
        # self.get_logger().info('Remapped /cmd_vel to /diff_cont/cmd_vel_unstamped')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRemapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()