import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

class GoalPoseSubscriber(Node):
    def __init__(self):
        super().__init__('goal_pose_subscriber')
        self.navigator = BasicNavigator()

        self.subscription = self.create_subscription(
            Float64MultiArray, '/nav2_goal_pose', self.goal_callback, 10)
        
        self.get_logger().info("Subscribed to /nav2_goal_pose topic")

    def create_pose_stamped(self, navigator, position_x, position_y, rotation_z):
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, rotation_z)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = position_x
        goal_pose.pose.position.y = position_y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = q_x
        goal_pose.pose.orientation.y = q_y
        goal_pose.pose.orientation.z = q_z
        goal_pose.pose.orientation.w = q_w
        return goal_pose

    def goal_callback(self, msg):

        if len(msg.data) == 3:
            goal_x = msg.data[0]
            goal_y = msg.data[1]
            goal_theta = msg.data[2]

            self.get_logger().info(f"Goal: x={goal_x}, y={goal_y}, theta={goal_theta}")

            goal_pose = self.create_pose_stamped(self.navigator, goal_x, goal_y, goal_theta)
            self.navigator.goToPose(goal_pose)
            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                self.get_logger().info(f"Feedback: {feedback}")

            self.get_logger().info(f"Result: {self.navigator.getResult()}")

        else:
            self.get_logger().error("Invalid goal pose message received")

def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()