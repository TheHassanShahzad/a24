import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations


def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Define the quaternion from Euler angles
    qx, qy, qz, qw = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)

    # Create the initial pose message
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()

    # Set the position and orientation
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.position.z = 0.0
    initial_pose.pose.orientation.x = qx
    initial_pose.pose.orientation.y = qy
    initial_pose.pose.orientation.z = qz
    initial_pose.pose.orientation.w = qw

    # Set the initial pose
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2 to be active
    navigator.waitUntilNav2Active()

    goal_pose=PoseStamped()
    goal_pose.header.frame_id='map'
    goal_pose.header.stamp=navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x=3.0
    goal_pose.pose.position.y=3.0
    goal_pose.pose.position.z=0.0
    goal_pose.pose.orientation.x=qx
    goal_pose.pose.orientation.y=qy
    goal_pose.pose.orientation.z=qz
    goal_pose.pose.orientation.w=qw

    navigator.goToPose(goal_pose)
    while not navigator.isTaskComplete():
        feedback=navigator.getFeedback()
        print(feedback)


    # Shutdown rclpy properly
    rclpy.shutdown()


if __name__ == '__main__':
    main()
