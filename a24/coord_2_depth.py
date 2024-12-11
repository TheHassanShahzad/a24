import rclpy
from rclpy.node import Node
from a24_interfaces.srv import CoordDistance
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge

class CoordDistanceService(Node):
    def __init__(self):
        super().__init__('coord_distance_service')
        self.service = self.create_service(CoordDistance, 'calculate_distance', self.calculate_distance_callback)
        self.subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_image_callback,
            10
        )
        self.bridge = CvBridge()
        self.latest_depth_image = None
        self.get_logger().info('CoordDistance service ready!')

    def depth_image_callback(self, msg):
        try:
            # Convert the ROS Image message to a CV2 image
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")

    def calculate_distance_callback(self, request, response):
        x = request.x
        y = request.y

        if self.latest_depth_image is None:
            self.get_logger().error('No depth image received yet.')
            response.distance = float('nan')
            return response

        # Check if the requested coordinates are within the image bounds
        if x < 0 or x >= self.latest_depth_image.shape[1] or y < 0 or y >= self.latest_depth_image.shape[0]:
            self.get_logger().error(f"Coordinates ({x}, {y}) are out of bounds.")
            response.distance = float('nan')
            return response

        # Retrieve the depth value at the given coordinates
        depth_value = self.latest_depth_image[y, x]

        if np.isnan(depth_value) or depth_value <= 0:
            self.get_logger().error(f"Invalid depth value at ({x}, {y}): {depth_value}")
            response.distance = float('nan')
            return response

        response.distance = float(depth_value)
        self.get_logger().info(f"Distance at ({x}, {y}) is {response.distance} meters.")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = CoordDistanceService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()