import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Trigger  # Service for saving images
import cv2
import os
from ament_index_python.packages import get_package_share_directory


class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')

        # Get the path to your package
        package_name = 'a24'  # Replace with your package name
        package_path = get_package_share_directory(package_name)

        # Set the output directory to the "images" folder inside the package
        self.output_directory = os.path.join(package_path, 'images')

        # package_path = os.path.join(os.getcwd(), 'src', package_name)
        # self.output_directory = os.path.join(package_path, 'images')

        os.makedirs(self.output_directory, exist_ok=True)  # Create folder if it doesn't exist

        self.declare_parameter('image_topic', '/camera/image_raw')  # Change this to your camera topic
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.latest_image = None  # Store the latest image
        self.image_counter = 0  # To track the number of saved images

        # Create a service to trigger saving an image
        self.service = self.create_service(
            Trigger,
            'save_image',
            self.handle_save_image
        )

        self.get_logger().info(f"Subscribed to topic: {self.image_topic}")
        self.get_logger().info(f"Saving images to: {self.output_directory}")
        self.get_logger().info("Service '/save_image' ready to save images on request.")

    def image_callback(self, msg):
        """Callback to store the latest image."""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def handle_save_image(self, request, response):
        """Service handler to save the latest image."""
        if self.latest_image is None:
            response.success = False
            response.message = "No image received yet."
            return response

        try:
            # Generate filename
            filename = os.path.join(self.output_directory, f"image.png")
            
            # Save the image
            cv2.imwrite(filename, self.latest_image)
            self.get_logger().info(f"Saved image as {filename}")

            response.success = True
            response.message = f"Image saved as {filename}"
        except Exception as e:
            response.success = False
            response.message = f"Error saving image: {e}"

        return response


def main(args=None):
    rclpy.init(args=args)
    image_saver = ImageSaver()
    rclpy.spin(image_saver)
    image_saver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()