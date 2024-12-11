import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from a24_interfaces.srv import CheckCoordinates

class MapOccupancyService(Node):
    def __init__(self):
        super().__init__('map_occupancy_service')
        self.map_data = None  # To store the map data

        # Subscription to the /map topic
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Service to check coordinate occupancy
        self.service = self.create_service(
            CheckCoordinates,
            'check_coordinate',
            self.handle_check_coordinate
        )

        self.get_logger().info('Map Occupancy Service is ready.')

    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().info('Map updated.')

    def handle_check_coordinate(self, request, response):
        if self.map_data is None:
            self.get_logger().warning('Map data is not yet available.')
            response.success = False
            response.is_occupied = False
            return response

        # Convert the (x, y) coordinates to map indices
        x = request.x
        y = request.y
        resolution = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y

        # Compute grid indices
        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)

        if grid_x < 0 or grid_y < 0 or grid_x >= self.map_data.info.width or grid_y >= self.map_data.info.height:
            self.get_logger().warning('Coordinates out of map bounds.')
            response.success = False
            response.is_occupied = False
            return response

        # Calculate the index in the data array
        index = grid_y * self.map_data.info.width + grid_x
        occupancy_value = self.map_data.data[index]

        # Check occupancy
        if occupancy_value == -1:
            self.get_logger().info('Cell is unknown.')
            response.success = False
            response.is_occupied = False
        elif occupancy_value >= 50:
            self.get_logger().info('Cell is occupied.')
            response.success = True
            response.is_occupied = True
        else:
            self.get_logger().info('Cell is free.')
            response.success = True
            response.is_occupied = False

        return response

def main(args=None):
    rclpy.init(args=args)
    node = MapOccupancyService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
