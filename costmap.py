import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from nav_msgs.msg import OccupancyGrid
import numpy as np
from .lib.maze_manipulation import inflate
from array import array as Array


class Costmap(Node):
    def __init__(self):
        super().__init__("costmap_pub")
        self.subscription = self.create_subscription(
            OccupancyGrid, "/map", self.listener_callback, qos_profile_sensor_data
        )
        self.publisher = self.create_publisher(
            OccupancyGrid, "/global_costmap/costmap", qos_profile_system_default
        )
        self.get_logger().info("Costmap Publishing...")

    def listener_callback(self, msg):
        occdata = np.array(msg.data)
        odata = inflate(
            occdata.reshape(msg.info.height, msg.info.width),
            dilate=int((0.243 / 2) // msg.info.resolution + 1),
            inflation_radius=5,
            inflation_step=5,
            threshold=60,
            erode=6,
        )
        omap = OccupancyGrid()
        omap.data = Array("b", odata.ravel().astype(np.int8))
        omap.info = msg.info
        omap.header = msg.header
        self.publisher.publish(omap)


def main(args=None):
    rclpy.init(args=args)

    costmap = Costmap()
    rclpy.spin(costmap)

    costmap.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
