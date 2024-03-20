import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from visualization_msgs.msg import Marker, MarkerArray


def wp_to_rviz(
        coord: tuple,
        color: tuple = (0.0, 1.0, 0.0),
        marker_size: float = 0.04
    ):
    """Convert tuple of (x,y) into a Marker object
    Returns: Marker (append into marker array)

    Args:
        coord (tuple): (x,y) in floats, rviz coordinates
        color (tuple): (r,g,b) in floats (default green)
        marker_size (float): size of marker
    """
    marker = Marker()
    marker.header.frame_id = "/map"

    # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    marker.type = 1

    # Set the scale of the marker
    marker.scale.x = marker_size
    marker.scale.y = marker_size
    marker.scale.z = marker_size

    # Set the color
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = 1.0

    # Set the pose of the marker
    marker.pose.position.x = coord[0]
    marker.pose.position.y = coord[1]
    marker.pose.position.z = marker_size

    marker.lifetime = Duration(seconds=10).to_msg()

    return marker


class FirstOccupy(Node):
    def __init__(self):
        super().__init__("waypoints")
        self.marker_pub = self.create_publisher(
            MarkerArray, "/trajectory_node_list", 5
        )
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("publishing")
        markerA = MarkerArray()
        marker = wp_to_rviz((0.25,2.1))
        markerA.markers.append(marker)
        self.marker_pub.publish(markerA)

def main(args=None):
    rclpy.init(args=args)

    firstoccupy = FirstOccupy()
    rclpy.spin(firstoccupy)
    rclpy.shutdown()
    # cv.destroyAllWindows()


if __name__ == "__main__":
    main()
