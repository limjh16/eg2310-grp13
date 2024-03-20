import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import time

class FirstOccupy(Node):
    def __init__(self):
        super().__init__("waypoints")
        self.marker_pub = self.create_publisher(MarkerArray, "/trajectory_node_list", 2)

    def pub(self):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = self.get_clock().now().to_msg()
        
        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 2
        marker.id = 0

        # Set the scale of the marker
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        # Set the color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Set the pose of the marker
        marker.pose.position.x = 0.5
        marker.pose.position.y = 0.1
        marker.pose.position.z = 0.0
        # marker.pose.orientation.x = 0.0
        # marker.pose.orientation.y = 0.0
        # marker.pose.orientation.z = 0.0
        # marker.pose.orientation.w = 1.0

        markerA = MarkerArray()
        markerA.markers.append(marker)

        self.marker_pub.publish(markerA)

def main(args=None):
    rclpy.init(args=args)

    firstoccupy = FirstOccupy()
    firstoccupy.pub()
    time.sleep(100)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rclpy.shutdown()
    # cv.destroyAllWindows()


if __name__ == "__main__":
    main()
