import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from visualization_msgs.msg import Marker, MarkerArray

def get_waypoints(path_array:list): # path_array in rviz coord
    """Generate waypoints (vertices) from shortest path

    Args:
        path_array (list): List of each individual point (in (x,y) rviz coordinates) of the shortest path

    Returns:
        waypoints_array (list): List of waypoints (in rviz coordinates format)
    """
    markerA = MarkerArray()
    waypoints_array = []
    prev_diff = (path_array[1][0] - path_array[0][0], path_array[1][1] - path_array[0][1])
    for i in range(1,len(path_array)):
        current_diff = (path_array[i][0] - path_array[i-1][0], path_array[i][1] - path_array[i-1][1])
        if prev_diff != current_diff:
            prev_diff = current_diff
            waypoints_array.append(path_array[i-1])
            marker = wp_to_rviz(path_array[i-1],id=i-1)
            markerA.markers.append(marker)
    waypoints_array.append(path_array[-1])
    marker = wp_to_rviz(path_array[-1],id=-1)
    markerA.markers.append(marker)
    return markerA


def wp_to_rviz(
        coord: tuple,
        color: tuple = (0.0, 1.0, 0.0),
        marker_size: float = 0.04,
        id: int = 0
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
    marker.id = id
    marker.ns = str(id)

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
        self.timer = self.create_timer(2, self.timer_callback)

    def timer_callback(self):
        path_array = [(-0.4999999910593034, -0.29999998956918716), (-0.4499999903142453, -0.2499999888241291), (-0.39999998956918725, -0.19999998807907104), (-0.39999998956918725, -0.14999998733401299), (-0.39999998956918725, -0.09999998658895493), (-0.39999998956918725, -0.049999985843896866), (-0.39999998956918725, 1.4901161193847656e-08), (-0.39999998956918725, 0.050000015646219254), (-0.39999998956918725, 0.10000001639127731), (-0.39999998956918725, 0.15000001713633537), (-0.39999998956918725, 0.20000001788139343), (-0.39999998956918725, 0.2500000186264515), (-0.39999998956918725, 0.30000001937150955), (-0.39999998956918725, 0.3500000201165676), (-0.39999998956918725, 0.40000002086162567), (-0.39999998956918725, 0.45000002160668373), (-0.39999998956918725, 0.5000000223517418), (-0.39999998956918725, 0.5500000230967999), (-0.39999998956918725, 0.6000000238418579), (-0.39999998956918725, 0.650000024586916), (-0.39999998956918725, 0.700000025331974), (-0.39999998956918725, 0.7500000260770321), (-0.39999998956918725, 0.8000000268220901), (-0.39999998956918725, 0.8500000275671482), (-0.39999998956918725, 0.9000000283122063), (-0.39999998956918725, 0.9500000290572643), (-0.39999998956918725, 1.0000000298023224), (-0.39999998956918725, 1.0500000305473804), (-0.39999998956918725, 1.1000000312924385), (-0.39999998956918725, 1.1500000320374966), (-0.39999998956918725, 1.2000000327825546), (-0.3499999888241292, 1.2500000335276127), (-0.29999998807907113, 1.3000000342726707), (-0.24999998733401307, 1.3500000350177288), (-0.19999998658895501, 1.3500000350177288), (-0.14999998584389695, 1.4000000357627869), (-0.0999999850988389, 1.450000036507845), (-0.049999984353780835, 1.500000037252903), (1.639127722441458e-08, 1.550000037997961), (0.050000017136335284, 1.600000038743019), (0.10000001788139334, 1.6500000394880772), (0.10000001788139334, 1.7000000402331352), (0.1500000186264514, 1.7500000409781933), (0.1500000186264514, 1.8000000417232513), (0.20000001937150946, 1.8500000424683094), (0.20000001937150946, 1.9000000432133675), (0.20000001937150946, 1.9500000439584255), (0.2500000201165675, 2.0000000447034836), (0.2500000201165675, 2.0500000454485416), (0.2500000201165675, 2.1000000461935997)]
        
        self.get_logger().info("publishing")
        # markerA = get_waypoints(path_array)
        markerA = MarkerArray()
        # markerA.markers.append(wp_to_rviz((-0.45,-0.25)))
        markerA.markers.append(wp_to_rviz((-1.27,0.14)))
        self.marker_pub.publish(markerA)

def main(args=None):
    rclpy.init(args=args)

    firstoccupy = FirstOccupy()
    rclpy.spin(firstoccupy)
    rclpy.shutdown()
    # cv.destroyAllWindows()


if __name__ == "__main__":
    main()
