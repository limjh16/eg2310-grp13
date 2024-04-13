import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import scipy.stats
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import cv2 as cv
#from pyamaze import maze,agent,textLabel
from queue import PriorityQueue
import math
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import scipy.stats
from .lib.maze_manipulation import get_waypoints, dilate123
from .lib.pid_tf2 import move_straight, move_turn, return_cur_pos
from .lib.occupy_nodes import first_scan, a_star_scan, return_odata_origin, a_star_search, go_to_doors
from .lib.open_door_http import open_door

UNKNOWN = 1
UNOCCUPIED = 2
OCCUPIED = 3

# constants
occ_bins = [-1, 0, 50, 100]
path_main = []
dilate_size = 2
global quit
quit = 0

class mapCheck(Node):
    def __init__(self):
        super().__init__("mapcheck")
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.occ_callback,
            10)
        self.subscription  # prevent unused variable warning
        # occdata = np.array([])
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
    
    def occ_callback(self, msg):
        occdata = np.array(msg.data)
        cdata = occdata.reshape(msg.info.height, msg.info.width)
        cdata[cdata == 100] = -1
        cdata[cdata >= 0] = 1
        cdata[cdata == -1] = 0
        no_wall_indexes = np.nonzero(cdata)
        y_dist = np.max(no_wall_indexes[0]) - return_odata_origin()[1]
        print("distance_to_furthest:  "+str(y_dist))
        if (y_dist > (2.15 / msg.info.resolution)):
            print ("!!!!!!!!!!!!!!!!!!!!!!!exit found")
        print( round(float(return_cur_pos().y - msg.info.origin.position.y) / msg.info.resolution)  - return_odata_origin()[1])
        if (round(float(return_cur_pos().y - msg.info.origin.position.y) / msg.info.resolution)  - return_odata_origin()[1]) > (2.15 / msg.info.resolution):
            global quit
            quit = 1
            print("!!!!!!!!!!!!!!!!!!!!!!!!quit!!!!!!!!!!!!!!!!!!")
        

def main(args=None):
    rclpy.init(args=args)

    first_scan()

    # create matplotlib figure
    plt.ion()
    plt.show()
    mapcheck = mapCheck()

   
    for _ in range(3):
        path_main = a_star_scan()

        outwps = get_waypoints(path_main)
        print("out waypoints: " + str(outwps))
        for x in outwps:
            print(x)
            # time.sleep(2)
            move_turn(x)
            if quit:
                break
            # time.sleep(1)
            move_straight(x)
            # time.sleep(1)

            rclpy.spin_once(mapcheck)
        if quit:
            print("quit, at maze exit")
            break

    path_main = go_to_doors()
    outwps = get_waypoints(path_main)
    print("out waypoints: " + str(outwps))
    for x in outwps:
        print(x)
        # time.sleep(2)
        move_turn(x)
        # time.sleep(1)
        move_straight(x)
        # time.sleep(1)

        rclpy.spin_once(mapcheck)
    

    # door = open_door("192.168.67.")
    plt.close()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # occupy.destroy_node()
    rclpy.shutdown()
    # cv.destroyAllWindows()


if __name__ == "__main__":
    main()
