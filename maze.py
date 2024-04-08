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
from .lib.pid_tf2 import move_straight, move_turn
from .lib.occupy_nodes import first_scan, a_star_scan
from .lib.open_door_http import open_door

UNKNOWN = 1
UNOCCUPIED = 2
OCCUPIED = 3

# constants
occ_bins = [-1, 0, 50, 100]
path_main = []
dilate_size = 2

def main(args=None):
    rclpy.init(args=args)

    first_scan()

    # create matplotlib figure
    plt.ion()
    plt.show()

    for _ in range(3):
        path_main = a_star_scan()

        outwps = get_waypoints(path_main)
        print("out waypoints: " + str(outwps))
        for x in outwps:
            print(x)
            # time.sleep(2)
            move_turn(x)
            # time.sleep(1)
            move_straight(x)
            # time.sleep(1)

    plt.close()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # occupy.destroy_node()
    rclpy.shutdown()
    # cv.destroyAllWindows()


if __name__ == "__main__":
    main()
