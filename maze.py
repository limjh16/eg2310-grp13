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
from .lib.occupy_nodes import delta_to_origin, first_scan, a_star_scan, return_odata_origin, a_star_search, go_to_doors, convert_to_odata, return_odata_origin_delta
from .lib.open_door_http import open_door
from .lib.bucket_utils import move_to_bucket
from .lib.servo_client import launch_servo
from .lib.dumb_move import time_straight, odom_turn
from .lib.lobby import door_mover

UNKNOWN = 1
UNOCCUPIED = 2
OCCUPIED = 3

# constants
occ_bins = [-1, 0, 50, 100]
path_main = []
dilate_size = 2
global quit
quit = 0
ipaddr = "192.168.bla.bla"
lobby_coord = (1.8,2.8) # supposed to be 2.8

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
        # self.tfBuffer = tf2_ros.Buffer()
        # self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
    
    def occ_callback(self, msg):
        occdata = np.array(msg.data)
        cdata = occdata.reshape(msg.info.height, msg.info.width)
        cdata[cdata == 100] = -1
        cdata[cdata >= 0] = 1
        cdata[cdata == -1] = 0
        no_wall_indexes = np.nonzero(cdata)
        odata_lobby_coord = delta_to_origin(convert_to_odata(lobby_coord, msg.info.origin.position.x, msg.info.origin.position.y))
        # y_dist = np.max(no_wall_indexes[0]) - return_odata_origin()[1]
        # print("distance_to_furthest:  "+str(y_dist))
        # if (y_dist > (3 / msg.info.resolution)):
        if (odata_lobby_coord[0] in no_wall_indexes[1] and odata_lobby_coord[1] in no_wall_indexes[0]):
            global quit
            quit = 1
            print("!!!!!!!!!!!!!!!!!!!!!!!!quit!!!!!!!!!!!!!!!!!!")
        

def main(args=None):
    rclpy.init(args=args)
    # door_mover(1)
    # time.sleep(100)
    
    # time_straight(0.1,1)
    time_straight(-0.1,4)
    first_scan()

    # create matplotlib figure
    plt.ion()
    plt.show()
    mapcheck = mapCheck()


    for _ in range(15):
        path_main = a_star_scan()

        outwps = get_waypoints(path_main)
        print("out waypoints: " + str(outwps))
        time_start = time.time()
        for x in outwps:
            print(x)
            # time.sleep(2)
            if quit:
                break
            if time.time()-time_start > 20:
                break
            # will reset once every 20 seconds unless exit is seen: if exit seen, will move directly to exit and skip the resets.
            # once exit is seen, don't reset anymore (exitbreak will never equal 1) until quit is called
            move_turn(x)
            # time.sleep(1)
            move_straight(x)
            # time.sleep(1)

            rclpy.spin_once(mapcheck)
        if quit:
            print("quit, can see elevator")
            break

    print("---------------going to lobby!---------------")
    path_main = go_to_doors(goal=lobby_coord)
    outwps = get_waypoints(path_main)
    print("out waypoints: " + str(outwps))
    for x in outwps:
        print(x)
        # time.sleep(2)
        move_turn(x)
        # time.sleep(1)
        move_straight(x)
        # time.sleep(1)

    # print("---------------http call!---------------")
    # door = 0
    # while door == 0:
    #     door = open_door(ipaddr)
    door = 2

    print("!!!!!!!!!!!!!!!---------------waiting 10s, pls open door!---------------!!!!!!!!!!!!!!!!!!!")
    time.sleep(10)
    

    print("---------------going to door "+str(door)+"!---------------")
    # face front
    move_turn((return_cur_pos().x, return_cur_pos().y+5))
    # door_mover(door)
    # move_turn((return_cur_pos().x, return_cur_pos().y+5)) # to refresh cur_pos after door_mover
    if door == 1:
        turndeg = -90
    elif door == 2:
        turndeg = 90
    # odom_turn(turndeg)
    move_turn((return_cur_pos().x+turndeg, return_cur_pos().y))
    time_straight(0.15, 3)

    # door_coord = lobby_coord
    # if door == 1:
    #     door_coord = (1.40, door_coord[1])
    # elif door == 2:
    #     door_coord = (2.17, door_coord[1])
    # path_main = go_to_doors(goal=door_coord, range_dist=4)
    # outwps = get_waypoints(path_main)
    # print("out waypoints: " + str(outwps))
    # for x in outwps:
    #     print(x)
    #     # time.sleep(2)
    #     move_turn(x)
    #     # time.sleep(1)
    #     move_straight(x)
    #     # time.sleep(1)
    '''
    odata_delta = return_odata_origin_delta()
    door_coord = (
        door_coord[0] - odata_delta[0] * 0.02,
        door_coord[1] - odata_delta[1] * 0.02
    )
    move_turn(door_coord)
    move_straight(door_coord)
    '''

    print("---------------going to bucket!---------------")
    while(move_to_bucket(threshold=0.03) is None):
        print("---------------no bucket found :(---------------")
        print("---------------moving forward---------------")
        time_straight(0.15, 2)

    print("---------------launching servo!---------------")
    launch_servo()

    # Go back to explore the maze
    for _ in range(15):
        path_main = a_star_scan()

        outwps = get_waypoints(path_main)
        print("out waypoints: " + str(outwps))
        time_start = time.time()
        for x in outwps:
            print(x)
            # time.sleep(2)
            # will reset once every 20 seconds unless exit is seen: if exit seen, will move directly to exit and skip the resets.
            # once exit is seen, don't reset anymore (exitbreak will never equal 1) until quit is called
            move_turn(x, end_yaw_range=0.13, PID_angular=(2,0,4))
            # time.sleep(1)
            move_straight(x)
            # time.sleep(1)

            rclpy.spin_once(mapcheck)
    
    plt.close()


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # occupy.destroy_node()
    rclpy.shutdown()
    # cv.destroyAllWindows()


if __name__ == "__main__":
    main()
