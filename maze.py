from csv import excel
import time
import warnings
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
from .lib.occupy_nodes import delta_to_origin, first_scan, a_star_scan, return_odata_origin, a_star_search, go_to_doors, finish_maze, convert_to_odata, return_odata_origin_delta
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
ipaddr = "192.168.15.87"
lobby_coord = (1.8,2.85) # supposed to be 2.8

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
        # cdata[cdata == 100] = -1
        cdata[cdata >= 0] = 1
        cdata[cdata == -1] = 0
        no_unknown_indexes = np.nonzero(cdata)
        odata_lobby_coord = delta_to_origin(convert_to_odata(lobby_coord, msg.info.origin.position.x, msg.info.origin.position.y))
        # y_dist = np.max(no_unknown_indexes[0]) - return_odata_origin()[1]
        # print("distance_to_furthest:  "+str(y_dist))
        # if (y_dist > (3 / msg.info.resolution)):
        if (odata_lobby_coord[0]-3 in no_unknown_indexes[1] and odata_lobby_coord[0]+3 in no_unknown_indexes[1] and odata_lobby_coord[1]+5 in no_unknown_indexes[0]):
            global quit
            quit = 1
            print("!!!!!!!!!!!!!!!!!!!!!!!!quit!!!!!!!!!!!!!!!!!!")
        

def main(args=None):
    rclpy.init(args=args)

    # door_mover()
    # odom_turn(-90)
    # time_straight(0.15, 3)
    # while(move_to_bucket(threshold=0.02) is None):
    #     print("---------------no bucket found :(---------------")
    #     print("---------------moving forward---------------")
    #     time_straight(0.15, 2)
    # time.sleep(1000)
    
    # time_straight(0.1,1)
    time_straight(-0.1,10)
    first_scan()

    # create matplotlib figure
    plt.ion()
    plt.show()
    mapcheck = mapCheck()


    for _ in range(15):
        path_main = a_star_scan()
        # time.sleep(1000)
        outwps = get_waypoints(path_main)
        print("out waypoints: " + str(outwps))
        time_start = time.time()
        for x in outwps:
            # print(x)
            if quit:
                break
            if time.time()-time_start > 20:
                break
            # will reset once every 20 seconds unless exit is seen: if exit seen, will move directly to exit and skip the resets.
            # once exit is seen, don't reset anymore (exitbreak will never equal 1) until quit is called
            move_turn(x)
            move_straight(x)

            rclpy.spin_once(mapcheck)
        if quit:
            print("quit, can see elevator")
            break

    print("---------------going to lobby!---------------")
    
    # can actually update path every 10 seconds (similar to above) by breaking the loop and calling go_to_doors() again

    for _ in range(2): # run twice to confirm its at the lobby coordinates, as go_to_doors() allows for some margin of error
        path_main = go_to_doors(goal=lobby_coord)
        outwps = get_waypoints(path_main)
        print("out waypoints: " + str(outwps))
        for x in outwps:
            # print(x)
            move_turn(x)
            move_straight(x)

    # print("---------------http call!---------------")
    door = 0
    try:
        door = open_door(ipaddr)
    except Exception as e:
        print(e)
    while door == 0:
        print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!! request failed !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        time.sleep(1)
        try:
            door = open_door(ipaddr)
        except Exception as e:
            print(e)
    # door = 1

    print("!!!!!!!!!!!!!!!---------------waiting 5s, pls open door!---------------!!!!!!!!!!!!!!!!!!!")
    time.sleep(5)
    

    print("---------------going to door "+str(door)+"!---------------")
    # face front
    move_turn((return_cur_pos().x, return_cur_pos().y+5))
    door_mover()
    if door == 1:
        turndeg = -5
    elif door == 2:
        turndeg = 5
    move_turn((return_cur_pos().x, return_cur_pos().y+5)) # to refresh cur_pos after door_mover
    move_turn((return_cur_pos().x+turndeg, return_cur_pos().y))
    time_straight(0.15, 4)

    '''
    door_coord = lobby_coord
    if door == 1:
        door_coord = (door_coord[0]-0.4, door_coord[1])
        turndeg = -90
    elif door == 2:
        door_coord = (door_coord[0]+0.4, door_coord[1])
        turndeg = 90
    warnings.filterwarnings("error")
    try:
        path_main = go_to_doors(goal=door_coord)
        outwps = get_waypoints(path_main)
        print("out waypoints: " + str(outwps))
        for x in outwps:
            # print(x)
            move_turn(x)
            move_straight(x)
    except (RuntimeWarning, TypeError):
        # odom_turn(turndeg)
        door_mover()
        move_turn((return_cur_pos().x, return_cur_pos().y+5)) # to refresh cur_pos after door_mover
        move_turn((return_cur_pos().x+turndeg, return_cur_pos().y))
        time_straight(0.15, 4)
    '''    
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
        path_main = finish_maze()

        outwps = get_waypoints(path_main)
        print("out waypoints: " + str(outwps))
        time_start = time.time()
        for x in outwps:
            # print(x)
            if time.time()-time_start > 20:
                break
            # will reset once every 20 seconds.
            move_turn(x)
            move_straight(x)

            # rclpy.spin_once(mapcheck)

    plt.close()


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # occupy.destroy_node()
    rclpy.shutdown()
    # cv.destroyAllWindows()


if __name__ == "__main__":
    main()
