import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
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
from .maze_manipulation import dilate123, inflate
# from .lib.pid_tf2 import move_straight, move_turn
from array import array as Array


UNKNOWN = 1
UNOCCUPIED = 2
OCCUPIED = 3

# constants
occ_bins = [-1, 0, 60, 100]
path_main = []
dilate_size = 2

# use this function to convert raw odata coordinates to reference the defined origin
def reference_to_origin(raw_odata_coord):
    """ Offsetting of raw data of coordinates with respect to the defined origin on odata
    Args:
        raw_odata_coord (tuple): the (x, y) value in odata
    Returns: 
        tuple: (x, y) value in odata eith reference to the origin
    """
    odata_origin = return_odata_origin()
    return (
        raw_odata_coord[0] - odata_origin[0],
        raw_odata_coord[1] - odata_origin[1]
    )
#use this function to dereference odata coordinates from the defined origin
def dereference_to_origin(ref_odata_coord):
    """ resetting of coordinates with resepect to odata
    Args: 
        ref_odata_coord (tuple): the (x, y) value in odata with respect to the origin
    Returns: 
        tuple: (x, y) raw value in odata"""
    odata_origin = return_odata_origin()
    return (
        ref_odata_coord[0] + odata_origin[0],
        ref_odata_coord[1] + odata_origin[1]
    )

def delta_to_origin(ref_odata_coord):
    return (
        ref_odata_coord[0] + odata_origin_delta[0],
        ref_odata_coord[1] + odata_origin_delta[1]
    )

class CostmapSub(Node):
    def __init__(self):
        super().__init__('costmap_sub')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            qos_profile_sensor_data)

    def costmap_callback(self, msg):
        global costmap
        costmap = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        print("yes")

# use this function to convert from rviz coordinates to odata coordinates
def convert_to_rviz(odata_coord):
    """ Scaling of odata (x, y) coordinates to rviz (x, y) coordinates in meters
    Args: 
        odata_coord (tuple): (x, y) coordinates in odata
    Returns:
        tuple: (x, y) coordinates in rviz, in meters"""
    rviz = (
    odata_coord[0] * map_resolution + origin_pos_x,
    odata_coord[1] * map_resolution + origin_pos_y,
    )
    return rviz
# use this function to convert from odata coordinates to rviz coordinates
def convert_to_odata(rviz, origin_x, origin_y):
    """ Scaling of rviz (x, y) coordinates in meters to odata (x, y) coordinates
    Args: 
        rviz (tuple): (x, y) coordinates in rviz
    Returns: 
        tuple: (x, y) coordinates in odata"""
    odata_coord = (

        round(float(rviz[0] - origin_x) / map_resolution), 
        round(float(rviz[1] - origin_y) / map_resolution) 
    )
    return odata_coord

# this is used to check if a point is next to an unoccupied area
def check_neighbours(row, col, checknum):
        directions = [
            (1, 0), #right
            (0, 1), #forward
            (-1, 0), #left
            (0, -1), #back
            (1, 1), 
            (-1, 1),
            (-1, -1),
            (1, -1)
            ]
        for direction in directions:
            new_col = col + direction[0]
            new_row = row + direction[1]
            is_in_map = (new_row < odata_y and new_col < odata_x and new_row > 0 and new_col > 0)
            is_checknum = is_in_map and (odata[new_row][new_col] == checknum)
            if is_checknum:
                return True
        return False


class Occupy(Node):
    def __init__(self):
        super().__init__("occupy")
        
        self.subscription = self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self.occ_callback, qos_profile_sensor_data
        )
        self.subscription  # prevent unused variable warning
        # occdata = np.array([])
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

    def occ_callback(self, msg):
        global map_resolution
        global origin_pos_x
        global origin_pos_y
        
        map_resolution = msg.info.resolution
        origin_pos_x = msg.info.origin.position.x
        origin_pos_y = msg.info.origin.position.y

        global costmap
        costmap = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        # the process for obtaining odata (the occupancy grid)
        occdata = np.array(msg.data)
        # _, _, binnum = scipy.stats.binned_statistic(
        #     occdata, np.nan, statistic="count", bins=occ_bins
        # )
        # make odata a global variable to be accessible by all functions
        binnum = occdata.copy()
        binnum[occdata == -1] = 1
        binnum[occdata >= 0] = 2
        binnum[occdata == 100] = 3
        global odata
        odata = np.uint8(binnum.reshape(msg.info.height, msg.info.width))

        # dilate_size = int((0.243/2)//msg.info.resolution)
        # odata = dilate123(odata, size=dilate_size)
        # Robot is 0.243m at its longest, divide by 2 to get radius, divide by resolution to change to odata coords, no need to +1 since walls aren't 5cm
        global odata_x, odata_y
        (odata_y, odata_x) = odata.shape
        # self.get_logger().info("maze dilated")

        # obtain the rviz coordinates of the turtlebot
        # usually requires the Occupy() node to spin more than once
        try:
            trans = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info('No transformation found')
            return
        cur_pos = trans.transform.translation 
        self.get_logger().info('Trans: %f, %f' % (cur_pos.x, cur_pos.y))

        # rviz coordinates of turtlebot's current position
        temp_cur_pos = (cur_pos.x, cur_pos.y) 
        # convert turtlebot's rviz coordinates to odata coordintates to be reflected in the occupancy grid
        curr_pos_odata = convert_to_odata(temp_cur_pos, origin_pos_x, origin_pos_y) 
        self.get_logger().info('curr_pos_odata: ' + str(curr_pos_odata))

        # curr_pos_odata is not yet referenced to our defined origin from FirstOccupy()

        # do this to find the next closest point in odata that is unoccupied
        is_in_wall = (odata[curr_pos_odata[1]][curr_pos_odata[0]] == 3)

        odata[int(curr_pos_odata[1]), int(curr_pos_odata[0])] = 4 # curr_pos

        if is_in_wall: 
            unoccupied_pts = np.transpose(np.nonzero(odata == 2))
            closest_dist = 999999
            closest_pt = return_odata_origin()
            for pts in unoccupied_pts: 
                pts = (pts[1], pts[0])
                calc_dist = cost_to_goal(curr_pos_odata, pts)
                if calc_dist < closest_dist:
                    closest_dist = calc_dist
                    closest_pt = pts

            curr_pos_odata = closest_pt
            self.get_logger().info('new curr_pos_odata: ' + str(curr_pos_odata))




        # current position of turtlebot in odata, with reference to origin 
        global curr_pos
        curr_pos = reference_to_origin(curr_pos_odata)
        # curr_pos = curr_pos_odata
        self.get_logger().info('Current' + str(curr_pos))

       
        # goal = (0, 0)
        # # iterate through odata to find the goal (highest y coordinate)
        # maxnum = 0
        # for row in range(odata_y):
        #     for col in range(odata_x): 
        #         is_unoccupied = (odata[row][col] == 2)
        #         is_next_to_unknown = (check_neighbours(row, col, 1))
        #         # if odata[row][col] == 2 and row + col > maxnum:
        #         if row + col > maxnum and is_unoccupied and is_next_to_unknown:
        #             maxnum = row + col
        #             goal = (col, row)
        # goal_pos = reference_to_origin(goal)


            
        # find goal_pos, the goal relative to the origin coordinates
       
        
        # raise SystemExit

        # EVERYTHING SHOULD NOW BE CONVERTED TO ODATA COORDINATES
        # exit this node once start and goal is found
        raise SystemExit
    
def get_goal(col_start, row_start): 
    # iterate through odata to find the goal (highest y coordinate)
    # maxnum = 0
    width = odata_x
    height = odata_y
    goal = (0, 0)
    row = row_start
    while goal == (0, 0) and row >= 0:
        # print("Scanning row " + str(row))
        
        # start iterating from the last found goal
        if row == row_start:
            while goal == (0, 0) and col_start >= 0:
                # print("scanning col " + str(col))
                is_unoccupied = (odata[row][col_start] == 2)
                # print("is unoccupied? " + str(is_unoccupied))
                is_next_to_unknown = (check_neighbours(row, col_start, 1))
                # print("is next to unknown? " + str(is_next_to_unknown))
                if is_unoccupied and is_next_to_unknown:
                    # print(str(col), str(row))
                    goal = (col_start, row)
                col_start -= 1
        # once row containing last found goal has been fully searched, search the rest of the rows
        else: 
            col = width - 1
            while goal == (0, 0) and col >= 0:
                # print("scanning col " + str(col))
                is_unoccupied = (odata[row][col] == 2)
                # print("is unoccupied? " + str(is_unoccupied))
                is_next_to_unknown = (check_neighbours(row, col, 1))
                # print("is next to unknown? " + str(is_next_to_unknown))
                if is_unoccupied and is_next_to_unknown:
                    # print(str(col), str(row))
                    goal = (col, row)
                col -= 1
        row -= 1

    return reference_to_origin(goal)
    # return goal


def get_path(start, goal, range_dist = dilate_size):
    # start_pos = curr_pos
    
    odata_origin = return_odata_origin()
    # mark the curr_pos and goal on the 2D array (need to print absolute coordinates, not with reference to defined origin)
    odata[int(curr_pos[1] + odata_origin[1]), int(curr_pos[0] + odata_origin[0])] = 4 # curr_pos
    odata[int(goal[1]) + odata_origin[1], int(goal[0] + odata_origin[0])] = 4
    # odata[int(curr_pos[1]), int(curr_pos[0])] = 4 # curr_pos
    # odata[int(goal[1]), int(goal[0])] = 4

    print("finding path...")

    # create image from 2D array using PIL
    # img = Image.fromarray(odata)
    # show the image using grayscale map
    plt.imshow(odata, cmap="gray", origin="lower")
    # plt.draw_all()
    # pause to make sure the plot gets created
    plt.pause(0.5)
    came_from, cost_so_far, final_pos = a_star_search(odata, start, goal, range_dist=range_dist)
    if final_pos == 0:
        print("No path found")
        return 0


    '''
    So the goal is found alr
    coordinates of goal is somewhere in came_from and cost_to_goal
    1. Find GOAL as a key in came_from
    2. Add GOAL into path[]
    3. Find the values to find previous coordinates
    4. For each of the previous coordinates, find the one with lowest cost
    5. repeat from step 2 to 4
    '''
    
    last_pos = final_pos
    # create a list to store all the coordinates of the path, in reverse sequence, starting from last_pos
    path = [last_pos]
    prev_pos = 0
    # loop to find all points in the path until the start_pos is reached, append the path list whenever the next point is found
    while prev_pos != (start[0], start[1]):
        prev_pos = came_from[last_pos]
        last_pos = prev_pos
        path.append(last_pos)
    # reverse the order of the list, such that the path starts from start_pos
    path.reverse()
    # declare an empty array to store the rviz coordinates of the path, to be used to move the robot
    path_rviz = []
    print(str(path))

    #loop for every point in path to convert to rviz coordinates to be stored in path_rviz
    for point in path:
        map_point = dereference_to_origin(point)
        odata[map_point[1]][map_point[0]] = 4
        point_rviz = convert_to_rviz(map_point)
        path_rviz.append((round(point_rviz[0], 4), round(point_rviz[1], 4)))

    print(str(path_rviz))
    # global path_main
    # path_main = path_rviz
    
    # create image from 2D array using PIL
    # img = Image.fromarray(odata)
    # show the image using grayscale map
    plt.imshow(odata, cmap="gray", origin="lower")
    # plt.draw_all()
    # pause to make sure the plot gets created
    plt.pause(0.5)
    return path_rviz
    
    # # exit this node once path found
    
    # raise SystemExit
    


class FirstOccupy(Node):
    def __init__(self):
        super().__init__("firstoccupy")
        self.subscription = self.create_subscription(
            OccupancyGrid, "map", self.occ_callback, qos_profile_sensor_data
        )
        self.subscription  # prevent unused variable warning
        # occdata = np.array([])
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

    def occ_callback(self, msg):
        occdata = np.array(msg.data)
        self.info = msg.info
        _, _, binnum = scipy.stats.binned_statistic(
            occdata, np.nan, statistic="count", bins=[-1, 0, 40 , 100]
        )
        self.odata = np.uint8(binnum.reshape(msg.info.height, msg.info.width))
    
    def find_origin(self):
        rclpy.spin_once(self)
        (odata_y, odata_x) = self.odata.shape
        minnum = 10000000
        current_min = (0, 0)
        while current_min == (0,0):
            rclpy.spin_once(self)
            for j in range(odata_y):
                for i in range(odata_x):
                    if self.odata[j][i] == 2 and i + j < minnum:
                        minnum = i + j
                        current_min = (i, j)
        print(self.odata[current_min[1]][current_min[0]])
        global odata_origin_delta
        odata_origin_delta = (
            current_min[0] + int(self.info.origin.position.x / self.info.resolution),
            current_min[1] + int(self.info.origin.position.y / self.info.resolution)
        )
        
        current_min = (
            current_min[0] * self.info.resolution + self.info.origin.position.x,
            current_min[1] * self.info.resolution + self.info.origin.position.y,
        )


        self.get_logger().info("New Origin: " + str(current_min))
        # self.get_logger().info(str(msg.info.resolution))
        # resolution = 0.05m per 1 array unit

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "origin"

        t.transform.translation.x = current_min[0]
        t.transform.translation.y = current_min[1]
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_static_broadcaster.sendTransform(t)


# calculates euclidean distance from current position to the goal
def cost_to_goal(pos, goal_pos):
    # dist_x = abs(goal_pos[0] - pos[0])
    # dist_y = abs(goal_pos[1] - pos[1])
    # return math.sqrt(dist_x**2 + dist_y**2)
    (x1, y1) = pos
    (x2, y2) = goal_pos
    return 1.41 if abs(x1 - x2) and abs(y1 - y2) else 1 # sqrt2

    
def in_bounds(id):
    temp_pos = dereference_to_origin(id)
    if temp_pos[1] > len(odata) or temp_pos[0] > len(odata[0]) or temp_pos[1] < 0 or temp_pos[0] < 0:
        return False
    return True

def passable(id):
    temp_pos = dereference_to_origin(id)
    if odata[temp_pos[1]][temp_pos[0]] != 2:
        return False
    return True



def neighbors(id, graph):
    (x, y) = id
    results = [
        (x + 1, y), # right
        (x, y - 1), # down
        (x - 1, y), # left
        (x, y + 1), # up
        (x + 1, y + 1), 
        (x + 1, y - 1), 
        (x - 1, y + 1), 
        (x - 1, y - 1), 
        ]
    if (x + y) % 2 == 0:
        results.reverse()  # aesthetics
    results = filter(in_bounds, results)
    results = filter(passable, results)
    return results
    

def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    # return 1.4 if abs(x1 - x2) and abs(y1 - y2) else 1 # sqrt2
    return round(math.sqrt(abs(x1 - x2)**2 + abs(y1 - y2)**2),2)

# I got this one from online somewhere
def a_star_search(graph, start, goal, range_dist = dilate_size):
    """ This function searches for the shortest path via the a star search algo
    Args: 
        graph (2D Array): A matrix representation of the map
        start (tuple): The start position of the robot
        goal (tuple): The target or goal position 
    Returns: 
        came_from (dict): For each point in the path, the coordinates provided is the coordinates prior to that point
        cost_so_far (dict): 
        final_pos (tuple): The coordinates of the final position of the robot once the path is complete"""
    print("Start: " + str(start))
    print("Goal: " + str(goal))
    frontier = PriorityQueue()
    frontier.put((0, start))
    came_from = {start: None}
    cost_so_far = {start: 0}
    turning_cost = 0.50
    final_pos = 0 # initialise final_pos variable, if 0 is returned then a clear path is not found

    while not frontier.empty():
        (_,current) = frontier.get()
        # print("Current: " + str(current))
        # path.append(current)

        # bool variables to check if the current position is within range of goal
        # range_dist = dilate_size # if we dilate by 2, this needs to be 2
        is_within_goal_y = (current[1] < goal[1] + range_dist and current[1] > goal[1] - range_dist)
        is_within_goal_x = (current[0] < goal[0] + range_dist and current[0] > goal[0] - range_dist)

        if is_within_goal_x and is_within_goal_y:
        # if current == goal:
            print("Goal Found!!!")
            final_pos = current
            print(final_pos)
            break

        # if current == goal:
        #     break
        
        for next in neighbors(current, graph):
            new_cost = cost_so_far[current] + cost_to_goal(current, next) + costmap[next[1]][next[0]]
            # priority += costmap[next[1]][next[0]]
            prev = came_from[current]
            if prev != None:
                # next_direction = (int(next[0] - current[0]), int(next[1] - current[1]))
                # current_direction = (int(current[0] - prev[0]), int(current[1] - prev[1]))
                next_direction = (next[0] - current[0], next[1] - current[1])
                current_direction = (current[0] - prev[0], current[1] - prev[1])
                if  current_direction != next_direction:
                    
                    new_cost += turning_cost
                    # print("cost added")
            # new_cost = priority
            new_cost = round(new_cost,2)
            if next not in cost_so_far or new_cost < round(cost_so_far[next],2):

                priority = new_cost + heuristic(goal, next)
                cost_so_far[next] = new_cost
                frontier.put((priority,next))
                came_from[next] = current
    print(cost_so_far)
    return came_from, cost_so_far, final_pos
    # return path





def first_scan(): 
    firstoccupy = FirstOccupy()

    # rclpy.spin_once(firstoccupy)
    firstoccupy.find_origin()
    firstoccupy.destroy_node()

def a_star_scan(): 
    
    occupy = Occupy()
    # costmapsub = CostmapSub()
    # rclpy.spin_once(costmapsub)
    try:
        rclpy.spin(occupy)
    except SystemExit:                 # <--- process the exception 
        rclpy.logging.get_logger("Quitting").info('Done')
        
    occupy.destroy_node()

    goal_pos = get_goal(odata_x - 1, odata_y - 1)
    print("Goal: " + str(goal_pos))

    path_main = 0
    while path_main == 0:
        path_main = get_path(curr_pos, goal_pos)
        goal_pos = get_goal(goal_pos[0], goal_pos[1]) # start finding new goal from the next row onwards, iterate from goal_pos[1] and below

    return path_main

def go_to_doors(goal=(1.8, 2.8), range_dist=dilate_size):
    print("Going to doors!!!")
    occupy = Occupy()
    # costmapsub = CostmapSub()
    # rclpy.spin_once(costmapsub)
    try:
        rclpy.spin(occupy)
    except SystemExit:                 # <--- process the exception 
        rclpy.logging.get_logger("Quitting").info('Done')
    
    occupy.destroy_node()
    # path_main = get_path(curr_pos, goal_pos)
     # x = 1.7, y = 2.9
    goal_rviz = goal
    print("goal_rviz: " + str(goal_rviz))
    goal_odata = delta_to_origin(convert_to_odata(goal_rviz, origin_pos_x, origin_pos_y))

    if goal_odata == curr_pos:
        return goal_odata

    is_in_unoccupied = (odata[goal_odata[1]][goal_odata[0]] == 2)

    odata[int(goal_odata[1]), int(goal_odata[0])] = 4 # curr_pos

    if not is_in_unoccupied: 
        unoccupied_pts = np.transpose(np.nonzero(odata == 2))
        closest_dist = 999999
        closest_pt = return_odata_origin()
        for pts in unoccupied_pts: 
            pts = (pts[1], pts[0])
            calc_dist = cost_to_goal(goal_odata, pts)
            if calc_dist < closest_dist:
                closest_dist = calc_dist
                closest_pt = pts

        goal_odata = closest_pt
        print('new goal_odata: ' + str(goal_odata))


    odata_origin = return_odata_origin()
    goal_odata = (
        goal_odata[0] - odata_origin[0],
        goal_odata[1] - odata_origin[1]
    )
    print("goal_odata" + str(goal_odata))
    #     print("going between doors")
    #     goal_pos =
    path_main = get_path(curr_pos, goal_odata, range_dist=range_dist)
    print(str(path_main))
    return path_main


def finish_maze(upper_bound=(1.8, 2.8)):
    print("Finishing maze!!!")
    occupy = Occupy()
    # costmapsub = CostmapSub()
    # rclpy.spin_once(costmapsub)
    try:
        rclpy.spin(occupy)
    except SystemExit:                 # <--- process the exception 
        rclpy.logging.get_logger("Quitting").info('Done')
    
    occupy.destroy_node()
    # path_main = get_path(curr_pos, goal_pos)
    upper_bound_rviz = upper_bound
    print("upper_bound_rviz: " + str(upper_bound_rviz))
    upper_bound_odata = delta_to_origin(convert_to_odata(upper_bound_rviz, origin_pos_x, origin_pos_y))
    odata_origin = return_odata_origin()
    upper_bound_odata = (
        upper_bound_odata[0] - odata_origin[0],
        upper_bound_odata[1] - odata_origin[1]
    )
    goal_pos = get_goal(upper_bound_odata[0] - 1, upper_bound_odata[1] - 1)
    print("Goal: " + str(goal_pos))
    path_main = 0
    while path_main == 0:
        path_main = get_path(curr_pos, goal_pos)

        # search for goal_pos is continually referenced to origin during each iteration
        goal_pos = get_goal(goal_pos[0], goal_pos[1]) # start finding new goal from the next row onwards, iterate from goal_pos[1] and below

    return path_main


def return_odata_origin():
    delta_map = convert_to_odata((-origin_pos_x, -origin_pos_y), origin_pos_x, origin_pos_y)
    odata_origin = (delta_map[0] + odata_origin_delta[0],
                    delta_map[1] + odata_origin_delta[1])
    return odata_origin

def return_odata_origin_delta():
    return odata_origin_delta 
