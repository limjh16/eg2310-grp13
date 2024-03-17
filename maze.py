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

UNKNOWN = 1
UNOCCUPIED = 2
OCCUPIED = 3

# constants
occ_bins = [-1, 0, 50, 100]


def dilate123(src, size=1, shape=cv.MORPH_RECT):
    """Dilation for odata array (after binning and reshape)

    Args:
        src (ndarray): input odata array
        size (integer): number of iterations of dilation,
                            this size / resolution is the meters that the image is dilated.
        shape (integer): cv2 MorphShapes

    Returns:
        ndarray: output of dilated array
    """
    array_edited = np.copy(src)
    array_edited[array_edited <= 2] = 0
    array_edited //= 3
    array_dilated = cv.dilate(
        array_edited,
        cv.getStructuringElement(shape, (2 * size + 1, 2 * size + 1)),
    )
    array_dilated *= 3
    return np.maximum(src, array_dilated)



# use this function to convert raw odata coordinates to reference the defined origin
def reference_to_origin(raw_odata_coord):
    return (
        raw_odata_coord[0] - odata_origin[0],
        raw_odata_coord[1] - odata_origin[1]
    )
#use this function to dereference odata coordinates from the defined origin
def dereference_to_origin(ref_odata_coord):
    return (
        ref_odata_coord[0] + odata_origin[0],
        ref_odata_coord[1] + odata_origin[1]
    )


class Occupy(Node):
    def __init__(self):
        super().__init__("occupy")
        
        self.subscription = self.create_subscription(
            OccupancyGrid, "map", self.occ_callback, qos_profile_sensor_data
        )
        self.subscription  # prevent unused variable warning
        # occdata = np.array([])
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

    def occ_callback(self, msg):

        # use this function to convert from rviz coordinates to odata coordinates
        def convert_to_rviz(odata_coord):
            rviz = (
            odata_coord[0] * msg.info.resolution + msg.info.origin.position.x,
            odata_coord[1] * msg.info.resolution + msg.info.origin.position.y,
            )
            return rviz
        # use this function to convert from odata coordinates to rviz coordinates
        def convert_to_odata(rviz):
            odata_coord = (

                round(float(rviz[0] - msg.info.origin.position.x) / msg.info.resolution), 
                round(float(rviz[1] - msg.info.origin.position.y) / msg.info.resolution) 
            )
            return odata_coord

        # the process for obtaining odata (the occupancy grid)
        occdata = np.array(msg.data)
        _, _, binnum = scipy.stats.binned_statistic(
            occdata, np.nan, statistic="count", bins=occ_bins
        )
        odata = np.uint8(binnum.reshape(msg.info.height, msg.info.width))
        odata = dilate123(odata, size=4)  # 4 here means 20cm
        (odata_y, odata_x) = odata.shape
        self.get_logger().info("maze dilated")

        # obtain the rviz coordinates of the turtlebot
        # usually requires the Occupy() node to spin more than once
        try:
            trans = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info('No transformation found')
            return
        cur_pos = trans.transform.translation 
        self.get_logger().info('Trans: %f, %f' % (cur_pos.x, cur_pos.y))

        # confirm our defined origin coordinates in odata, obtained from FirstOccupancy()
        self.get_logger().info('odata_origin: ' + str(odata_origin))

        # rviz coordinates of turtlebot's current position
        temp_cur_pos = (cur_pos.x, cur_pos.y) 
        # convert turtlebot's rviz coordinates to odata coordintates to be reflected in the occupancy grid
        curr_pos_odata = convert_to_odata(temp_cur_pos) 
        self.get_logger().info('curr_pos_odata: ' + str(curr_pos_odata))

        # current position of turtlebot in odata, with reference to origin 
        curr_pos = reference_to_origin(curr_pos_odata)
        # add a cost variable (x, y, cost)
        curr_pos = (curr_pos[0], curr_pos[1], 0)
        # curr_pos = ((curr_pos_odata[0] - odata_origin[0]), (curr_pos_odata[1] - odata_origin[1]), 0)
        self.get_logger().info('Current' + str(curr_pos))

        goal = (0, 0)
        # iterate through odata to find the goal (highest y coordinate)
        maxnum = 0
        for row in range(odata_y):
            for col in range(odata_x): 
                if odata[row][col] == 2 and row + col > maxnum:
                    minnum = row + col
                    goal = (col, row)

        
        # find goal_pos, the goal relative to the origin coordinates
        goal_pos = reference_to_origin(goal)
        # update goal_pos with the cost from the start to the goal
        goal_pos = (goal_pos[0], goal_pos[1], cost_to_goal(curr_pos, goal_pos))
        # goal_pos = ((goal[0] - odata_origin[0]), (goal[1] - odata_origin[1]), 0)
        self.get_logger().info('Goal' + str(goal_pos))


        # EVERYTHING SHOULD NOW BE CONVERTED TO ODATA COORDINATES

        start_pos = curr_pos
        

        # mark the curr_pos on the 2D array (need to print absolute coordinates, not with reference to defined origin)
        odata[int(curr_pos[1] + odata_origin[1]), int(curr_pos[0] + odata_origin[0])] = 4

        self.get_logger().info("finding path...")

        # bool variables to check if the current position is within range of goal
        range_dist = 12
        is_within_goal_y = (curr_pos[1] < goal_pos[1] + range_dist and curr_pos[1] > goal_pos[1] - range_dist)
        is_within_goal_x = (curr_pos[0] < goal_pos[0] + range_dist and curr_pos[0] > goal_pos[0] - range_dist)

        loop_count = 0
        # exit the loop when current position is in range of the goal
        while not (is_within_goal_x and is_within_goal_y):
            new_pos = a_star(curr_pos, start_pos, goal_pos, odata) # finds next point to travel to
            self.get_logger().info(str(new_pos))
            
            # need to print absolute coordinates, not with reference to defined origin
            map_pos = dereference_to_origin(new_pos)
            # loop to visibly observe curr_pos in the occupancy grid
            for i in range(1):
                odata[int(map_pos[1]), int(map_pos[0])] = 4
            
            # create image from 2D array using PIL
            if loop_count % 15 == 0: # print new map only every 3 movements
                img = Image.fromarray(odata)
                # show the image using grayscale map
                plt.imshow(img, cmap="gray", origin="lower")
                plt.draw_all()
                # pause to make sure the plot gets created
                plt.pause(0.00000000001)
            self.get_logger().info("finding next")
            

            # this is just to make sure the loop exits and doesn't loop again once path found
            # if new_pos == (0, 0, 0):
            #     break
            
            # update curr_pos 
            curr_pos = new_pos
            # update bool status of whether or not curr_pos is in range of goal
            is_within_goal_y = (curr_pos[1] < goal_pos[1] + range_dist and curr_pos[1] > goal_pos[1] - range_dist)
            is_within_goal_x = (curr_pos[0] < goal_pos[0] + range_dist and curr_pos[0] > goal_pos[0] - range_dist)
            loop_count += 1

        self.get_logger().info("path found!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        # np.savetxt('/home/jh/ogm.txt',odata,fmt='%i')

        # create image from 2D array using PIL
        # img = Image.fromarray(odata)
        # show the image using grayscale map
        plt.imshow(odata, cmap="gray", origin="lower")
        # plt.draw_all()
        # pause to make sure the plot gets created
        plt.pause(5.00000000001)
        
        # exit this node once path found
        raise SystemExit
        



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
        _, _, binnum = scipy.stats.binned_statistic(
            occdata, np.nan, statistic="count", bins=occ_bins
        )
        odata = np.uint8(binnum.reshape(msg.info.height, msg.info.width))

        (odata_y, odata_x) = odata.shape
        minnum = 10000000
        current_min = (0, 0)
        for j in range(odata_y):
            for i in range(odata_x):
                if odata[j][i] == 2 and i + j < minnum:
                    minnum = i + j
                    current_min = (i, j)

        global odata_origin
        odata_origin = current_min
        
        current_min = (
            current_min[0] * msg.info.resolution + msg.info.origin.position.x,
            current_min[1] * msg.info.resolution + msg.info.origin.position.y,
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


# this takes in a matrix and finds the coordinates of the point with the highest y coordinate
def find_greatest_y(m):
    for row in range(m, -1, -1):
        for col in range(m[row]): 
            if m[row][col] == 2:
                return (col, row)
    return (0, 0)    
        

# calculates euclidean distance from current position to the goal
def cost_to_goal(pos, goal_pos):
    dist_x = abs(goal_pos[0] - pos[0])
    dist_y = abs(goal_pos[1] - pos[1])
    return math.sqrt(dist_x**2 + dist_y**2)

# curr_pos = (x-coordinate, y-coordinate, cost from start point)
def a_star(curr_pos, start_pos, goal_pos, gridmap):
    # possible directions
    """
    Get all possible 8-connectivity movements. Equivalent to get_movements_in_radius(1).
    :return: list of movements with cost [(dx, dy, movement_cost)]
    """
    s2 = math.sqrt(2)
    directions = [(1, 0, 1.0), #right
            (0, 1, 1.0), #forward
            (-1, 0, 1.0), #left
            (0, -1, 1.0), #back
            (1, 1, s2), 
            (-1, 1, s2),
            (-1, -1, s2),
            (1, -1, s2)]
    next_pos = curr_pos
    min_cost = 0
    for direction in directions: 
        new_pos = (int(curr_pos[0] + direction[0]), int(curr_pos[1] + direction[1]), int(curr_pos[2] + direction[2]))
        #for i in range(0, 2, 1):
         #   new_pos[i] += direction[i]
        
        temp_pos = dereference_to_origin(new_pos)
        if gridmap[temp_pos[1], temp_pos[0]] == 2:
            calc_cost = cost_to_goal(new_pos, goal_pos) + new_pos[2]
        
        # if min_cost == 0:
        #     min_cost = calc_cost
            if not min_cost or calc_cost < min_cost:
                min_cost = calc_cost
                next_pos = new_pos
        else: 
            continue
    return next_pos
            
        
def main(args=None):
    rclpy.init(args=args)

    occupy = Occupy()
    firstoccupy = FirstOccupy()
    rclpy.spin_once(firstoccupy)
    firstoccupy.destroy_node()

    # create matplotlib figure
    plt.ion()
    plt.show()

    try:
        rclpy.spin(occupy)
    except SystemExit:                 # <--- process the exception 
        rclpy.logging.get_logger("Quitting").info('Done')

  
    

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    occupy.destroy_node()
    rclpy.shutdown()
    # cv.destroyAllWindows()


if __name__ == "__main__":
    main()