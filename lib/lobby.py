import numpy as np
import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from .pid_tf2 import stop_kill

class DoorMover(Node):
    def __init__(self):
        super().__init__("door_mover")
        self.subscription = self.create_subscription(
            LaserScan, "scan", self.listener_callback, qos_profile_sensor_data
        )
        self.cmdvelpub = self.create_publisher(Twist, "cmd_vel", 10)
        self.angle_increment = None
        self.lidar_ranges: np.ndarray = None

    def listener_callback(self, msg: LaserScan):
        self.lidar_ranges = msg.ranges
        self.angle_increment = msg.angle_increment

    def move(self, door, speed: float = 0.1):
        rclpy.spin_once(self)
        if door == 1:
            index_high = int(np.deg2rad(270-25)/self.angle_increment)
            index_low = int(np.deg2rad(270+25)/self.angle_increment)
        elif door == 2:
            index_high = int(np.deg2rad(90+25)/self.angle_increment)
            index_low = int(np.deg2rad(90-25)/self.angle_increment)
        else:
            self.get_logger().error("door invalid!")
            return None
        twist = Twist()
        twist.linear.x = -speed
        key_dist_high = self.lidar_ranges[index_high]
        if np.isnan(key_dist_high):
            key_dist_high = self.lidar_ranges[index_high-1]
        key_dist_low = self.lidar_ranges[index_low]
        if np.isnan(key_dist_low):
            key_dist_low = self.lidar_ranges[index_low-1]
        print((key_dist_high, key_dist_low))
        try:
            while key_dist_high < 0.6 or key_dist_low < 0.6:
                rclpy.spin_once(self)
                key_dist_high = self.lidar_ranges[index_high]
                if np.isnan(key_dist_high):
                    key_dist_high = self.lidar_ranges[index_high-1]
                key_dist_low = self.lidar_ranges[index_low]
                if np.isnan(key_dist_low):
                    key_dist_low = self.lidar_ranges[index_low-1]
                print((key_dist_high, key_dist_low))
                self.cmdvelpub.publish(twist)
        except KeyboardInterrupt:
            self.get_logger().info("KeyboardInterrupt!")
        stop_kill(self)
        return 1

def door_mover(door, speed: float = 0.08):
    doormover = DoorMover()
    doormover.move(door, speed)
