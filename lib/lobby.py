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

    def move(self, speed: float = 0.1):
        rclpy.spin_once(self)
        target = 0.57
        front_dist = self.lidar_ranges[len(self.lidar_ranges) // 2]
        print(front_dist)
        twist = Twist()
        try:
            while True:
                rclpy.spin_once(self)
                front_dist = self.lidar_ranges[len(self.lidar_ranges) // 2]
                print(front_dist)
                if np.isnan(front_dist):
                    continue
                if abs(front_dist - target) < 0.03:
                    break
                if front_dist - target > 0:
                    twist.linear.x = -speed
                else:
                    twist.linear.x = speed
                self.cmdvelpub.publish(twist)
        except KeyboardInterrupt:
            self.get_logger().info("KeyboardInterrupt!")
        stop_kill(self)
        return 1

def door_mover(speed: float = 0.08):
    doormover = DoorMover()
    doormover.move(speed)
