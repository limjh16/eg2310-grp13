import time

import numpy as np
from simple_pid import PID
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from .tf2_quat_utils import (
    quaternion_multiply,
    quaternion_from_euler,
    euler_from_quaternion,
)
from .pid_tf2 import stop_kill


class TimeStraight(Node):
    def __init__(self):
        super().__init__("timestraight")
        self.cmdvelpub = self.create_publisher(Twist, "cmd_vel", 10)

    def move(self, speed: float, timing):
        twist = Twist()
        twist.linear.x = -speed
        now = time.time()
        try:
            while time.time() - now < timing:
                self.cmdvelpub.publish(twist)
                time.sleep(0.05)
        except KeyboardInterrupt:
            self.get_logger().info("KeyboardInterrupt!")
        stop_kill(self)


def time_straight(speed: float, timing):
    timestraight = TimeStraight()
    timestraight.move(speed, timing)


class OdomTurn(Node):
    def __init__(
        self,
        target_yaw: np.degrees,
        end_yaw_range: float = 0.05,
        PID_angular: tuple = (1, 0, 2),
        angular_speed_limit: float = 1,
    ):
        super().__init__("odomturn")
        self.cmdvelpub = self.create_publisher(Twist, "cmd_vel", 10)
        self.subscription = self.create_subscription(
            Odometry, "odom", self.listener_callback, qos_profile_sensor_data
        )
        self.end_yaw_range = end_yaw_range
        self.pid_angular = PID(
            PID_angular[0],
            PID_angular[1],
            PID_angular[2],
            setpoint=0,
            output_limits=(-angular_speed_limit, angular_speed_limit),
        )
        self.q_r = quaternion_from_euler(0, 0, -np.deg2rad(target_yaw))
        # print((self.q_r[0], self.q_r[1], self.q_r[2], self.q_r[3]))
        # print((self.q_r[2], np.deg2rad(target_yaw)))

    def listener_callback(self, data):
        self.quat = data.pose.pose.orientation
        # print((self.quat.x, self.quat.y, self.quat.z, self.quat.w))

    def turn(self):
        rclpy.spin_once(self)
        self.q_2 = quaternion_multiply(
            (self.q_r[3], self.q_r[0], self.q_r[1], self.q_r[2]), (self.quat.w, self.quat.x, self.quat.y, self.quat.z)
        )
        # print("target: "+ str((self.q_2[2], self.q_2[3])))
        rot_tf_yaw = 100
        try:
            while abs(rot_tf_yaw) > self.end_yaw_range:
                rclpy.spin_once(self)
                rot_tf = quaternion_multiply(
                    self.q_2, (-self.quat.w, self.quat.x, self.quat.y, self.quat.z)
                )
                _, _, rot_tf_yaw = euler_from_quaternion(rot_tf[1], rot_tf[2], rot_tf[3], rot_tf[0])
                # print(np.rad2deg(rot_tf_yaw))
                # time.sleep(10)
                angular_speed = self.pid_angular(-rot_tf_yaw)
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = float(angular_speed)
                self.cmdvelpub.publish(twist)
        except KeyboardInterrupt:
            self.get_logger().info("KeyboardInterrupt!")
        stop_kill(self)


def odom_turn(
    target_yaw: np.degrees,
    end_yaw_range: float = 0.01,
    PID_angular: tuple = (4, 0, 0.4),
    angular_speed_limit: float = 1.2,
):
    odomturn = OdomTurn(
        target_yaw=target_yaw,
        end_yaw_range=end_yaw_range,
        PID_angular=PID_angular,
        angular_speed_limit=angular_speed_limit,
    )
    odomturn.turn()


def main(args=None):
    rclpy.init(args=args)
    odom_turn(45)
    # time_straight(0.15, 3)
    # odom_turn(9)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
