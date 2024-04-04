import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist
from .tf2_quat_utils import euler_from_quaternion
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from simple_pid import PID
from typing import Tuple


class WPMover(Node):
    def __init__(
        self,
        target: tuple,
        end_distance_range: float = 0.1,
        PID_angular: tuple = (0.5, 0, 1),
        PID_linear: tuple = (0.3, 0, 1),
        angular_speed_limit: float = 1,  # old 2.84
        linear_speed_limit: float = 0.1,  # old 0.22
    ):
        super().__init__("wpmover")
        self.subscription = self.create_subscription(
            TFMessage, "tf", self.listener_callback, qos_profile_sensor_data
        )
        self.target_x = target[0]
        self.target_y = target[1]
        self.end_distance_range = end_distance_range
        self.tfBuffer = tf2_ros.Buffer(cache_time = rclpy.duration.Duration(seconds=0.1))
        self.tfListener = tf2_ros.TransformListener(
            self.tfBuffer, self, qos=qos_profile_sensor_data
        )
        self.pid_angular = PID(
            PID_angular[0],
            PID_angular[1],
            PID_angular[2],
            output_limits=(-angular_speed_limit, angular_speed_limit),
        )
        self.pid_linear = PID(
            PID_linear[0],
            PID_linear[1],
            PID_linear[2],
            setpoint=0,
            output_limits=(-linear_speed_limit, 0),
        )
        self.cmdvelpub = self.create_publisher(Twist, "cmd_vel", 10)

    def listener_callback(self, _):
        try:
            trans = self.tfBuffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info("No transformation found")
            return
        cur_pos = trans.transform.translation
        cur_rot = trans.transform.rotation
        # self.get_logger().info(str(cur_pos.x))
        # self.get_logger().info('Trans: %f, %f' % (cur_pos.x, cur_pos.y))
        # convert quaternion to Euler angles
        _, _, yaw = euler_from_quaternion(cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)
        # yaw = yaw - math.pi if yaw > 0 else yaw + math.pi
        # self.get_logger().info('Rot-Yaw: R: %f D: %f' % (yaw, np.degrees(yaw)))
        dx = cur_pos.x - self.target_x
        dy = cur_pos.y - self.target_y
        linear_dist = math.sqrt(dx**2 + dy**2)
        linear_speed = self.pid_linear(linear_dist)
        self.pid_angular.setpoint = math.atan(dy / dx)
        angular_speed = self.pid_angular(yaw)
        twist = Twist()
        if linear_dist < self.end_distance_range:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            now = time.time()
            while time.time() - now < 0.5:
                time.sleep(0.1)
                self.cmdvelpub.publish(twist)
            self.get_logger().info("Done")
            self.subscription.destroy()
            self.destroy_node()
            raise SystemExit
        twist.linear.x = float((linear_speed))
        twist.angular.z = float(angular_speed)
        self.cmdvelpub.publish(twist)
        # self.get_logger().info('LinSpd: '+str(linear_speed)+' LinDst: '+str(linear_dist)+' AngSpd: '+str(angular_speed)+' Yaw: '+str(yaw))


class WPTurner(Node):
    def __init__(
        self,
        target: tuple,
        end_yaw_range: float = 0.05,
        PID_angular: tuple = (1, 0, 2),
        angular_speed_limit: float = 1,  # old 2.84
    ):
        super().__init__("wpturner")
        self.subscription = self.create_subscription(
            TFMessage, "tf", self.listener_callback, qos_profile_sensor_data
        )
        self.target_x = target[0]
        self.target_y = target[1]
        self.end_yaw_range = end_yaw_range
        self.tfBuffer = tf2_ros.Buffer(cache_time = rclpy.duration.Duration(seconds=0.1))
        self.tfListener = tf2_ros.TransformListener(
            self.tfBuffer, self, qos=qos_profile_sensor_data
        )
        self.pid_angular = PID(
            PID_angular[0],
            PID_angular[1],
            PID_angular[2],
            output_limits=(-angular_speed_limit, angular_speed_limit),
        )
        self.cmdvelpub = self.create_publisher(Twist, "cmd_vel", 10)

    def listener_callback(self, _):
        try:
            trans = self.tfBuffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info("No transformation found")
            return
        cur_pos = trans.transform.translation
        cur_rot = trans.transform.rotation
        _, _, yaw = euler_from_quaternion(cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)
        # yaw = yaw - math.pi if yaw > 0 else yaw + math.pi
        # self.get_logger().info('Rot-Yaw: R: %f D: %f' % (yaw, np.degrees(yaw)))
        dx = cur_pos.x - self.target_x
        dy = cur_pos.y - self.target_y
        self.pid_angular.setpoint = math.atan(dy / dx)
        angular_speed = self.pid_angular(yaw)
        twist = Twist()
        twist.linear.x = 0.0
        if abs(math.atan(dy / dx) - yaw) < self.end_yaw_range:
            twist.angular.z = 0.0
            now = time.time()
            while time.time() - now < 0.5:
                time.sleep(0.1)
                self.cmdvelpub.publish(twist)
            self.get_logger().info("Done")
            self.subscription.destroy()
            self.destroy_node()
            raise SystemExit
        twist.angular.z = float(angular_speed)
        self.cmdvelpub.publish(twist)


def move_straight(
    target: Tuple[float, float],
    end_distance_range: float = 0.1,
    PID_angular: Tuple[float, float, float] = (0.5, 0, 1),
    PID_linear: Tuple[float, float, float] = (0.3, 0, 1),
    angular_speed_limit: float = 1,  # old 2.84
    linear_speed_limit: float = 0.1,  # old 0.22
):
    """Move Straight to RViz Waypoint

    Args:
        target (Tuple[float, float]): (x,y) in RViz coordinates
        end_distance_range (float, optional): When target is x meters away, stop function. Defaults to 0.1.
        PID_angular (Tuple[float, float, float], optional): (kP, kI, kD). Defaults to (0.5, 0, 1).
        PID_linear (Tuple[float, float, float], optional): (kP, kI, kD). Defaults to (0.3, 0, 1).
        angular_speed_limit (float, optional): Angular Velocity Limit. Defaults to 1.
        linear_speed_limit (float, optional): Linear Velocity Limit. Defaults to 0.1.
    """
    wpmover = WPMover(
        target,
        end_distance_range,
        PID_angular,
        PID_linear,
        angular_speed_limit,
        linear_speed_limit,
    )
    try:
        rclpy.spin(wpmover)
    except Exception and KeyboardInterrupt:
        print("kb interrupt")
    except SystemExit:
        print("sys exit done")


def move_turn(
    target: Tuple[float, float],
    end_yaw_range: float = 0.05,
    PID_angular: Tuple[float, float, float] = (1, 0, 2),
    angular_speed_limit: float = 1,  # old 2.84
):
    """Turn to face RViz Waypoint

    Args:
        target (Tuple[float, float]): (x,y) in RViz coordinates
        end_yaw_range (float, optional): When robot is within x rad, stop function. Defaults to 0.05.
        PID_angular (Tuple[float, float, float], optional): (kP, kI, kD). Defaults to (1, 0, 2).
        angular_speed_limit (float, optional): Angular Velocity Limit. Defaults to 1.
    """
    wpturner = WPTurner(target, end_yaw_range, PID_angular, angular_speed_limit)
    try:
        rclpy.spin(wpturner)
    except Exception and KeyboardInterrupt:
        print("kb interrupt")
    except SystemExit:
        print("sys exit done")


def main(args=None):
    rclpy.init(args=args)

    wpturner = WPTurner((-1.1, 0.14))
    try:
        rclpy.spin(wpturner)
    except Exception and KeyboardInterrupt:
        pass
    except SystemExit:
        print("sys exit done")

    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    now = time.time()
    while time.time() - now < 0.5:
        wpturner.cmdvelpub.publish(twist)
        time.sleep(0.1)
    wpturner.get_logger().info("Done")
    wpturner.subscription.destroy()
    wpturner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
