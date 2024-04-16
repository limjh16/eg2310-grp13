import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from simple_pid import PID
from typing import Tuple
from .tf2_quat_utils import euler_from_quaternion, quaternion_multiply, quaternion_from_euler


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
        # self.subscription = self.create_subscription(
        #     Odometry, "odom", self.listener_callback, qos_profile_sensor_data
        # )
        self.timer = self.create_timer(1/30, self.listener_callback)
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
            setpoint=0,
            output_limits=(-angular_speed_limit, angular_speed_limit),
        )
        self.pid_linear = PID(
            PID_linear[0],
            PID_linear[1],
            PID_linear[2],
            setpoint=0,
            output_limits=(-linear_speed_limit, -0.01),
        )
        self.cmdvelpub = self.create_publisher(Twist, "cmd_vel", 10)

    def listener_callback(self):
        try:
            trans = self.tfBuffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            # self.get_logger().info("No transformation found")
            return
        global cur_pos
        cur_pos, cur_rot, dx, dy = dxdy(trans,self)
        angular_speed, _ = angular(cur_pos, cur_rot, dx, dy, self)
        linear_dist = math.sqrt(dx**2 + dy**2)
        linear_speed = self.pid_linear(linear_dist)

        twist = Twist()
        if linear_dist < self.end_distance_range:
            stop_kill(self)
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
        # self.subscription = self.create_subscription(
        #     Odometry, "odom", self.listener_callback, qos_profile_sensor_data
        # )
        self.timer = self.create_timer(1/30, self.listener_callback)
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
            setpoint=0,
            output_limits=(-angular_speed_limit, angular_speed_limit),
        )
        self.cmdvelpub = self.create_publisher(Twist, "cmd_vel", 10)

    def listener_callback(self):
        try:
            trans = self.tfBuffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            # self.get_logger().info("No transformation found")
            return
        
        angular_speed, rot_tf_yaw = angular(*dxdy(trans,self),self)
        twist = Twist()
        twist.linear.x = 0.0
        if abs(rot_tf_yaw) < self.end_yaw_range:
            stop_kill(self)
            raise SystemExit
        twist.angular.z = float(angular_speed)
        self.cmdvelpub.publish(twist)


def move_straight(
    target: Tuple[float, float],
    end_distance_range: float = 0.05,
    PID_angular: Tuple[float, float, float] = (0.6, 0, 0.2),
    PID_linear: Tuple[float, float, float] = (0.8, 0, 0.2),
    angular_speed_limit: float = 1,  # old 2.84
    linear_speed_limit: float = 0.2,  # old 0.22
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
        stop_kill(wpmover)
        print("killed")
    except SystemExit:
        print("sys exit done")


def move_turn(
    target: Tuple[float, float],
    end_yaw_range: float = 0.01,
    PID_angular: Tuple[float, float, float] = (4, 0, 0.4),
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
        stop_kill(wpturner)
        print("killed")
    except SystemExit:
        print("sys exit done")

def stop_kill(node: Node):
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    now = time.time()
    while time.time() - now < 0.2:
        node.cmdvelpub.publish(twist)
        time.sleep(0.05)
    node.get_logger().info("Done")
    # node.subscription.destroy()
    node.destroy_node()

def angular(cur_pos, cur_rot, dx, dy, node: Node):
    i, j, cur_yaw = euler_from_quaternion(cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)
    target_yaw = math.atan2(dy , dx)
    target_rot = quaternion_from_euler(i,j,-target_yaw)

    rot_tf = quaternion_multiply(target_rot,(cur_rot.x, cur_rot.y, cur_rot.z, -cur_rot.w))
    _, _, rot_tf_yaw = euler_from_quaternion(rot_tf[0],rot_tf[1],rot_tf[2],rot_tf[3])
    # self.pid_angular.setpoint = rot_tf
    # yaw = yaw - math.pi if yaw > 0 else yaw + math.pi
    # self.get_logger().info('cur_yaw: %f target_yaw: %f diff: %f quat_tf: %f' % (cur_yaw, target_yaw, target_yaw - cur_yaw, rot_tf_yaw))
    # time.sleep(0.1)
    angular_speed = node.pid_angular(-rot_tf_yaw)
    return angular_speed, rot_tf_yaw

def dxdy(trans: tf2_ros.TransformStamped,node:Node):
    cur_pos = trans.transform.translation
    cur_rot = trans.transform.rotation
    dx = cur_pos.x - node.target_x
    dy = cur_pos.y - node.target_y
    # self.get_logger().info(str(cur_pos.x))
    # self.get_logger().info('Trans: %f, %f' % (cur_pos.x, cur_pos.y))
    return cur_pos, cur_rot, dx, dy

def return_cur_pos():
    return cur_pos

def main(args=None):
    rclpy.init(args=args)

    move_turn((1,-0.22))
    move_straight((1,-0.22))

    # wpturner = WPTurner((-1, 0.22))
    # try:
    #     rclpy.spin(wpturner)
    # except Exception and KeyboardInterrupt:
    #     pass
    # except SystemExit:
    #     print("sys exit done")

    # twist = Twist()
    # twist.linear.x = 0.0
    # twist.angular.z = 0.0
    # now = time.time()
    # while time.time() - now < 0.5:
    #     wpturner.cmdvelpub.publish(twist)
    #     time.sleep(0.1)
    # wpturner.get_logger().info("Done")
    # wpturner.subscription.destroy()
    # wpturner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
