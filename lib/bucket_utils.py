import math
import warnings
import numpy as np
import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from .tf2_quat_utils import (
    euler_from_quaternion,
    quaternion_from_euler,
    quaternion_multiply,
)
from .pid_tf2 import move_straight, move_turn


def check_bucket(
    perp_dist: float,
    b: float,
    theta: math.radians,
    threshold: float = 0.03,
    bucket_radius: float = 0.135,
) -> bool:
    """Check if the given LiDAR distance / angle is shooting at the EG2310 bucket

    Args:
        perp_dist (float (in meters)): Shortest perpendicular distance to the bucket
        b (float (in meters)): Selected test point at theta rad away from shortest perpendicular distance
        theta (math.radians): Angle in radians away from the shortest distance angle
        threshold (float (in meters), optional): Margin for error when checking. Defaults to 0.03.
        bucket_radius (float (in meters), optional): Radius of bucket, just in case changes need to be made. Defaults to 0.135.

    Returns:
        bool: True or not, if it matches expected bucket dimension
    """
    a = perp_dist + bucket_radius
    cos_theta = math.cos(theta)
    two_a_cos_theta = 2 * a * cos_theta
    # print((two_a_cos_theta, a))
    b_expected = (
        two_a_cos_theta - math.sqrt(two_a_cos_theta**2 - 4 * (a**2 - bucket_radius**2))
    ) / 2
    # print(b_expected)
    if abs(b - b_expected) < threshold:
        return True
    else:
        return False


def check_bucket_lidar(
    lidar_ranges: np.ndarray,
    angle_increment: math.radians,
    threshold: float = 0.03,
    bucket_radius: float = 0.135,
) -> math.radians:
    """Return angle of bucket given a LiDAR ranges array

    Args:
        lidar_ranges (np.ndarray): obtain from msg.ranges
        angle_increment (math.radians): How much each array index represents in angle increments
        threshold (float (in meters), optional): Margin for error when checking. Defaults to 0.03.
        bucket_radius (float (in meters), optional): Radius of bucket, just in case changes need to be made. Defaults to 0.135.

    Returns:
        math.radians: Angle of center of bucket
    """
    if abs(lidar_ranges.size * angle_increment - 6.28) > 0.1:
        warnings.warn("lidar_ranges.size does not match angle_increment!!")
    for i in np.argsort(lidar_ranges):  # start checking from shortest
        if (math.pi/4)/angle_increment > i or i > (7*math.pi/4)/angle_increment:
            # print("out of range, selected point is behind robot")
            continue
        distance = lidar_ranges[i]
        theta: math.radians = math.asin(bucket_radius / (bucket_radius + distance))
        checking_fractions = (0.85, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1)
        # checking_fractions = (0.75, 0.6, 0.5, 0.3)
        exit = True
        for frac in checking_fractions:
            target_theta = frac * theta
            target_index_plus = (
                i + int(target_theta // angle_increment)
            ) % lidar_ranges.size
            target_index_minus = (
                i - int(target_theta // angle_increment)
            ) % lidar_ranges.size
            if bool(
                {
                    lidar_ranges[i],
                    lidar_ranges[target_index_plus],
                    lidar_ranges[target_index_minus],
                }
                & {np.inf}
            ):
                exit = False
                break
            # @TODO add a check for if range = 0 (i.e. if range = nan), so that we are still able to use the other adjacent ranges
            left_check = check_bucket(
                lidar_ranges[i],
                lidar_ranges[target_index_plus],
                target_theta,
                threshold=threshold,
                bucket_radius=bucket_radius,
            )
            right_check = check_bucket(
                lidar_ranges[i],
                lidar_ranges[target_index_minus],
                target_theta,
                threshold=threshold,
                bucket_radius=bucket_radius,
            )
            if left_check is False or right_check is False:
                exit = False
                break
        if exit:
            return i * angle_increment
    return None


class BucketScanner(Node):
    def __init__(self, threshold: float = 0.03, bucket_radius: float = 0.135):
        """Spin Node to obtain current LiDAR data and check for bucket

        Args:
            threshold (float (in meters), optional): Margin for error when checking. Defaults to 0.03.
            bucket_radius (float (in meters), optional): Radius of bucket, just in case changes need to be made. Defaults to 0.135.
        """
        super().__init__("bucket_scanner")
        self.subscription = self.create_subscription(
            LaserScan, "scan", self.listener_callback, qos_profile_sensor_data
        )
        self.angle_increment = None
        self.threshold = threshold
        self.bucket_radius = bucket_radius
        self.averaged_ranges: np.ndarray = None
        self.lidar_ranges: np.ndarray = None

    def listener_callback(self, msg):
        self.lidar_ranges = np.array(msg.ranges)
        # self.lidar_ranges[self.lidar_ranges == 0] = np.nan  # replace 0's with nan
        self.lidar_ranges[np.isnan(self.lidar_ranges)] = 0
        if self.angle_increment is None:
            self.angle_increment = msg.angle_increment

    def pub_bucket(self, iter=5):
        cnt = 0
        fail_cnt = 0
        avg_angle = 0
        avg_dist = 0
        while cnt < iter:
            if self.run_check() is None:
                fail_cnt += 1
                if fail_cnt >= iter:
                    print("No Bucket Found!")
                    return None
            else:
                cnt += 1
                avg_angle += self.angle
                avg_dist += self.lidar_ranges[int(self.angle / self.angle_increment)]
                print(
                    "Angle: "
                    + str(self.angle)
                    + " | Dist: "
                    + str(self.lidar_ranges[int(self.angle / self.angle_increment)])
                )
        avg_angle /= iter
        avg_dist /= iter
        avg_angle = np.arctan2(
            np.sin(-avg_angle), np.cos(-avg_angle)
        )  # https://stackoverflow.com/a/2321125
        avg_dist += self.bucket_radius
        print("Avg Angle: " + str(avg_angle) + " | Avg Dist: " + str(avg_dist))

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        while True:
            try:
                rclpy.spin_once(self)
                trans = self.tfBuffer.lookup_transform(
                    "map",
                    "base_scan",
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.2),
                )
                break
            except (
                LookupException,
                ConnectivityException,
                ExtrapolationException,
            ) as e:
                self.get_logger().info("No transformation found " + str(e))
        cur_pos = trans.transform.translation
        cur_rot = trans.transform.rotation
        # _, _, scan_yaw = euler_from_quaternion(
        #     cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w
        # )
        q_r = quaternion_from_euler(0, 0, avg_angle)
        q_2 = quaternion_multiply(q_r, (cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w))
        # bucket_yaw_from_scan = scan_yaw - avg_angle
        _, _, bucket_yaw_from_scan = euler_from_quaternion(*q_2)
        dx = avg_dist * np.cos(bucket_yaw_from_scan)
        dy = avg_dist * np.sin(bucket_yaw_from_scan)
        self.bucket_pos = (cur_pos.x + dx, cur_pos.y + dy)
        # print(
        #     "dxdy: "
        #     + str((dx, dy))
        #     + " | bucket yaw: "
        #     + str(bucket_yaw_from_scan)
        #     + " | robot yaw: "
        #     + str(scan_yaw)
        # )

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "bucket"

        t.transform.translation.x = self.bucket_pos[0]
        t.transform.translation.y = self.bucket_pos[1]
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_static_broadcaster.sendTransform(t)
        return 1

    def move_to_bucket(self, dist=0.33):
        print(dist)
        move_turn(self.bucket_pos, end_yaw_range=0.03)
        move_straight(self.bucket_pos, end_distance_range=dist)

    def run_check(self):
        rclpy.spin_once(self)

        self.angle = check_bucket_lidar(
            self.lidar_ranges,
            self.angle_increment,
            self.threshold,
            self.bucket_radius,
        )
        # np.savetxt("lidar_bucket.txt", self.averaged_ranges, "%0.5f")
        return self.angle


def move_to_bucket(threshold=0.04, dist=0.33, iter=5, bucket_radius=0.135):
    """Move to bucket based on LiDAR

    Args:
        threshold (float (meters), optional): Threshold for bucket detection. Defaults to 0.04.
        dist (float (meters), optional): Distance before bucket to stop (measured to center of bucket). Defaults to 0.33.
        iter (int, optional): Number of times to collect and average bucket location. Defaults to 5.
        bucket_radius (float (meters), optional): Radius of bucket. Defaults to 0.135.
    """
    bucket_scanner = BucketScanner(threshold=threshold, bucket_radius=bucket_radius)
    # print(bucket_scanner.run_check())  # On actual robot, angle_increment always changes
    if bucket_scanner.pub_bucket(iter=iter) is not None:
        bucket_scanner.move_to_bucket(dist=dist)
        return 0
    else:
        return None
    bucket_scanner.destroy_node()


def main(args=None):
    # print(check_bucket(0.3746, 0.39, np.deg2rad(13.5)))

    # angle_increment = math.pi / 180  # obtain this from msg.angle_increment
    # lidar_ranges = np.loadtxt("lidar_bucket.txt")
    # print(
    #     np.rad2deg(
    #         check_bucket_lidar(
    #             lidar_ranges, angle_increment, threshold=0.04, bucket_radius=0.2286
    #         )
    #     )
    # )

    rclpy.init(args=args)
    move_to_bucket()  # bucket_radius=0.2286)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
