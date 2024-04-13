import math
import warnings
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


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
            self.angle_increment = (
                msg.angle_increment
            )  # @TODO need to check if this is variable...

    def run_check(self, iter=1):
        if iter != 1:
            for _ in range(iter):
                rclpy.spin_once(self)
                if self.averaged_ranges is None:
                    self.averaged_ranges = self.lidar_ranges
                else:
                    self.averaged_ranges += self.lidar_ranges
            self.averaged_ranges /= iter
        else:
            rclpy.spin_once(self)
            self.averaged_ranges = self.lidar_ranges

        self.angle = check_bucket_lidar(
            self.averaged_ranges,
            self.angle_increment,
            self.threshold,
            self.bucket_radius,
        )
        # np.savetxt("lidar_bucket.txt", self.averaged_ranges, "%0.5f")
        return self.angle


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
    bucket_scanner = BucketScanner(threshold=0.04)  # , bucket_radius=0.2286)
    print(
        bucket_scanner.run_check(iter=1)
    )  # On actual robot, angle_increment always changes
    bucket_scanner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
