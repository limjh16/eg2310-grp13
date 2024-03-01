import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import scipy.stats
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import cv2 as cv

# constants
occ_bins = [-1, 0, 50, 100]

def dilate123(src, size):
    array_edited = np.copy(src)
    array_edited[array_edited <= 2] = 0
    array_dilated = cv.dilate(
        array_edited,
        cv.getStructuringElement(cv.MORPH_CROSS, (2 * size + 1, 2 * size + 1)),
    )
    return np.maximum(src, array_dilated)


class Occupy(Node):
    def __init__(self):
        super().__init__("occupy")
        self.subscription = self.create_subscription(
            OccupancyGrid, "map", self.occ_callback, qos_profile_sensor_data
        )
        self.subscription  # prevent unused variable warning
        # occdata = np.array([])

    def occ_callback(self, msg):
        occdata = np.array(msg.data)
        _, _, binnum = scipy.stats.binned_statistic(
            occdata, np.nan, statistic="count", bins=occ_bins
        )
        odata = np.uint8(binnum.reshape(msg.info.height, msg.info.width))
        odata = dilate123(odata, 4)

        # create image from 2D array using PIL
        img = Image.fromarray(odata)
        # show the image using grayscale map
        plt.imshow(img, cmap="gray", origin="lower")
        plt.draw_all()
        # pause to make sure the plot gets created
        plt.pause(0.00000000001)


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

        current_min = (
            current_min[0] * msg.info.resolution + msg.info.origin.position.x,
            current_min[1] * msg.info.resolution + msg.info.origin.position.y,
        )

        self.get_logger().info("New Origin: " + str(current_min))

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


def main(args=None):
    rclpy.init(args=args)

    occupy = Occupy()
    firstoccupy = FirstOccupy()
    rclpy.spin_once(firstoccupy)

    # create matplotlib figure
    plt.ion()
    plt.show()

    rclpy.spin(occupy)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    occupy.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
