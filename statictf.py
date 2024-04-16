import rclpy
from rclpy import Node
from geometry_msgs.msg import PointStamped
from rclpy.qos import qos_profile_system_default
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class StaticTF(Node):

    def __init__(self):
        super().__init__('statictf')
        self.subscription = self.create_subscription(
            PointStamped,
            'scan',
            self.listener_callback,
            qos_profile_system_default)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

