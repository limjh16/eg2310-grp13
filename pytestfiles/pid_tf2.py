import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage
import numpy as np
from geometry_msgs.msg import Twist
from lib.tf2_quat_utils import euler_from_quaternion, quaternion_from_euler
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from simple_pid import PID

class WPMover(Node):

    def __init__(self):
        super().__init__('wpmover')
        self.subscription = self.create_subscription(
            TFMessage,
            'tf',
            self.listener_callback,
            qos_profile_sensor_data)
        # self.subscription  # prevent unused variable warning
        # self.subscription = self.create_timer(0.01,self.listener_callback)
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.pid_angular = PID(1, 0, 2, output_limits=(-2.84,2.84))
        self.pid_linear = PID(0.3, 0, 1, setpoint = 0, output_limits=(-0.22,0))
        self.cmdvelpub = self.create_publisher(Twist,'cmd_vel',10)

    def listener_callback(self,_):
        try:
            trans = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info('No transformation found')
            return
        cur_pos = trans.transform.translation
        cur_rot = trans.transform.rotation
        # self.get_logger().info('Trans: %f, %f' % (cur_pos.x, cur_pos.y))
        # convert quaternion to Euler angles
        _, _, yaw = euler_from_quaternion(cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)
        yaw = yaw - math.pi if yaw > 0 else yaw + math.pi
        # self.get_logger().info('Rot-Yaw: R: %f D: %f' % (yaw, np.degrees(yaw)))
        dx = cur_pos.x - (-0.45)
        dy = cur_pos.y - (-0.25)
        linear_dist = math.sqrt(dx**2+dy**2)
        linear_speed = self.pid_linear(linear_dist)
        self.pid_angular.setpoint = math.atan(dy/dx)
        angular_speed = self.pid_angular(yaw)
        twist = Twist()
        if linear_dist < 0.03:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmdvelpub.publish(twist)
            self.get_logger().info('Done')
            self.subscription.destroy()
            self.destroy_node()
        twist.linear.x = float(abs(linear_speed))
        twist.angular.z = angular_speed
        self.cmdvelpub.publish(twist)
        # self.get_logger().info('LinSpd: '+str(linear_speed)+' LinDst: '+str(linear_dist)+' AngSpd: '+str(angular_speed)+' Yaw: '+str(yaw))




def main(args=None):
    rclpy.init(args=args)

    wpmover = WPMover()
    try:
        rclpy.spin(wpmover)
    except Exception and KeyboardInterrupt:
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        wpmover.cmdvelpub.publish(twist)

    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    wpmover.cmdvelpub.publish(twist)
    wpmover.get_logger().info('Done')
    wpmover.subscription.destroy()
    wpmover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
