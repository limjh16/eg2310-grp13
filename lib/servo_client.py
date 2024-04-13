import sys

from example_interfaces.srv import SetBool
import rclpy
from rclpy.node import Node


class ServoClientAsync(Node):

    def __init__(self):
        super().__init__('servo_client_async')
        self.cli = self.create_client(SetBool, 'set_bool')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetBool.Request()

    def send_request(self, request: bool):
        self.req.data = request
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = ServoClientAsync()
    response = minimal_client.send_request(True)
    minimal_client.get_logger().info(response.message)

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()