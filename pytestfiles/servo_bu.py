from example_interfaces.srv import SetBool
import rclpy
from rclpy.node import Node
from time import sleep, time

import pigpio

class ServoService(Node):

    def __init__(self):
        super().__init__('servo_service')
        self.srv = self.create_service(SetBool, 'set_bool', self.set_bool_callback)

    def set_bool_callback(self, request, response):
        if request.data:
            self.throw_routine()
            response.success = True
            response.message = "Done at " + str(time())
        else:
            response.success = False
            response.message = "input is False"
        return response

    def throw_routine(self):
        self.pi = pigpio.pi()

        self.pi.set_PWM_frequency(12, 50) # set frequency to 50Hz for servo
        self.pi.set_PWM_frequency(13, 50) # set frequency to 50Hz for servo
        self.pi.set_PWM_dutycycle(12, 8) # 2.5% duty cycle, 2.5/100*255~7, 0 degrees
        self.pi.set_PWM_dutycycle(13, 33) # starting position

        sleep(0.1)

        self.pi.set_PWM_dutycycle(12, 22) # halfway mark
        self.pi.set_PWM_dutycycle(13, 19)

        sleep(0.5)

        self.pi.set_PWM_dutycycle(12, 27) # 3/4 mark
        self.pi.set_PWM_dutycycle(13, 14)

        sleep(3)

        self.pi.set_PWM_dutycycle(12, 33)
        self.pi.set_PWM_dutycycle(13, 8)

        sleep(1)

        self.pi.set_PWM_dutycycle(12, 24)
        self.pi.set_PWM_dutycycle(13, 17)

        sleep(1)

        self.pi.set_PWM_dutycycle(12, 8)
        self.pi.set_PWM_dutycycle(13, 33)

        sleep(0.1)

        self.pi.set_mode(12, pigpio.INPUT)
        self.pi.set_mode(13, pigpio.INPUT)
        self.pi.stop()

def main():
    rclpy.init()

    servo_service = ServoService()

    rclpy.spin(servo_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
