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

        try:
            self.pi.set_PWM_frequency(12, 50) # set frequency to 50Hz for servo
            self.pi.set_PWM_frequency(13, 50) # set frequency to 50Hz for servo
            self.pi.set_PWM_dutycycle(12, 8) # 2.5% duty cycle, 2.5/100*255~7, 0 degrees
            self.pi.set_PWM_dutycycle(13, 33) # starting position

            sleep(1)

            # go more than halfway
            self.pi.set_PWM_dutycycle(12, 28)
            self.pi.set_PWM_dutycycle(13, 13)

            sleep(1)

            # go all the way
            self.pi.set_PWM_dutycycle(12, 33)
            self.pi.set_PWM_dutycycle(13, 8)

            sleep(2)

            for i in range(1, 33-8+1):
                self.pi.set_PWM_dutycycle(12, 33-i)
                self.pi.set_PWM_dutycycle(13, 8+i)
                sleep(0.1)

        except KeyboardInterrupt:
            pass

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
