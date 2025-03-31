#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import RPi.GPIO as GPIO

LEAK_SENSOR_PIN = 17  # Adjust based on wiring


class LeakDetector(Node):

    def __init__(self):
        super().__init__('leak_detector_service')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('tube_name', rclpy.Parameter.Type.STRING),
            ],
        )
        self.tube_name = self.get_parameter(
            'tube_name').get_parameter_value().string_value

        # Internal flag to avoid repeated calls
        self.leak_already_sent = False

        # Create client to leak server
        self.cli = self.create_client(srv_type=SetBool, srv_name='set_leak')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for set_leak service...')

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(LEAK_SENSOR_PIN, GPIO.IN)

        self.timer = self.create_timer(1.0 / 50, self.on_timer)

        self.get_logger().info(
            f'Leak detector started for tube: {self.tube_name}')

    def on_timer(self):
        leak = GPIO.input(LEAK_SENSOR_PIN) == GPIO.HIGH

        if leak and not self.leak_already_sent:
            self.get_logger().warn('LEAK DETECTED! Notifying server...')
            self.send_leak_alert()
            self.leak_already_sent = True
        # elif not leak and self.leak_already_sent:
        #     # Optional: allow reset if leak goes away (for testing)
        #     self.get_logger().info('Leak signal cleared.')
        #     self.leak_already_sent = False

    def send_leak_alert(self):
        req = SetBool.Request()
        req.data = True
        self.cli.call_async(req)

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()


def main():
    rclpy.init()
    node = LeakDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # finally:
    #     node.destroy_node()
    #     rclpy.shutdown()


if __name__ == '__main__':
    main()
