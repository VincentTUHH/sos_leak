#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger
import os

# Pixhawk 6c settings
DEVICE_ADDRESS = '/dev/ttyAMA0'
BAUD_RATE = 115200


class LeakSafety(Node):

    def __init__(self):
        super().__init__('leak_safety_service')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('tube_name', rclpy.Parameter.Type.STRING),
            ],
        )
        self.tube_name = self.get_parameter(
            'tube_name').get_parameter_value().string_value

        # Leak service client
        self.leak_check_client = self.create_client(srv_type=Trigger,
                                                    srv_name='get_leak')
        while not self.leak_check_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for /get_leak service...')

        # Manipulator disarm service
        self.manipulator_client = self.create_client(srv_type=SetBool,
                                                     srv_name='arm_manipulator')
        while not self.manipulator_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for manipulator disarm service...')

        # Vehicle disarm service
        self.vehicle_client = self.create_client(srv_type=SetBool,
                                                 srv_name='arm')
        while not self.vehicle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for vehicle disarm service...')

        # Internal state
        self.shutdown_triggered = False

        # Timer to check leak state regularly
        self.timer = self.create_timer(timer_period_sec=1 / 50,
                                       callback=self.on_check_leak)

        self.get_logger().info(
            f'Leak safety node started for tube: {self.tube_name}')

    def on_check_leak(self):
        if self.shutdown_triggered:
            return

        req = Trigger.Request()
        future = self.leak_check_client.call_async(req)
        future.add_done_callback(self.handle_leak_response)

    def handle_leak_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().warn(
                    'Leak detected! Initiating safety response.')
                self.shutdown_triggered = True
                self.disarm_vehicle()
                self.disarm_manipulator()
        except Exception as e:
            self.get_logger().error(f'Error checking leak status: {e}')

    def disarm_vehicle(self):
        req = SetBool.Request()
        req.data = False
        future = self.vehicle_client.call_async(req)
        future.add_done_callback(self.handle_disarm_response)

    def disarm_manipulator(self):
        req = SetBool.Request()
        req.data = False
        future = self.manipulator_client.call_async(req)
        future.add_done_callback(self.handle_disarm_response)

    def handle_disarm_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Disarm successful: {response.message}')
            else:
                self.get_logger().warn(f'Disarm failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Disarm service call failed: {e}')
        self.shutdown_pi()

    def shutdown_pi(self):
        self.get_logger().info('Shutting down Raspberry Pi...')
        os.system('sudo shutdown now')


def main():
    rclpy.init()
    node = LeakSafety()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # finally:
    #     node.destroy_node()
    #     rclpy.shutdown()


if __name__ == '__main__':
    main()
