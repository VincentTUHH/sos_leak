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
                ('vehicle_name', rclpy.Parameter.Type.STRING),
            ],
        )

        self.tube_name = self.get_parameter(
            'tube_name').get_parameter_value().string_value
        self.vehicle_name = self.get_parameter(
            'vehicle_name').get_parameter_value().string_value

        # Leak service client
        global_path_get_leak = f'/{self.vehicle_name}/get_leak'
        self.leak_check_client = self.create_client(
            srv_type=Trigger, srv_name=global_path_get_leak)
        while not self.leak_check_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for /get_leak service...')

        # Manipulator disarm service
        global_path_arm_manipulator = f'/{self.vehicle_name}/arm_manipulator'
        self.manipulator_client = self.create_client(
            srv_type=SetBool, srv_name=global_path_arm_manipulator)
        while not self.manipulator_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for manipulator disarm service...')

        # Vehicle disarm service
        global_path_arm = f'/{self.vehicle_name}/arm'
        self.vehicle_client = self.create_client(srv_type=SetBool,
                                                 srv_name=global_path_arm)
        while not self.vehicle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for vehicle disarm service...')

        # Internal state
        self.vehicle_disarmed = False
        self.manipulator_disarmed = False
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
        future.add_done_callback(
            lambda f: self.handle_disarm_response(f, 'vehicle'))

    def disarm_manipulator(self):
        req = SetBool.Request()
        req.data = False
        future = self.manipulator_client.call_async(req)
        future.add_done_callback(
            lambda f: self.handle_disarm_response(f, 'manipulator'))

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

    def handle_disarm_response(self, future, component):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f'{component.capitalize()} disarm successful: {response.message}'
                )
            else:
                self.get_logger().warn(
                    f'{component.capitalize()} disarm failed: {response.message}'
                )
        except Exception as e:
            self.get_logger().error(
                f'{component.capitalize()} disarm service call failed: {e}')

        # Track which disarm succeeded
        if component == 'vehicle':
            self.vehicle_disarmed = True
        elif component == 'manipulator':
            self.manipulator_disarmed = True

        # Check if both are done
        if self.vehicle_disarmed and self.manipulator_disarmed and self.shutdown_triggered:
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
