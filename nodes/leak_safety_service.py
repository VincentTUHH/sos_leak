#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger, Empty
from buttons_msgs.msg import Button
import os

# Pixhawk 6c settings
DEVICE_ADDRESS = '/dev/ttyAMA0'
BAUD_RATE = 115200


class LeakSafety(Node):

    def __init__(self):
        super().__init__('leak_safety_service')

        self.buzzer_pub = self.create_publisher(Button, 'sos_buzzer', 10)
        
        # Leak service client
        global_path_get_leak = 'get_leak'
        self.leak_check_client = self.create_client(
            srv_type=Trigger, srv_name=global_path_get_leak)
        while not self.leak_check_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for /get_leak service...')

        # Manipulator disarm service
        global_path_arm_manipulator = 'arm_manipulator'
        self.manipulator_client = self.create_client(
            srv_type=SetBool, srv_name=global_path_arm_manipulator)

        # Vehicle disarm service
        global_path_arm = 'arm'
        self.vehicle_client = self.create_client(srv_type=SetBool,
                                                 srv_name=global_path_arm)

        self.stop_alarm_srv = self.create_service(
            Empty,  # Standard service without request fields
            'stop_alarm',
            self.stop_alarm
        )

        # Internal state
        self.vehicle_disarmed = False
        self.manipulator_disarmed = False
        self.shutdown_triggered = False
        self.alarm_active = False
        self.alarm_timer = None

        # Timer to check leak state regularly
        self.timer = self.create_timer(timer_period_sec=1 / 50,
                                       callback=self.on_check_leak)

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
                self.start_alarm()
                self.disarm_vehicle()
                self.disarm_manipulator()
        except Exception as e:
            self.get_logger().error(f'Error checking leak status: {e}')

    def disarm_vehicle(self):
        if not self.vehicle_client.service_is_ready():
            self.get_logger().warn('Vehicle disarm service not available.')
            return
        req = SetBool.Request()
        req.data = False
        future = self.vehicle_client.call_async(req)
        future.add_done_callback(
            lambda f: self.handle_disarm_response(f, 'vehicle'))

    def disarm_manipulator(self):
        if not self.manipulator_client.service_is_ready():
            self.get_logger().warn('Manipulator disarm service not available.')
            return
        req = SetBool.Request()
        req.data = False
        future = self.manipulator_client.call_async(req)
        future.add_done_callback(
            lambda f: self.handle_disarm_response(f, 'manipulator'))

    # def handle_disarm_response(self, future):
    #     try:
    #         response = future.result()
    #         if response.success:
    #             self.get_logger().info(f'Disarm successful: {response.message}')
    #         else:
    #             self.get_logger().warn(f'Disarm failed: {response.message}')
    #     except Exception as e:
    #         self.get_logger().error(f'Disarm service call failed: {e}')
    #     self.shutdown_pi()

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
        # To-Do: hier den Buzzer aktivieren, statt den UVMS lahm zu legen
        # oder: volles Stromzufuhr kappen -> relais
        # if self.vehicle_disarmed and self.manipulator_disarmed and self.shutdown_triggered:
        #     self.shutdown_pi()

    def shutdown_pi(self):
        self.get_logger().info('Shutting down Raspberry Pi...')
        os.system('sudo shutdown now')

    def start_alarm(self):
        if self.alarm_timer is None:
            self.alarm_active = True
            self.alarm_timer = self.create_timer(0.5, self.publish_alarm)

    def publish_alarm(self):
        if self.alarm_active:
            msg = Button()
            msg.button = 1 # value is irrelevant for buzzer activation
            self.buzzer_pub.publish(msg)

    def stop_alarm(self, request, response):
        if self.alarm_timer is not None:
            self.alarm_timer.cancel()
            self.alarm_timer = None
        self.alarm_active = False
        self.get_logger().info('Alarm stopped.')
        return response


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
