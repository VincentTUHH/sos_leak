#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import os
from std_srvs.srv import SetBool

# Pixhawk 6c settings
DEVICE_ADDRESS='/dev/ttyAMA0'
BAUD_RATE=115200

class LeakSafety(Node):
    def __init__(self):
        super().__init__(node_name='leak_safety')

        self.leak_main_sub = self.create_subscription(
            msg_type=Bool, 
            topic='main/leak', 
            callback=self.on_leak_main, 
            qos_profile=1
        )
        self.leak_buddy_sub = self.create_subscription(
            msg_type=Bool, 
            topic='buddy/leak', 
            callback=self.on_leak_buddy, 
            qos_profile=1
        )

        # adapt service topics 
        self.manipulator_client = self.create_client(
            srv_type=SetBool, 
            srv_name='arm_manipulator'
        )
        while not self.manipulator_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for manipulator disarm service...')
        
        self.vehicle_client = self.create_client(
            srv_type=SetBool, 
            srv_name='arm'
        )
        while not self.vehicle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for vehicle disarm service...')

        self.main_leak = False
        self.buddy_leak = False
        self.shutdown_triggered = False

    def on_leak_main(self, msg):
        self.main_leak = msg.data
        self.check_leak()

    def on_leak_buddy(self, msg):
        self.buddy_leak = msg.data
        self.check_leak()

    def check_leak(self):
        if self.main_leak or self.buddy_leak and not self.shutdown_triggered:
            self.shutdown_triggered = True
            self.get_logger().warn('🚨 Leak detected! Initiating safety response.')
            self.disarm_vehicle()
            self.disarm_manipulator()

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