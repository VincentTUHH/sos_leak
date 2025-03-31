#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger


class LeakServer(Node):

    def __init__(self):
        super().__init__('leak_server')

        # Internal leak status
        self.leak_detected = False

        # Declare services
        self.set_service = self.create_service(srv_type=SetBool,
                                               srv_name='set_leak',
                                               callback=self.on_set_leak)
        self.get_service = self.create_service(srv_type=Trigger,
                                               srv_name='get_leak',
                                               callback=self.on_get_leak)

        self.get_logger().info('Leak server node started.')

    def on_set_leak(self, request, response):
        if request.data:
            if not self.leak_detected:
                self.get_logger().warn('Leak detected! Setting flag to True.')
            self.leak_detected = True
        else:
            self.get_logger().info(
                'Received request to reset leak status to False.')
            self.leak_detected = False

        response.success = True
        response.message = f'Leak status set to {self.leak_detected}'
        return response

    def on_get_leak(self, request, response):
        response.success = self.leak_detected
        response.message = f'Leak detected: {self.leak_detected}'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LeakServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # finally:
    #     node.destroy_node()
    #     rclpy.shutdown()


if __name__ == '__main__':
    main()
