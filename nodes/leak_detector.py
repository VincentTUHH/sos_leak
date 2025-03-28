#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import RPi.GPIO as GPIO

LEAK_SENSOR_PIN = 17  # Adjust based on wiring

class LeakDetector(Node):
    def __init__(self):
        super().__init__(node_name='leak_detector')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('tube_name', rclpy.Parameter.Type.STRING),
            ],
        )

        tube_name = self.get_parameter('tube_name').get_parameter_value().string_value


        self.leak_pub = self.create_publisher(
            msg_type=Bool, 
            topic=f'{tube_name}/leak', 
            qos_profile=1
        )

        self.timer = self.create_timer(
            timer_period_sec=1 / 50,
            callback=self.on_timer,
        )

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(LEAK_SENSOR_PIN, GPIO.IN)

        self.get_logger().info('Leak detector node started.')

    def on_timer(self):
        leak = GPIO.input(LEAK_SENSOR_PIN) == GPIO.HIGH
        msg = Bool()
        msg.data = leak
        self.leak_pub.publish(msg)
        self.get_logger().info(f'Leak detected: {leak}')

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