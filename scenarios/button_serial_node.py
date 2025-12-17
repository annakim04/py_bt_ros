#!/usr/bin/env python3

import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8


class ButtonSerialNode(Node):
    def __init__(self):
        super().__init__('button_serial_node')

        # publisher
        self.publisher = self.create_publisher(
            UInt8,
            '/limo/button_status',
            10
        )

        # 반드시 by-id 사용(esp코드로 인한 mqtt로 연결해본 결과 안돼서 물리적 연결로 바꿈)
        self.port_name = (
            '/dev/serial/by-id/'
            'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
        )

        try:
            self.serial_port = serial.Serial(
                port=self.port_name,
                baudrate=115200,
                timeout=0.1
            )
            self.get_logger().info(f'Opened serial port: {self.port_name}')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise

        self.timer = self.create_timer(0.01, self.read_serial)

    def read_serial(self):
        try:
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode(errors='ignore').strip()

                if line in ('0', '1'):
                    msg = UInt8()
                    msg.data = int(line)
                    self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Serial read error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ButtonSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
