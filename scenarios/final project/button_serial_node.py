"""
esp32 아두이노를 활용해서 1과 0을 받는 코드 만들어서 esp32에 코드 저장 
const int buttonPin = 33;

int lastStableState = HIGH;
int lastReading = HIGH;

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;  // ms

void setup() {
  Serial.begin(115200);
  pinMode(buttonPin, INPUT_PULLUP);
}

void loop() {
  int reading = digitalRead(buttonPin);

  // 값이 바뀌면 시간 기록
  if (reading != lastReading) {
    lastDebounceTime = millis();
  }

  // 일정 시간 동안 상태가 유지되면 안정 상태로 인정
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != lastStableState) {
      lastStableState = reading;

      if (lastStableState == LOW) {
        Serial.println("1");  // pressed
      } else {
        Serial.println("0");  // released
      }
    }
  }

  lastReading = reading;
}
"""


#!/usr/bin/env python3
#esp코드에서 0과 1에 대해서 포트로 물리적으로 연결해서 값을 받아오면 밑 노드가 ros화 시켜서 값을 받아옴
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8


class ButtonSerialNode(Node):
    def __init__(self):
        super().__init__('button_serial_node')

        # 버튼 상태(0 / 1)를 퍼블리시할 토픽
        self.publisher = self.create_publisher(
            UInt8,
            '/limo/button_status',
            10
        )

        # ESP32가 연결된 시리얼 포트 (고정 경로 사용)
        self.port_name = (
            '/dev/serial/by-id/'
            'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
        )

        # 시리얼 포트 열기
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

        # 0.01초마다 시리얼 데이터를 읽음
        self.timer = self.create_timer(0.01, self.read_serial)

    def read_serial(self):
        # ESP32에서 데이터가 들어왔는지 확인
        try:
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode(
                    errors='ignore'
                ).strip()

                # 버튼 값이 0 또는 1일 때만 처리
                if line in ('0', '1'):
                    msg = UInt8()
                    msg.data = int(line)
                    self.publisher.publish(msg)
        except Exception as e:
            # USB 끊김 등 시리얼 에러 로그 출력
            self.get_logger().error(f'Serial read error: {e}')


def main(args=None):
    rclpy.init(args=args)

    node = ButtonSerialNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
