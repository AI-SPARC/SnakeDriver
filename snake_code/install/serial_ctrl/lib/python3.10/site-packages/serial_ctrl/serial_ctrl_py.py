import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from math import pi, floor
import serial
import threading

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('serial_ctrl')
        self.position = []
        self.subscription = self.create_subscription(JointState, 'joint_states', self.listener_callback, 10)

        self.ser = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=1)
        self.ser.setRTS(False)
        self.ser.setDTR(False)

        self.set_position_periodic()

    def set_position(self, array, timer=False):
        msgs = [0] * (5 * 2)  

        if len(array) != 5:
            self.get_logger().warn(f"Received position array with unexpected length: {len(array)}")
        else:
            for i in range(5):
                array[i] = 500 + array[i] * 4
                numero = floor(array[i])
                msgs[2 * i] = numero & 0x000000ff
                msgs[2 * i + 1] = (numero & 0x0000ff00) >> 8

            message = [
                0x55, 0x55, 0x14, 0x03, 0x05, 0x20, 0x03, 0x01,
                msgs[0], msgs[1], 0x02,
                msgs[2], msgs[3], 0x03,
                msgs[4], msgs[5], 0x04,
                msgs[6], msgs[7], 0x05,
                msgs[8], msgs[9]
            ]

            self.ser.write(message)

            if not timer:
                print("Sent position:", array)

    def set_position_periodic(self):
        threading.Timer(1.0, self.set_position_periodic).start()  # Chama a função a cada 2 segundos
        self.set_position(self.position, timer=True)

    def listener_callback(self, msg):
        a = msg.position
        angle1 = a[0] * (180.0 / pi)
        angle2 = a[1] * (180.0 / pi)
        angle3 = a[2] * (180.0 / pi)
        angle4 = a[3] * (180.0 / pi)
        angle5 = a[4] * (180.0 / pi)

        array = [angle5, angle4, angle3, angle2, angle1]
        self.position = array  # Salva a posição atual para enviar periodicamente

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
