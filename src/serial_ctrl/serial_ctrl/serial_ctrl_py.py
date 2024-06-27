import rclpy
from rclpy.node import Node
import serial
import sys
from sensor_msgs.msg import JointState

class MinimalSubscriber(Node):
    def __init__(self, port):
        super().__init__('serial_ctrl')
        self.position = []
        self.subscription = self.create_subscription(JointState, 'joint_states', self.listener_callback, 10)

        self.ser = serial.Serial(port, baudrate=115200, dsrdtr=None)
        self.ser.setRTS(False)
        self.ser.setDTR(False)
    
    def set_position(self, array):
        msgs = [0] * (5 * 2)  

        for i in range(5):
            array[i] = 500 + array[i] * 4
            numero = int(array[i])

            msgs[2 * i] = numero & 0x000000ff
            msgs[2 * i + 1] = (numero & 0x0000ff00) >> 8

        message = [
            0x55, 0x55, 0x14, 0x03, 0x05, 0x90, 0x01, 0x01,
            msgs[0], msgs[1], 0x02,
            msgs[2], msgs[3], 0x03,
            msgs[4], msgs[5], 0x04,
            msgs[6], msgs[7], 0x05,
            msgs[8], msgs[9]
        ]

        self.ser.write(bytearray(message))

    def listener_callback(self, msg):
        a = msg.position

        array = [a[0], a[1], a[2], a[3], a[4]]
        self.set_position(array)

def main(args=None):
    port = '/dev/ttyUSB0'

    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber(port)
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
