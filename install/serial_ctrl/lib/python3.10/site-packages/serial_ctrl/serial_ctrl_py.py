import rclpy
from rclpy.node import Node
import serial
from sensor_msgs.msg import JointState
from math import pi, floor

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

        # print("msgs[0]:", msgs[0])
        # print("msgs[1]:", msgs[1])
        # print("msgs[2]:", msgs[2])
        # print("msgs[3]:", msgs[3])
        # print("msgs[4]:", msgs[4])
        # print("msgs[5]:", msgs[5])
        # print("msgs[6]:", msgs[6])
        # print("msgs[7]:", msgs[7])
        # print("msgs[8]:", msgs[8])
        # print("msgs[9]:", msgs[9])

        # print("a[0]:", array[0])
        # print("a[1]:", array[1])
        # print("a[2]:", array[2])
        # print("a[3]:", array[3])
        # print("a[4]:", array[4])

        self.ser.write(bytearray(message))

    def listener_callback(self, msg):
        a = msg.position

        angle1 = floor(a[0] * (180.0 / pi))
        angle2 = floor(a[1] * (180.0 / pi))
        angle3 = floor(a[2] * (180.0 / pi))
        angle4 = floor(a[3] * (180.0 / pi))
        angle5 = floor(a[4] * (180.0 / pi))

        print("angle1:", angle1)
        print("angle2:", angle2)
        print("angle3:", angle3)
        print("angle4:", angle4)
        print("angle5:", angle5)
        
        array = [angle1, angle2, angle3, angle4, angle5]    

      

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
