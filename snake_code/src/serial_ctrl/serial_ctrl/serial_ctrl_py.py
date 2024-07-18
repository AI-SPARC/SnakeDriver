import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from math import pi, floor
import serial
import threading
import keyboard  # Import the keyboard library


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('serial_ctrl')
        self.position = []
        self.subscription = self.create_subscription(JointState, 'joint_states', self.listener_callback, 10)

        self.ser = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=1)
        self.ser.setRTS(False)
        self.ser.setDTR(False)
       
        self.create_timer(0.2, self.read_message)
        
        stop_message = [
            0x55, 0x55, 0x02, 0x07,
        ]
        self.ser.write(stop_message)

        self.message1 = 0

    def set_position(self):
        array = self.position
        msgs = [0] * (5 * 2)

        if len(array) != 5:
            self.get_logger().warn(f"Posição do robô não publicada")
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

            if self.message1 != message:
                self.ser.write(message)
                print("Mensagem enviada! Mensagem:", message)
                print("Data:", bytearray(message))
                self.message1 = message
            else:
                self.message1 = message

            # if not timer:
            #     print("Posição:", array)

       
    def read_message(self):
        
        message3 = [
            0x55, 0x55, 0x08, 0x15,
            0x05, 0x01, 0x02, 0x03,
            0x04, 0x05
        ]
        
        self.ser.write(message3)
        data = self.ser.read(20)
        message = data.hex()
        separated_string = ' '.join([message[i:i+2] for i in range(0, len(message), 2)])
        print("Data received:", separated_string)

       
    # def set_position_periodic(self):
    #     threading.Timer(0.1, self.set_position_periodic).start()
    #     self.set_position(self.position, timer=True)

    # def read_message_periodic(self):
    #     threading.Timer(0.01, self.read_message_periodic).start()
    #     self.read_message()

    def listener_callback(self, msg):
        a = msg.position
        angle1 = a[0] * (180.0 / pi)
        angle2 = a[1] * (180.0 / pi)
        angle3 = a[2] * (180.0 / pi)
        angle4 = a[3] * (180.0 / pi)
        angle5 = a[4] * (180.0 / pi)

        array = [angle1, angle2, angle3, angle4, angle5]
        self.position = array

        self.set_position()

   

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
