import rclpy
import serial
import threading
import keyboard

from rclpy.node import Node
from sensor_msgs.msg import JointState
from math import pi, floor

DIVISAO = 180 / pi

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('serial_ctrl')
        self.position = []
        self.subscription = self.create_subscription(JointState, 'joint_states', self.listener_callback, 10)
        self.publisher = self.create_publisher(JointState, 'target_joint_states', 10)

        self.ser = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=1)
        self.ser.setRTS(False)
        self.ser.setDTR(False)
       
        self.create_timer(0.01, self.read_message)
        
        stop_message = [
            0x55, 0x55, 0x02, 0x07,
        ]
        self.ser.write(stop_message)

        self.message1 = 0

    def set_position(self):
        array = self.position
        msgs = [0] * (5 * 2)

        if len(array) != 5:
            # self.get_logger().warn(f"Posição do robô não publicada")
            pass
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
                # print("Mensagem enviada! Mensagem:", message)
                # print("Data:", bytearray(message))
                self.message1 = message
            else:
                self.message1 = message

            
       
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
        corrected_array = []
        concatenated_values = []
        decimal_values = []
        hex_pairs = separated_string.split()
        n = len(hex_pairs)


        a = [0] * 5
        
        # Encontrar a posição da sequência '55 55'
        for i in range(n - 1):
            if hex_pairs[i] == '55' and hex_pairs[i + 1] == '55':
                # Reorganizar o array
                corrected_array = hex_pairs[i:] + hex_pairs[:i]
                break
        
        if len(corrected_array) >= 20:
            # Concatenar os pares específicos
            concatenated_values = [
                corrected_array[7] + corrected_array[6],
                corrected_array[10] + corrected_array[9],
                corrected_array[13] + corrected_array[12],
                corrected_array[16] + corrected_array[15],
                corrected_array[19] + corrected_array[18]
            ]

            decimal_values = [int(hex_pair, 16) for hex_pair in concatenated_values]

            # Imprimir os valores decimais
            # print("Valores DECIMAIS", decimal_values)

            a = decimal_values

            a[0] = decimal_values[0] / DIVISAO
            a[1] = decimal_values[1] / DIVISAO
            a[2] = decimal_values[2] / DIVISAO
            a[3] = decimal_values[3] / DIVISAO
            a[4] = decimal_values[4] / DIVISAO
            
            # Publicar os valores no tópico 'target_joint_states'
            joint_state_msg = JointState()
            joint_state_msg.name = ['rotationalMotor1', 'rotationalMotor2', 'rotationalMotor3', 'rotationalMotor4', 'rotationalMotor5']
            joint_state_msg.position = a
            # self.publisher.publish(joint_state_msg)
        
        # print("\nData received:", separated_string)
        # print("\nData ordered", corrected_array)

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