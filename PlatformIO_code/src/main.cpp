#include <Arduino.h>
#include <HardwareSerial.h>
#include <sstream>
#include <cmath> 
#include <iostream>
#include <string>

// Set LED_BUILTIN if it is not defined by Arduino framework
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

HardwareSerial SerialPort_PC(0);
HardwareSerial SerialPort(2);

void set_position(double *array, int size)
{
    byte msgs[5*2]; // each position message requires 2 bytes, a lower and a high 

    for (int i = 0; i < 5; i++)
    {
        
      array[i] = 500 + array[i] * 4;

      int numero = std::floor(array[i]);

      // Using binary operations to extract the two bytes, the lower and the higher values
      msgs[2*i] =  numero & 0x000000ff;
      msgs[2*i + 1] = (numero & 0x0000ff00) >> 8;
    }
    byte message[] = {
       85, 85, 20, 3, 5, 32, 3, 1, 51, 2, 2, 167, 2, 3, 51, 2, 4, 182, 0, 5, 4, 3
    };

    SerialPort.write(message, sizeof(message));
}

void setup()
{
  // initialize LED digital pin as an output.
  //pinMode(LED_BUILTIN, OUTPUT);
  SerialPort_PC.begin(9600);
  SerialPort.begin(9600);

  
  /*// // initialize LED digital pin as an output.
  double array[5] = {10.0, -45.6, 20.0, 40.0, 0.0};
  int size = 5;

  //set_position(array, 5);*/
}

void loop()
{

  if (SerialPort_PC.available()) {
    // Read bytes from UART0
    while (SerialPort_PC.available()) {
      char byteRead = SerialPort_PC.read();
      // Write bytes to UART2
      SerialPort.write(byteRead);
    }
  }

  if (SerialPort.available()) {
    // Read bytes from UART0
    while (SerialPort.available()) {
      char byteRead = SerialPort.read();
      // Write bytes to UART2
      SerialPort_PC.write(byteRead);
    }
  }
  /* Test messages */
  // byte message3[] = {0x55, 0x55, 0x14, 0x03, 0x05, 0x90, 0x01, 0x01, 0x94, 0x02, 0x02, 0x8C, 0x00, 0x03, 0xF4, 0x01, 0x04, 0x5C, 0x03, 0x05, 0xF4, 0x01};  
  // SerialPort.write(message3, sizeof(message3));

  // delay(1000);

  // byte message4[] = {0x55, 0x55, 0x14, 0x03, 0x05, 0x20, 0x03, 0x01, 0xBC, 0x02, 0x02, 0x5C, 0x03, 0x03, 0x8C, 0x00, 0x04, 0x7C, 0x01, 0x05, 0xA8, 0x02};  
  // SerialPort.write(message4, sizeof(message4));

  // delay(1000);

  // byte message5[] = {85, 85, 20, 3, 5, 32, 3, 1, 73, 2, 2, 229, 1, 3, 113, 1, 4, 43, 2, 5, 77, 2};  
  // SerialPort.write(message5, sizeof(message5));

  // delay(1000);

  // byte message6[] = {0x55, 0x55, 0x14, 0x03, 0x05, 0x20, 0x03, 0x01, 0xF4, 0x01, 0x02, 0xF4, 0x01, 0x03, 0xF4, 0x01, 0x04, 0xF4, 0x01, 0x05, 0xF4, 0x01};  
  // SerialPort.write(message6, sizeof(message6));

  // delay(1000);

  //  byte message7[] = {0x55, 0x55, 0x14, 0x03, 0x05, 0x20, 0x03, 0x01, 0x8C, 0x00, 0x02, 0x5C, 0x03, 0x03, 0xF4, 0x01, 0x04, 0x8C, 0x00, 0x05, 0xF4, 0x01};  
  // SerialPort.write(message7, sizeof(message7));

  // delay(1000);

  
  // byte message8[] = {0x55, 0x55, 0x14, 0x03, 0x05, 0x20, 0x03, 0x01, 0x8C, 0x00, 0x02, 0xA8, 0x02, 0x03, 0x5C, 0x03, 0x04, 0xF4, 0x01, 0x05, 0x8C, 0x00};  
  // SerialPort.write(message8, sizeof(message8));
 
  // delay(1000);

  //  byte message9[] = {0x55, 0x55, 0x14, 0x03, 0x05, 0x20, 0x03, 0x01, 0x8C, 0x00, 0x02, 0x8C, 0x00, 0x03, 0xF4, 0x01, 0x04, 0x5C, 0x03, 0x05, 0xF4, 0x01};  
  // SerialPort.write(message9, sizeof(message9));

    /*
    // turn the LED on (HIGH is the voltage level)
    digitalWrite(LED_BUILTIN, HIGH);
    // wait for a second
    delay(500);
    // turn the LED off by making the voltage LOW
    digitalWrite(LED_BUILTIN, LOW);
    // wait for a second
    delay(500);*/
}