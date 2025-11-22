#!/usr/bin/env python3

import time
import rclpy
import serial
from rclpy.node import Node
from msgs.msg import SensorMsg
import json

from std_msgs.msg import String


class SerialInterface(Node):

    def __init__(self):
        super().__init__('serial_interface')

        print("Starting Serial Interface Node")

        # Publishers
        self.string_publisher_ = self.create_publisher(String, 'serial_output', 10)

        self.string_subscription_ = self.create_subscription(
            String,
            'serial_commands',
            self.string_command_callback,
            10
        )

        self.ports = ['/dev/ttyACM1', '/dev/ttyACM2']
        self.baudrate = 115200

        self.connect_serial()
                
        self.timer = self.create_timer(0.01, self.serial_loop)
    
    def serial_loop(self):
        if not self.ser or not self.ser.is_open:
            self.get_logger().warn("Serial port not open. Retrying...")
            self.connect_serial()

        try:
            # reads a line from serial and takes only ascii characters
            line = self.ser.readline().decode("utf-8", "replace").strip()
            if "sout" in line:
                line = line.split("sout")[-1].strip()
                
            if len(line) == 0:
                return

            # line = "".join(c for c in line if ord(c) < 128 and ord(c) > 0)


            self.get_logger().info(f"Read line from serial: {line}")

            msg = String()
            msg.data = line
            self.string_publisher_.publish(msg)
            self.get_logger().info(f"Published serial output: {line}")

        except serial.SerialException as e:
            self.get_logger().error(f"Serial exception: {e}")
            self.connect_serial()  # if serial disconnects somehow, probably the weird tty1->tty2 switch

        except Exception as e:
            self.get_logger().error(f"Error in serial loop: {e}")

    def string_command_callback(self, msg):
        self.get_logger().info(f"Received serial command: {msg.data}")
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((msg.data + "\n").encode())
                self.get_logger().info(f"Sent string command to serial: {msg.data}")
            except Exception as e:
                self.get_logger().error(f"Failed to send string command to serial: {e}")
        else:
            self.get_logger().warn("Serial port not open. Cannot send string command.")

    def connect_serial(self):
        connected = False
        i = 0
        while not connected:
            for port in self.ports:
                if i % 6767 == 0:
                    self.get_logger().info(f"Trying port: {port}")
                try:
                    self.ser = serial.Serial(port, self.baudrate, timeout=0.01)
                    self.get_logger().info(f"Connected to serial port {port} at {self.baudrate} baud.")
                    connected = True
                    break
                except serial.SerialException as e:
                    if i % 6767 == 0:
                        self.get_logger().error(f"Failed to connect to serial port {port}: {e}")
                    i += 1

def main(args=None):
    rclpy.init(args=args)
    serial_interface_node = SerialInterface()
    try:
        rclpy.spin(serial_interface_node)
    except KeyboardInterrupt:
        pass
    finally:
        if serial_interface_node.ser and serial_interface_node.ser.is_open:
            serial_interface_node.ser.close()
        serial_interface_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
