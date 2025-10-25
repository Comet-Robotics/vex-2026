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
        self.publisher_ = self.create_publisher(SensorMsg, 'sensors', 10)
        self.string_publisher_ = self.create_publisher(String, 'serial_output', 10)

        # Subscribers
        self.subscription_ = self.create_subscription(
            String,
            'motor_commands',
            self.command_callback,
            10
        )
        self.string_subscription_ = self.create_subscription(
            String,
            'serial_commands',
            self.string_command_callback,
            10
        )
        
        self.declare_parameter('port', '/dev/ttyACM2')
        self.declare_parameter('baudrate', 115200)

        self.connect_serial()
                
        self.timer = self.create_timer(0.01, self.serial_loop)
    
    def serial_loop(self):
        if not self.ser or not self.ser.is_open:
            self.get_logger().warn("Serial port not open. Retrying...")
            self.connect_serial()

        try:
            # reads a line from serial and takes only ascii characters
            line = self.ser.readline().decode("utf-8", "replace").strip()
            line = "".join(c for c in line if ord(c) < 128 and ord(c) > 0)

            if len(line) == 0:
                return

            self.get_logger().info(f"Read line from serial: {line}")

            # get everything after sout
            if "sout" in line:
                line = line.split("sout")[-1].strip()

            msg = String()
            msg.data = line
            self.string_publisher_.publish(msg)
            self.get_logger().info(f"Published serial output: {line}")

        except Exception as e:
            self.get_logger().error(f"Error in serial loop: {e}")

    def command_callback(self, msg):
        self.get_logger().info(f"Received motor command: {msg.data}")
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((msg.data).encode())
                self.get_logger().info(f"Sent command to serial: {msg.data}")
            except Exception as e:
                self.get_logger().error(f"Failed to send command to serial: {e}")   
        else:
            self.get_logger().warn("Serial port not open. Cannot send command.")

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
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        connected = False
        i = 0
        while not connected:
            try:
                self.ser = serial.Serial(port, baudrate, timeout=0.01)
                self.get_logger().info(f"Connected to serial port {port} at {baudrate} baud.")
                connected = True
            except serial.SerialException as e:
                if i % 10 == 0:
                    self.get_logger().error(f"Failed to connect to serial port {port}: {e}")
                self.ser = None
                time.sleep(0.1)
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


'''
import serial
import json
import time
 
def service_serial():
    port = "/dev/ttyACM1"
    baud = 115200
    ser = serial.Serial(port, baud, timeout=None)
 
    while True:
        try:
            if not ser:
                ser = serial.Serial(port, baud, timeout=None)
            line = ser.readline().decode("utf-8", "replace").strip()
            line = "".join(c for c in line if ord(c) < 128 and ord(c) > 0)
            line = line[5:]

            print(line)
            if len(line) < 8 :
                print("line too short, continuing with next")
                continue

            try:
                r1 = line[3]
                r2 = line[7]

                print("attempting parse - r1:", r1, "r2:", r2)

                ser.write((str((int(r1) - int(r2)) * 6000) + "\n").encode())

            except Exception as e:
                print("encountered error with line, continuing with next")
                print(e)
                continue
            time.sleep(1.0 / 50.0)  
        except serial.SerialException as e:
            print("Encountered error in serial loop:", e)
            del ser
            time.sleep(1.0)

service_serial()
''' 