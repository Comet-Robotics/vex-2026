#!/usr/bin/env python3

import time
import rclpy
import serial
import struct
from rclpy.node import Node
from msgs.msg import Brain, Velocity
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Header


class SerialInterface(Node):

    def __init__(self):
        super().__init__('serial_interface')

        print("Starting Serial Interface Node")

        # Publishers
        self.brain_publisher_ = self.create_publisher(Brain, 'brain', 10)
        self.odom_publisher_ = self.create_publisher(Odometry, 'odom', 10)

        # Subscriptions
        self.string_subscription_ = self.create_subscription(
            String,
            'serial_commands',
            self.string_command_callback,
            10
        )
        
        self.velocity_subscription_ = self.create_subscription(
            Velocity,
            'wheel_odometry',
            self.velocity_callback,
            10
        )

        self.ports = ['/dev/ttyACM1', '/dev/ttyACM2']
        self.baudrate = 115200

        self.connect_serial()
                
        self.timer = self.create_timer(0.01, self.serial_loop)
    
    def from_hex(self, hex_str):
        """Convert hex string to bytes."""
        return bytes.fromhex(hex_str)
    
    def parse_teleop_data(self, data_bytes):
        """
        Parse data according to teleop.cpp format:
        Bytes 0-3:   leftX (int, big-endian)
        Bytes 4-7:   leftY (int, big-endian)
        Bytes 8-11:  rightX (int, big-endian)
        Bytes 12-15: rightY (int, big-endian)
        Bytes 16-31: leftMotors[4] (4 ints, big-endian)
        Bytes 32-47: rightMotors[4] (4 ints, big-endian)
        Bytes 48-55: imuHeading (double, big-endian)
        """
        if len(data_bytes) < 56:
            self.get_logger().warn(f"Data too short: {len(data_bytes)} bytes, expected 56")
            return None
        
        # Parse controller inputs (4 ints)
        controller_inputs = []
        for i in range(4):
            val = struct.unpack('>i', data_bytes[i*4:(i+1)*4])[0]
            controller_inputs.append(val)
        
        # Parse motor positions (8 ints)
        motor_positions = []
        for i in range(8):
            val = struct.unpack('>i', data_bytes[16 + i*4:16 + (i+1)*4])[0]
            motor_positions.append(val)
        
        # Parse IMU heading (1 double)
        imu_heading = struct.unpack('>d', data_bytes[48:56])[0]
        
        return {
            'controller': {
                'leftX': controller_inputs[0],
                'leftY': controller_inputs[1],
                'rightX': controller_inputs[2],
                'rightY': controller_inputs[3]
            },
            'motor_positions': motor_positions,
            'imu_heading': imu_heading
        }
    
    def serial_loop(self):
        if not self.ser or not self.ser.is_open:
            self.get_logger().warn("Serial port not open. Retrying...")
            self.connect_serial()

        try:
            # Read a line from serial (hex string)
            line = self.ser.readline().decode("utf-8", "replace").strip()
            if "sout" in line:
                line = line.split("sout")[-1].strip()
                
            if len(line) == 0:
                return

            # Parse hex string to bytes
            try:
                data_bytes = self.from_hex(line)
            except ValueError as e:
                self.get_logger().warn(f"Invalid hex string: {line[:50]}...")
                return

            # Parse teleop data
            parsed_data = self.parse_teleop_data(data_bytes)
            if parsed_data is None:
                return

            # Publish Brain message for wheel_odom
            brain_msg = Brain()
            brain_msg.header = Header()
            brain_msg.header.stamp = self.get_clock().now().to_msg()
            brain_msg.header.frame_id = "base_link"
            
            # Calculate average positions for left and right motors
            # Motors 0-3 are left, motors 4-7 are right
            left_positions = parsed_data['motor_positions'][:4]
            right_positions = parsed_data['motor_positions'][4:8]
            
            brain_msg.left_pos = float(sum(left_positions)) / len(left_positions) if left_positions else 0.0
            brain_msg.right_pos = float(sum(right_positions)) / len(right_positions) if right_positions else 0.0
            brain_msg.w = float(parsed_data['imu_heading'])
            
            self.brain_publisher_.publish(brain_msg)

        except serial.SerialException as e:
            self.get_logger().error(f"Serial exception: {e}")
            self.connect_serial()  # if serial disconnects somehow, probably the weird tty1->tty2 switch

        except Exception as e:
            self.get_logger().error(f"Error in serial loop: {e}")

    def velocity_callback(self, msg):
        """
        Convert Velocity message to nav_msgs Odometry for particle_filter.
        """
        odom_msg = Odometry()
        odom_msg.header = msg.header
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        
        # Set velocities in twist
        odom_msg.twist.twist.linear.x = float(msg.vx)
        odom_msg.twist.twist.linear.y = float(msg.vy)
        odom_msg.twist.twist.angular.z = float(msg.w)
        
        # Publish to odom topic for particle_filter
        self.odom_publisher_.publish(odom_msg)

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
