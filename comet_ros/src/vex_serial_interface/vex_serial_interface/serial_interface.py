#!/usr/bin/env python3

import rclpy
import serial # type: ignore
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist


class SerialInterface(Node):

    def __init__(self):
        super().__init__('serial_interface')

        print("Starting Serial Interface Node")

        # Publishers
        self.twist_publisher_ = self.create_publisher(Twist, 'robot_twist', 10)

        # Subscriptions
        self.pose_subscription_ = self.create_subscription(
            Pose2D,
            'pf_pose_geometry',
            self.pose_to_serial,
            10
        )

        self.ports = ['/dev/vexbrain']
        self.baudrate = 115200

        self.connect_serial()
                
        self.timer = self.create_timer(0.01, self.serial_loop)
    
    def serial_loop(self):
        if not self.ser or not self.ser.is_open:
            self.get_logger().warn("Serial port not open. Retrying...")
            self.connect_serial()

        try:
            # Read a line from serial (hex string)
            line = self.ser.readline().decode("utf-8", "replace").strip()

            self.get_logger().info(f"Raw line: {line}")

            if "sout" in line:
                line = line.split("sout")[-1].strip()
                
            if len(line) == 0:
                return

            self.get_logger().warn(f"Got data: {line}")

            parts = line.split(',')
            if len(parts) != 3: # FIND SOME WAY TO MAKE THIS DYNAMIC
                self.get_logger().warn(f"Bad packet length: {len(parts)} → {line}")
                return
            
            values = [int(p) for p in parts]

            # Get twist values from array and publish
            twistMsg = Twist()
            twistMsg.linear.x = values[0]
            twistMsg.linear.y = values[1]
            twistMsg.angular.z = values[2]
            self.twist_publisher_.publish(twistMsg)

        except serial.SerialException as e:
            self.get_logger().error(f"Serial exception: {e}")
            self.connect_serial()  # if serial disconnects somehow, probably the weird tty1->tty2 switch

        except Exception as e:
            self.get_logger().error(f"Error in serial loop: {e}")

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

    def pose_to_serial(self, msg: Pose2D):
        serial_str = str(msg.x) + "," + str(msg.y) + "," + str(msg.theta)
        self.send_to_serial(serial_str)

    def send_to_serial(self, str: str):
        '''
        Sends string to serial. Note that a "\\n" is automatically appended to the string.

        Args:
            str: The string to send
        '''
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((str + "\n").encode())
                self.get_logger().info(f"Sent string command to serial: {str}")
            except Exception as e:
                self.get_logger().error(f"Failed to send string command to serial: {e}")
        else:
            self.get_logger().warn("Serial port not open. Cannot send string command.")

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
