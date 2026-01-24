#!/usr/bin/env python3

import time
import rclpy
import serial
import struct
from rclpy.node import Node
from msgs.msg import Brain, Velocity
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Header
import constants


class SerialInterface(Node):

    def __init__(self):
        super().__init__('serial_interface')

        print("Starting Serial Interface Node")

        # Publishers
        self.brain_publisher_ = self.create_publisher(Brain, 'brain', 10)
        self.odom_publisher_ = self.create_publisher(Odometry, 'odom', 10)
        self.serial_command_publisher_ = self.create_publisher(String, 'serial_commands', 10)

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

        self.ports = ['/dev/vexbrain']
        self.baudrate = 115200

        self.connect_serial()
                
        self.timer = self.create_timer(0.01, self.serial_loop)
    
    def serial_loop(self):
        if not self.ser or not self.ser.is_open:
            self.get_logger().warn("Serial port not open. Retrying...")
            self.connect_serial()

        try:
            # self.get_logger().info("Reading from serial...")
            # print("Reading from serial...")
            # Read a line from serial (hex string)
            line = self.ser.readline().decode("utf-8", "replace").strip()

            self.get_logger().info(f"Raw line: {line}")

            if "sout" in line:
                line = line.split("sout")[-1].strip()
                
            if len(line) == 0:
                return

            self.get_logger().warn(f"Got data: {line}")

            parts = line.split(',')
            if len(parts) != 13: # FIND SOME WAY TO MAKE THIS DYNAMIC
                self.get_logger().warn(f"Bad packet length: {len(parts)} → {line}")
                return
            
            values = [int(p) for p in parts]

            lx, ly, rx, ry = values[0:4]
            motor_vels = values[4:12]
            imu_scaled = values[12]
            imu_heading = imu_scaled / 100.0  # degrees

            # Publish Brain message for wheel_odom
            brain_msg = Brain()
            brain_msg.header = Header()
            brain_msg.header.stamp = self.get_clock().now().to_msg()
            brain_msg.header.frame_id = "base_link"
            
            # Calculate average positions for left and right motors
            # Motors 0-3 are left, motors 4-7 are right
            left_velocities = motor_vels[:4]
            right_velocities = motor_vels[4:8]

            brain_msg.left_vel = float(sum(left_velocities)) / len(left_velocities) if left_velocities else 0.0
            brain_msg.right_vel = float(sum(right_velocities)) / len(right_velocities) if right_velocities else 0.0
            brain_msg.theta = float(imu_heading)

            # rot/min * in/rot = in/min * 1 min/60 sec = in/sec * 1 foot/12 in = ft/sec
            brain_msg.left_vel = brain_msg.left_vel * 2 * 3.14159 * constants.WHEEL_RADIUS / (12.0 * 60.0)  # assuming wheel radius 3.25 / 2 inches
            brain_msg.right_vel = brain_msg.right_vel * 2 * 3.14159 * constants.WHEEL_RADIUS / (12.0 * 60.0)
            
            brain_msg.left_vel = brain_msg.left_vel * constants.DRIVETRAIN_GEAR_RATIO
            brain_msg.right_vel = brain_msg.right_vel * constants.DRIVETRAIN_GEAR_RATIO

            self.get_logger().info(f"Left Vel: {brain_msg.left_vel:.2f} ft/s, Right Vel: {brain_msg.right_vel:.2f} ft/s, Heading: {brain_msg.theta:.2f} deg",)
            self.brain_publisher_.publish(brain_msg)

            # Controller inputs - TEMPORARY TESTING
            self.get_logger().info(f"Controller LX: {lx}, LY: {ly}, RX: {rx}, RY: {ry}")
            left_vel = (ly + rx) * (12000.0 / 127.0)  # scale to -12000 to 12000
            right_vel = (ly - rx) * (12000.0 / 127.0)
            voltages = [int(left_vel)] * 4 + [int(right_vel)] * 4
            command_str = ",".join(str(v) for v in voltages)
            self.get_logger().info(f"Sending command to teleop: {command_str}")
            msg = String()
            msg.data = command_str
            self.serial_command_publisher_.publish(msg)

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
        odom_msg.pose.pose.orientation.z = float(msg.theta) 
        
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
