import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
import time

class HzMonitor(Node):
    def __init__(self):
        super().__init__('hz_monitor')
        self.pub = self.create_publisher(Float32, 'scan_hz', 10)
        self.last_time = None
        self.sub = self.create_subscription(
            LaserScan, 'scan', self.callback, 10
        )

    def callback(self, msg):
        now = time.time()
        if self.last_time is not None:
            hz = 1.0 / (now - self.last_time)
            self.pub.publish(Float32(data=hz))
        self.last_time = now

def main(args=None):
    rclpy.init(args=args)
    node = HzMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
