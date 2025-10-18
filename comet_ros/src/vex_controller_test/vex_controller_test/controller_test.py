import rclpy
from rclpy.node import Node
from msgs.msg import SensorMsg

from std_msgs.msg import String


class ControllerTest(Node):

    def __init__(self):
        super().__init__('controller_test')

        # Publishers
        self.publisher_ = self.create_publisher(String, 'motor_commands', 10)

        # Subscriptions
        self.subscription_ = self.create_subscription(
            SensorMsg,
            'sensors',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "r1: %d, r2: %d"' % (msg.r1, msg.r2))
        r1 = msg.r1
        r2 = msg.r2
        command_data = (str((int(r1) - int(r2)) * 6000) + "\n")

        command = String()
        command.data = command_data
        self.publisher_.publish(command)


def main(args=None):
    rclpy.init(args=args)

    controller_test = ControllerTest()

    rclpy.spin(controller_test)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()