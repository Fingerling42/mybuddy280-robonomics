import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from robonomicsinterface import Account, Datalog


class RobonomicsROS2Receiver(Node):

    def __init__(self):
        """
        Class for creating node, that subscribe to the topic, get address from there and
        receive the last datalog from Robonomics
        """
        super().__init__('robonomics_ros2_receiver')

        # Create subscription to topic with address
        self.subscription = self.create_subscription(
            String,
            'robonomics/address',
            self.robonomics_address_callback,
            10)
        self.subscription

        self.account_address = ''
        self.account = Account()
        self.datalog = Datalog(self.account)

        # Publisher of datalog content for received address
        self.datalog_publisher = self.create_publisher(
            String,
            'robonomics/from/datalog',
            10
        )
        self.receiver_timer = self.create_timer(
            30,
            self.receiver_from_robonomics_callback
        )  # 30 sec to publish datalog

    def robonomics_address_callback(self, msg):
        """
        Method for getting address of Robonomics account
        :param msg: String
        :return: None
        """
        self.account_address = msg.data
        self.get_logger().info('Received address: %s' % self.account_address)

    def receiver_from_robonomics_callback(self):
        """
        Method for publish last datalog content from received Robonomics address
        :return: None
        """
        if self.account_address != '':
            datalog_msg = String()
            datalog_msg.data = str(self.datalog.get_item(self.account_address))
            self.datalog_publisher.publish(datalog_msg)


def main(args=None):
    rclpy.init(args=args)

    robonomics_ros2_receiver = RobonomicsROS2Receiver()

    rclpy.spin(robonomics_ros2_receiver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robonomics_ros2_receiver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
