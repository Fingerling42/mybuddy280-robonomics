import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from robonomicsinterface import Account, Datalog
from substrateinterface import KeypairType

class RobonomicsROS2Sender(Node):

    def __init__(self):
        """
        Class for creating node, that subscribe to the topic and send it data to Robonomics datalog
        """
        super().__init__('robonomics_ros2_sender')

        # Declare used parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('seed', rclpy.Parameter.Type.STRING),
                ('crypto_type', rclpy.Parameter.Type.STRING),
            ]
        )

        # Create subscription to topic
        self.subscription = self.create_subscription(
            String,
            'robonomics/to/datalog',
            self.send_to_robonomics_callback,
            10)
        self.subscription

        # Get used parameters for account creation
        account_seed = self.get_parameter('seed')
        account_type = self.get_parameter('crypto_type')

        # Checking the type of account and creating it
        if account_type.value == 'ED25519':
            crypto_type = KeypairType.ED25519
        elif account_type.value == 'SR25519':
            crypto_type = KeypairType.SR25519
        else:
            crypto_type = -1
        self.account = Account(seed=account_seed.value, crypto_type=crypto_type)
        self.datalog = Datalog(self.account)

    def send_to_robonomics_callback(self, msg):
        """
        Method that happened if the msg appears in topic
        :param msg: String
        :return: None
        """
        data = msg.data
        self.datalog.record(data)
        self.get_logger().info('Sent msg to datalog: %s' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    robonomics_ros2_sender = RobonomicsROS2Sender()

    rclpy.spin(robonomics_ros2_sender)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robonomics_ros2_sender.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
