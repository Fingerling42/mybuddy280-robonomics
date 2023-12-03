import rclpy
from rclpy.node import Node
from pymycobot.mybuddy import MyBuddy

from mybuddy280_interfaces.msg import MyBuddy280Angles

SERIAL_PORT = "/dev/ttyAMA0"
BAUD_RATE = 115200

class MyBuddy280ROSWrapper(Node):
    """
    A class that wraps pymycobot functions of the myBuddy 280 two-handed robot to ROS 2
    """

    def __init__(self):
        """
        Init communication with robot and simple publisher
        """
        super().__init__("mybuddy280_ros2_wrapper")

        self.mc = MyBuddy(SERIAL_PORT, BAUD_RATE)

        self.publisher_angles = self.create_publisher(MyBuddy280Angles, 'myBuddy280/joints/angles', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.get_angles)

    def get_angles(self):
        """
        Get angles of all joints
        :return: MyBuddy280Angles
        """
        angles_msg = MyBuddy280Angles()

        angles_msg.left_arm.name = [
            "LJ1",
            "LJ2",
            "LJ3",
            "LJ4",
            "LJ5",
            "LJ6",
        ]

        angles_msg.left_arm.velocity = []
        angles_msg.left_arm.effort = []
        angles = self.mc.get_angles(1)
        angles_msg.left_arm.position = [float(position) for position in angles]
        angles_msg.left_arm.header.stamp = self.get_clock().now().to_msg()

        angles_msg.right_arm.name = [
            "RJ1",
            "RJ2",
            "RJ3",
            "RJ4",
            "RJ5",
            "RJ6",
        ]

        angles_msg.right_arm.velocity = []
        angles_msg.right_arm.effort = []
        angles = self.mc.get_angles(2)
        angles_msg.right_arm.position = [float(position) for position in angles]
        angles_msg.right_arm.header.stamp = self.get_clock().now().to_msg()

        angles_msg.waist.name = [
            "W"
        ]

        angles_msg.waist.velocity = []
        angles_msg.waist.effort = []
        angles_msg.waist.position = float(self.mc.get_angle(3, 1))
        angles_msg.waist.header.stamp = self.get_clock().now().to_msg()

        self.publisher_angles.publish(angles_msg)


def main(args=None):
    rclpy.init(args=args)

    mybuddy280_ros_wrapper = MyBuddy280ROSWrapper()

    rclpy.spin(mybuddy280_ros_wrapper)

    mybuddy280_ros_wrapper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
