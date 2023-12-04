import rclpy
from rclpy.node import Node
from pymycobot.mybuddy import MyBuddy

from mybuddy280_interfaces.msg import MyBuddy280Angles

SERIAL_PORT = "/dev/ttyAMA0"
BAUD_RATE = 115200


class MyBuddy280ROSWrapper(Node):
    """
    A class that wraps pymycobot functions of the myBuddy280 to ROS 2
    """

    def __init__(self):
        """
        Init communication with robot and all publisher / subscriber and services
        """
        super().__init__("mybuddy280_ros2_wrapper")

        # Start connecting to robot
        self.mc = MyBuddy(SERIAL_PORT, BAUD_RATE)

        # Publisher node of joint states (position)
        self.publisher_joint_state = self.create_publisher(MyBuddy280Angles, 'myBuddy280/joints/angles', 10)
        self.timer_state = self.create_timer(0.5, self.joint_state_callback)  # 0.5 sec for msg

    def joint_state_callback(self):
        """
        Get states of all joints
        :return: MyBuddy280Angles
        """
        state_msg = MyBuddy280Angles()

        # Left arm state
        state_msg.left_arm.name = [
            "LJ1",
            "LJ2",
            "LJ3",
            "LJ4",
            "LJ5",
            "LJ6",
        ]
        state_msg.left_arm.velocity = []
        state_msg.left_arm.effort = []
        angles = self.mc.get_angles(1)
        state_msg.left_arm.position = [float(position) for position in angles]
        state_msg.left_arm.header.stamp = self.get_clock().now().to_msg()

        # Right arm state
        state_msg.right_arm.name = [
            "RJ1",
            "RJ2",
            "RJ3",
            "RJ4",
            "RJ5",
            "RJ6",
        ]
        state_msg.right_arm.velocity = []
        state_msg.right_arm.effort = []
        angles = self.mc.get_angles(2)
        state_msg.right_arm.position = [float(position) for position in angles]
        state_msg.right_arm.header.stamp = self.get_clock().now().to_msg()

        # Waist state
        state_msg.waist.name = [
            "W"
        ]
        state_msg.waist.velocity = []
        state_msg.waist.effort = []
        angles = float(self.mc.get_angle(3, 1))
        state_msg.waist.position.append(angles)
        state_msg.waist.header.stamp = self.get_clock().now().to_msg()

        self.publisher_joint_state.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)

    mybuddy280_ros_wrapper = MyBuddy280ROSWrapper()

    rclpy.spin(mybuddy280_ros_wrapper)

    mybuddy280_ros_wrapper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
