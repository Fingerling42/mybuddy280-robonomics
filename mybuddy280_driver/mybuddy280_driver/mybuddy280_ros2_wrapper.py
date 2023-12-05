import rclpy
from rclpy.node import Node
from pymycobot.mybuddy import MyBuddy

from mybuddy280_interfaces.msg import MyBuddy280Angles
from mybuddy280_interfaces.srv import MyBuddy280GetAngle

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
        self.publisher_joint_state = self.create_publisher(
            MyBuddy280Angles,
            'myBuddy280/joints/angles',
            10
        )
        self.timer_state = self.create_timer(
            0.5,
            self.joint_state_callback
        )  # 0.5 sec for msg

        # Service for sending angles to joints
        self.srv_send_joint_angle = self.create_service(
            MyBuddy280GetAngle,
            'myBuddy280/send_joint_angle',
            self.send_joint_angle_callback
        )

    def joint_state_callback(self):
        """
        Get states of all joints
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

    def send_joint_angle_callback(self, request, response):
        """
        Send angles to the one of robot's part
        :param request: part_id ('L' for left arm, 'R' for right arm, 'W' for waist)
                        joint_number[] --- 1 to 6
                        angle[] --- -165 — 165 (-175 — 175 for J6) degrees
                        speed[] --- joint speed 1 - 100
        :param response: status message
        :return: response
        """
        self.get_logger().info('Request for sending command to %s robot part' % request.part_id)
        if request.part_id == 'L':
            part_id = 1
        elif request.part_id == 'R':
            part_id = 2
        elif request.part_id == 'W':
            part_id = 3
        else:
            response.result = "Error: Wrong ID of robot part, use only L, R or W"
            return response

        i = 0
        for joint_number in request.joint_number:
            if joint_number in [1, 2, 3, 4, 5, 6]:
                if -175.0 <= request.angle[i] <= 175.0:
                    if 1 <= request.speed[i] <= 100:
                        self.mc.send_angle(part_id, joint_number, request.angle[i], request.speed[i])
                        i += 1
                    else:
                        response.result = "Error: Speed is out of range (1 .. 100)"
                        return response
                else:
                    response.result = "Error: Angle is out of range (-175 .. 175)"
                    return response
            else:
                response.result = "Error: Wrong joint number (only 1 .. 6)"
                return response

        response.result = "Success: Angles sent"
        return response


def main(args=None):
    rclpy.init(args=args)

    mybuddy280_ros_wrapper = MyBuddy280ROSWrapper()

    rclpy.spin(mybuddy280_ros_wrapper)

    mybuddy280_ros_wrapper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
