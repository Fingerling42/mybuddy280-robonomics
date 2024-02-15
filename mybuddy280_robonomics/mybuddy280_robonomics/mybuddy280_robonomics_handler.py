import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String
from mybuddy280_interfaces.msg import MyBuddy280Angles
from mybuddy280_interfaces.srv import MyBuddy280SendAngles

from robonomics_ros2_interfaces.srv import DownloadFromIPFS, UploadToIPFS, RobonomicsROS2SendDatalog

import json


class MyBuddy280Robonomics(Node):

    def __init__(self):
        """
        Class for execution of Robonomics function with myByddy280 ROS 2 package
        """
        super().__init__('mybuddy280_robonomics_handler')

        # Names of files and core IPFS dir
        self.srv_send_joint_angle_file_name = 'mybuddy280_srv_send_joints_angles.json'
        self.joints_angles_file_name = 'mybuddy280_joints_angles.json'
        self.ipfs_dir = 'ipfs_files'

        # Callback groups that prohibits async calls of callback functions
        launch_callback_group = MutuallyExclusiveCallbackGroup()
        datalog_callback_group = MutuallyExclusiveCallbackGroup()

        # Subscription for myBuddy280 joint angles data
        self.mybuddy280_joints_angles = MyBuddy280Angles()
        self.subscriber_joints_angles = self.create_subscription(
            MyBuddy280Angles,
            'myBuddy280/joints/angles',
            self.subscriber_joints_angles_callback,
            qos_profile=qos_profile_sensor_data,
        )
        self.subscriber_joints_angles  # prevent unused variable warning

        # Subscription for launch params
        self.subscriber_launch_param = self.create_subscription(
            String,
            'robonomics/launch_param',
            self.subscriber_launch_param_callback,
            10,
        )
        self.subscriber_launch_param  # prevent unused variable warning

        # Creating service clients for IPFS handler
        self.ipfs_download_client = self.create_client(
            DownloadFromIPFS,
            'ipfs/download',
            callback_group=launch_callback_group,
        )
        while not self.ipfs_download_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('IPFS handler service not available, waiting again...')

        self.ipfs_upload_client = self.create_client(
            UploadToIPFS,
            'ipfs/upload',
            callback_group=datalog_callback_group,
        )
        while not self.ipfs_upload_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('IPFS handler service not available, waiting again...')

        # Client for sending datalogs
        self.send_datalog_client = self.create_client(
            RobonomicsROS2SendDatalog,
            'robonomics/send_datalog',
            callback_group=datalog_callback_group,
        )

        # Client for sending angles to robot
        self.send_joint_angle_client = self.create_client(
            MyBuddy280SendAngles,
            'myBuddy280/send_joint_angles',
            callback_group=launch_callback_group,
        )
        while not self.send_joint_angle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('myBuddy 280 service for sending angles is not available, waiting again...')

        # Timer for publishing datalogs with joint angles
        self.timer_joints_angles = self.create_timer(
            60,
            self.timer_joints_angles_callback,
        )

    def subscriber_joints_angles_callback(self, msg):
        """
        Method for receiving joints angles msgs from robot
        :param msg: msg with mybuddy280_interfaces/msg/MyBuddy280Angles type
        :return: None
        """
        self.mybuddy280_joints_angles = msg

    def subscriber_launch_param_callback(self, msg):
        """
        Method that downloads IPFS file and publishes it content
        :param msg: string with IPFS cid
        :return: None
        """
        cid = msg.data
        # Send request to IPFS service
        response = self.ipfs_download_request(cid, self.srv_send_joint_angle_file_name)
        self.get_logger().info(response.result)

        # Open file from IPFS
        file = open(get_package_share_directory('ipfs_handler') + "/" + self.ipfs_dir
                    + "/" + self.srv_send_joint_angle_file_name)
        data = json.load(file)

        # Preparing request
        request = MyBuddy280SendAngles.Request()
        request.part_id = str(data['part_id'])
        request.joint_number = []
        request.angle = []
        request.speed = []
        for i in range(0, len(data['joint_number'])):
            request.joint_number.append(int(data['joint_number'][i]))
            request.angle.append(float(data['angle'][i]))
            request.speed.append(int(data['speed'][i]))

        # Making request
        future = self.send_joint_angle_client.call_async(request)
        self.executor.spin_until_future_complete(future)

        self.get_logger().info(future.result().result)

    def send_datalog_request(self, datalog_content):
        """
        Request function to send datalog
        :param datalog_content: string with data
        :return: result
        """
        request = RobonomicsROS2SendDatalog.Request()
        request.datalog_content = datalog_content
        future = self.send_datalog_client.call_async(request)
        self.executor.spin_until_future_complete(future)

        return future.result()

    def ipfs_download_request(self, cid, file_name):
        """
        Request function to download IPFS file
        :param cid: IPFS cid
        :param file_name: name for file
        :return: result
        """
        # Preparing request
        request = DownloadFromIPFS.Request()
        request.cid = cid
        request.file_name = file_name
        # Make request and wait for its execution
        future = self.ipfs_download_client.call_async(request)
        self.executor.spin_until_future_complete(future)

        return future.result()

    def ipfs_upload_request(self, file_name):
        """
        Request function to upload IPFS file
        :param file_name: name for file
        :return: cid
        """
        request = UploadToIPFS.Request()
        request.file_name = file_name
        future = self.ipfs_upload_client.call_async(request)
        self.executor.spin_until_future_complete(future)

        return future.result()

    def timer_joints_angles_callback(self):
        """
        Timer callback that publish datalogs with IPFS hash each timer period
        :return: None
        """
        # Preparing file
        file = open(get_package_share_directory('ipfs_handler') + "/" + self.ipfs_dir
                    + "/" + self.joints_angles_file_name, 'w')

        # Fill JSON dict with robot data
        data = {
            'timestamp': float(self.mybuddy280_joints_angles.left_arm.header.stamp.sec +
                               self.mybuddy280_joints_angles.left_arm.header.stamp.nanosec * pow(10, -9)),
            'left_arm': {
                self.mybuddy280_joints_angles.left_arm.name[0]: self.mybuddy280_joints_angles.left_arm.position[0],
                self.mybuddy280_joints_angles.left_arm.name[1]: self.mybuddy280_joints_angles.left_arm.position[1],
                self.mybuddy280_joints_angles.left_arm.name[2]: self.mybuddy280_joints_angles.left_arm.position[2],
                self.mybuddy280_joints_angles.left_arm.name[3]: self.mybuddy280_joints_angles.left_arm.position[3],
                self.mybuddy280_joints_angles.left_arm.name[4]: self.mybuddy280_joints_angles.left_arm.position[4],
                self.mybuddy280_joints_angles.left_arm.name[5]: self.mybuddy280_joints_angles.left_arm.position[5],
            },
            'right_arm': {
                self.mybuddy280_joints_angles.right_arm.name[0]: self.mybuddy280_joints_angles.right_arm.position[0],
                self.mybuddy280_joints_angles.right_arm.name[1]: self.mybuddy280_joints_angles.right_arm.position[1],
                self.mybuddy280_joints_angles.right_arm.name[2]: self.mybuddy280_joints_angles.right_arm.position[2],
                self.mybuddy280_joints_angles.right_arm.name[3]: self.mybuddy280_joints_angles.right_arm.position[3],
                self.mybuddy280_joints_angles.right_arm.name[4]: self.mybuddy280_joints_angles.right_arm.position[4],
                self.mybuddy280_joints_angles.right_arm.name[5]: self.mybuddy280_joints_angles.right_arm.position[5],
            },
            'waist': self.mybuddy280_joints_angles.waist.position[0],
        }

        # Save the JSON file
        json_object = json.dumps(data, indent=4)
        file.write(json_object)
        file.close()

        # Upload file to IPFS
        response_ipfs = self.ipfs_upload_request(self.joints_angles_file_name)

        # Send datalog
        response_datalog = self.send_datalog_request(response_ipfs.cid)
        self.get_logger().info(response_datalog.result)

    def __enter__(self):
        """
        Enter the object runtime context
        :return: object itself
        """
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """
        Exit the object runtime context
        :param exc_type: exception that caused the context to be exited
        :param exc_val: exception value
        :param exc_tb: exception traceback
        :return: None
        """


def main(args=None):
    rclpy.init(args=args)
    # Creating multithreaded executor to make proper callback orchestration
    executor = MultiThreadedExecutor()

    with MyBuddy280Robonomics() as mybuddy280_robonomics_handler:
        try:
            executor.add_node(mybuddy280_robonomics_handler)
            executor.spin()
        except KeyboardInterrupt:
            mybuddy280_robonomics_handler.get_logger().warn("Killing the mybuddy280_robonomics_handler...")
            executor.remove_node(mybuddy280_robonomics_handler)


if __name__ == '__main__':
    main()
