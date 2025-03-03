from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    share_dir = get_package_share_directory("imu_utils")
    data_path = os.path.join(share_dir, "data")

    if not os.path.exists(data_path):
        os.mkdir(data_path)

    return LaunchDescription(
        [
            Node(
                package="imu_utils",
                executable="imu_an",
                name="imu_utils",
                parameters=[
                    {
                        "imu_topic": "/imu",
                        "imu_name": "xsens",
                        "bag_file": "/path/to/rosbag2.db3",
                        "data_save_path": data_path,
                        "max_cluster": 200,
                    }
                ],
                # arguments=['--ros-args', '--log-level', 'debug'],
                output="screen",
            ),
        ]
    )
