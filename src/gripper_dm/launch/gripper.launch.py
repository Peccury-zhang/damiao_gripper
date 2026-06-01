import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('gripper_dm'),
        'config',
        'gripper.yaml'
    )

    return LaunchDescription([
        Node(
            package='gripper_dm',
            executable='gripper_node',
            name='gripper_node',
            parameters=[config],
            output='screen',
        )
    ])
