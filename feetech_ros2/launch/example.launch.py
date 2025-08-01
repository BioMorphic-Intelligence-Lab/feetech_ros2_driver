import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the parameter file
    param_file = os.path.join(get_package_share_directory('feetech_ros2'), 'config', 'example_params.yaml')

    return LaunchDescription([
        Node(
            package='feetech_ros2',
            executable='feetech_ros2_interface',
            name='feetech_ros2_interface',
            output='screen',
            parameters=[param_file]
        )
    ])
