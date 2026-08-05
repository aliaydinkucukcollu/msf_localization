import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('msf_localization_ros2')
    default_config = os.path.join(pkg_share, 'config', 'params.yaml')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Package parameters.',
    )

    reference_node = Node(
        package='msf_localization_ros2',
        executable='msf_localization_ros2',
        name='msf_localization_ros2_node',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('config_file')],
    )

    return LaunchDescription([
        config_file_arg,
        reference_node,
    ])
