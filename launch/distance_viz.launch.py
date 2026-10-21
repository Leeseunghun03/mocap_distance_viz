import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('mocap_distance_viz')
    config = LaunchConfiguration('config')
    rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=os.path.join(share, 'config', 'distance_viz.yaml'),
            description='Parameter file'),
        DeclareLaunchArgument('rviz', default_value='false', description='Launch RViz2'),
        DeclareLaunchArgument(
            'fake', default_value='false',
            description='Publish fake mocap data (no OptiTrack needed)'),
        Node(
            package='mocap_distance_viz',
            executable='distance_viz_node',
            name='mocap_distance_viz',
            parameters=[config],
            output='screen',
            # Required for the live terminal dashboard.
            emulate_tty=True,
            output_format='{line}',
        ),
        Node(
            package='mocap_distance_viz',
            executable='fake_mocap_publisher',
            name='fake_mocap_publisher',
            parameters=[config],
            output='screen',
            condition=IfCondition(LaunchConfiguration('fake')),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(share, 'rviz', 'distance_viz.rviz')],
            condition=IfCondition(rviz),
        ),
    ])
