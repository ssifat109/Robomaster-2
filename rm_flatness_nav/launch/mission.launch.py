from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim = LaunchConfiguration('sim')
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='true'),
        DeclareLaunchArgument('output_dir', default_value='data/simulation'),
        DeclareLaunchArgument('config_file', default_value=''),
        Node(package='rm_flatness_nav', executable='logger', output='screen',
             parameters=[{'output_dir': LaunchConfiguration('output_dir')}]),
        Node(package='rm_flatness_nav', executable='planner', output='screen'),
        Node(package='rm_flatness_nav', executable='simulator', condition=IfCondition(sim)),
        Node(package='rm_flatness_nav', executable='driver', output='screen', condition=UnlessCondition(sim),
             parameters=[{'config_file': LaunchConfiguration('config_file')}],
             on_exit=[Shutdown(reason='Hardware driver exited')]),
    ])
