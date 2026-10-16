from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_patrol',
            executable='gotopose_action_node_executable',
            output='screen'
        ),
    ])