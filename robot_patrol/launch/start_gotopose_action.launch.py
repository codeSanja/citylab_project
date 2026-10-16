from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_patrol',
            executable='go_to_pose_action_server_executable',
            output='screen'
        ),
    ])