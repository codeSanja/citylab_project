from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_patrol',
            executable='direction_service_executable',
            output='screen'
        ),
        Node(
            package='robot_patrol',
            executable='patrol_with_service_node_executable',
            output='screen'
        )# ,
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     arguments=[
        #         '-d',
        #         '/home/user/ros2_ws/src/citylab_project/robot_patrol/rviz/patrol.rviz'
        #     ],
        #     output='screen'
        # )
    ])