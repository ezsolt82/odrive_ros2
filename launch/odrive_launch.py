from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='odrive_ros2',
            executable='odrive_node',
            name='odrive_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'simulation_mode': True}
            ]
        )
    ])