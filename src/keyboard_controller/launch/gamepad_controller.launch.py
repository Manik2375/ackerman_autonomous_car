from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='keyboard_controller',
            executable='gamepad_controller_node',
            name='gamepad_controller',
            output='screen',
        ),
    ])
