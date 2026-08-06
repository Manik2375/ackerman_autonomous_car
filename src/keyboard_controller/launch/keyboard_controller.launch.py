from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='keyboard_controller',
            executable='keyboard_controller_node',
            name='keyboard_controller',
            output='screen',
            emulate_tty=True,
        ),
    ])
