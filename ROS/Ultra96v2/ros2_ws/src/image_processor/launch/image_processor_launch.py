from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_processor',
            executable='image_processor_node',
            name='image_processor_node',
            output='screen'
        ),
    ])
