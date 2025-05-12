from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ocr_pkg',
            executable='ocr.py',
            name='ocr_node',
            output='screen'
        ),
        Node(
            package='vlm_pkg',
            executable='vlm.py',
            name='vlm_node',
            output='screen'
        ),
        Node(
            package='con_pkg',
            executable='con.py',
            name='con_node',
            output='screen'
        )
    ])