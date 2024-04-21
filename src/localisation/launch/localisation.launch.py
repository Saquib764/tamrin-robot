import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='localisation',  # Replace with your package name
            executable='detect_aruco',  # Replace with your node's Python script
            name='detect_aruco',
            output='screen'
        )
    ])

if __name__ == '__main__':
    generate_launch_description()

