import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.webots_controller import WebotsController
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory('tamrin_robot'))
    robot_description_path = os.path.join(pkg_path, 'description', 'my_robot.webots.urdf')
    robot_description_text = None
    with open(robot_description_path, 'r') as file:
        robot_description_text = file.read()

    webots = WebotsLauncher(
        world=os.path.join(pkg_path, 'worlds', 'webots_world_1.wbt')
    )

    ros2_supervisor = Ros2SupervisorLauncher()


    my_robot_driver = WebotsController(
        robot_name='my_robot',
        parameters=[
            {
                'robot_description': robot_description_path
            },
        ],
        respawn=True
    )

    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description_text}],
    )


    return LaunchDescription([
        webots,
        ros2_supervisor,
        my_robot_driver,
        state_publisher,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])