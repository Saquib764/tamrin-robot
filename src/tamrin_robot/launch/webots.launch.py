import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory('tamrin_robot'))
    print(pkg_path)

    webots = WebotsLauncher(
        world=os.path.join(pkg_path, 'worlds', 'webots_world_1.wbt')
    )

    ros2_supervisor = Ros2SupervisorLauncher()


    return LaunchDescription([
        webots,
        ros2_supervisor,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])