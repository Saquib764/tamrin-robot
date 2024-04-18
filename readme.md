# Webots ros install
https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html
https://docs.ros.org/en/foxy/Tutorials/Advanced/Simulators/Webots/Installation-MacOS.html



# Develop
source /opt/ros/humble/setup.bash

colcon build && source install/setup.sh

source install/setup.sh && ros2 launch tamrin_robot webots.launch.py
