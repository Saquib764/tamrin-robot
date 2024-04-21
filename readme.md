# Webots ros install
https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html
https://docs.ros.org/en/foxy/Tutorials/Advanced/Simulators/Webots/Installation-MacOS.html



# Develop

## Start webot server on host
https://github.com/cyberbotics/webots-server/blob/main/local_simulation_server.py

export WEBOTS_HOME=/Applications/Webots.app
python3 local.py

## Run on UTM VM machine
Do not use conda

source /opt/ros/humble/setup.bash

source install/setup.sh
colcon build 

ros2 launch tamrin_robot webots.launch.py
colcon build  && ros2 launch tamrin_robot webots_robot.launch.py
ros2 launch tamrin_robot webots_main.launch.py

ros2 launch localisation localisation.launch.py

ros2 run teleop_twist_keyboard teleop_twist_keyboard


# Create package
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python <name>


# Topics
ros2 topic list
/cmd_vel
/my_robot/camera/camera_info
/my_robot/camera/image_color
/parameter_events
/remove_urdf_robot
/rosout


# Issues
Dealing with permission issue-
sudo chown -R saquib:saquib ./webots_execute
