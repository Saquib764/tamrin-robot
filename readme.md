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


# Issues
Dealing with permission issue-
sudo chown -R saquib:saquib ./webots_execute
