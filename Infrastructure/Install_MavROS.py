# Based on :
#https://pixhawk.org/dev/ros/ground_rover

#----Main installation : ----
# https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation
import os

sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
os.system("wget https://github.com/mavlink/mavros/tree/master/mavros/scripts/install_geographiclib_datasets.sh")


sudo apt-get install python-catkin-tools python-rosinstall-generator -y

# 1. Create the workspace: unneded if you already has workspace
cd ~/catkin_ws
wstool init src

# 2. Install MAVLink
#    we use the Kinetic reference for all ROS distros as it's not distro-specific and up to date
rosinstall_generator --rosdistro kinetic mavlink | tee /tmp/mavros.rosinstall

# 3. Install MAVROS: get source (upstream - released)
rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall
# alternative: latest source
# rosinstall_generator --upstream-development mavros | tee -a /tmp/mavros.rosinstall
# For fetching all the dependencies into your catkin_ws, just add '--deps' to the above scripts
# ex: rosinstall_generator --upstream mavros --deps | tee -a /tmp/mavros.rosinstall

# 4. Create workspace & deps
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src -j4
rosdep install --from-paths src --ignore-src -y

# 5. Install GeographicLib datasets:
./src/mavros/mavros/scripts/install_geographiclib_datasets.sh

# 6. Build source
catkin build

# 7. Make sure that you use setup.bash or setup.zsh from workspace.
#    Else rosrun can't find nodes from this workspace.
source devel/setup.bash


# mavlink extras :
#https://github.com/mavlink/mavros/blob/master/mavros_extras/README.md
