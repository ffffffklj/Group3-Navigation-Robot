# Clone essential packages
[ -d "ar_track_alvar_msgs" ] || git clone -b noetic-devel https://github.com/ros-perception/ar_track_alvar_msgs.git
[ -d "open_manipulator" ] || git clone -b noetic-devel https://github.com/ROBOTIS-GIT/open_manipulator.git
[ -d "open_manipulator_msgs" ] || git clone -b noetic-devel https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git
[ -d "open_manipulator_simulations" ] || git clone -b noetic-devel https://github.com/ROBOTIS-GIT/open_manipulator_simulations.git
[ -d "open_manipulator_with_tb3" ] || git clone https://github.com/ROBOTIS-GIT/open_manipulator_with_tb3.git

[ -d "turtlebot3" ] || git clone -b noetic-devel git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
[ -d "turtlebot3_msgs" ] || git clone -b noetic git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
[ -d "turtlebot3_simulations" ] || git clone -b noetic-devel git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

# TODO: move arm.yaml to config/

# Install dependent system packages
sudo apt update
sudo apt install ros-noetic-moveit ros-noetic-moveit-msgs python3-rospkg python3-defusedxml -y

rosdep install --from-paths src --ignore-src -r -y

# Return to workspace and build
cd ..
rm -rf build/ devel/
catkin_make
source devel/setup.sh