# Group3-Navigation-Robot
AIS Robotic System

## structure
```
navibot_ws/
│── build/
│── devel/
│── src/           # Git root
│   │── .git/
│   │── .gitignore
│   │── README.md
│   │── navibot/             # main
│   │── navibot_navigation/  # navigation
│   │── navibot_arm/         # arm
│   │── navibot_voice/       # voice
```

## create
```
mkdir -p ~/navibot_ws/src
cd ~/navibot_ws
catkin_make
```

create main pkg (already done)
```
cd ~/navibot_ws/src # which is git root dir
catkin_create_pkg navibot std_msgs rospy roscpp
```

create sub pkg
```
cd ~/navibot_ws/src
catkin_create_pkg navibot_navigation roscpp rospy std_msgs move_base amcl # navigation
catkin_create_pkg navibot_arm roscpp rospy std_msgs moveit_core moveit_ros_planning_interface # arm
catkin_create_pkg navibot_voice rospy std_msgs # voice control (already done)
```

## compile
```
cd ~/navibot_ws
catkin_make
source devel/setup.bash
```

## navigation
TODO

## arm
TODO

## voice
```
# sudo apt install portaudio19-dev
pip install openai SpeechRecognition pyaudio
```
test
```
roscore
```
in new terminal
```
rosrun navibot_voice voice_command.py
```
