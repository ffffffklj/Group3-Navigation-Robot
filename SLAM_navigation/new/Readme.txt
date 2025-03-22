需要安装局部规划器teb_local_planner（如果没有）
sudo apt-get install ros-noetic-teb-local-planner
查找teb_local_planner 包的目录，新建param包导入 teb_local_planner_params.yaml 文件，包含了该规划其的参数信息（自己可以调整）




导航首先bringup，后运行Project.launch文件
注意Project.launch需要在ROS包下，可以新建一个ROS包然后将所需文件导入
注意路径yaml里面的地图图片路径要和实际一致不然报错
主要Project.launch里面
<!-- Path Planning Node -->
  <!-- To replace XXXXX below! -->
  <node pkg="autonomous" name="path_planning_Node" type="path_planning.py" args="$(arg map_file)"/>
  pkg即为Project.launch所在的ROS包名，后面为py文件名，以及py里面的节点名和这里的name要一致
path_planning.py即为路径规划，可根据实际地图更新参数
自己定义的move_base.launch需要找到turtlebot3_navigation下的同一文件进行替换，该文件定义了amcl节点以及全局规划和局部规划的内容



运行导航Project.launch命令是
roslaunch ROS包名 Project.launch map_file:=地图路径名
地图需要SLAM获得,使用karto方法（课堂为gmapping)


以上均未实物测试，之后需要bringup实物后进行验证






仿真：
启动roscore后
构图过程：
首先在gazebo仿真环境中启动turtlebot3小车
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch（替换为自己的环境模型）

开启SLAM功能，建图算法选择karto

    export TURTLEBOT3_MODEL=waffle_pi
    roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=karto
    
启动键盘控制节点，控制小车在环境内转一圈，尽可能扫出完整封闭的地图

    export TURTLEBOT3_MODEL=waffle_pi
    roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
保存地图

rosrun map_server map_saver -f ~/karto(最后的karto为名字）


导航功能：
打开仿真环境
roslaunch turtlebot3_gazebo turtlebot3_house.launch
注意替换实际的环境模型

加载保存的地图启动导航
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/zwy/karto.yaml
注意进行图片和包的替换

rostopic echo /gazebo/model_states
用于查看gazebo模型的坐标或者

rosservice call /gazebo/get_model_state "model_name: 'turtlebot3'"
或者只查看turtlebot3的坐标

rosrun autonomous initial_pose.py(autonoumous替换为自己包的名字)
初始化节点位置（里面坐标需要根据实际gazebo里的位置调整）,会使得RIZ的机器人位置初始化和gazebo一样

rosrun autonomous path_planning.py
将会按照waypoint的位置进行路径规划到对应位置（同上）
注意：以上文件的坐标设置都以gazebo里面的位置为准



