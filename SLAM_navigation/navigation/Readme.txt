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
