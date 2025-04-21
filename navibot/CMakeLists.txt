cmake_minimum_required(VERSION 3.0.2)
project(navibot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES navibot
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# 添加 Python 脚本安装配置
# catkin_install_python(PROGRAMS
#   scripts/my_script.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# 添加其他可能需要的 Python 脚本，按需取消注释和修改文件名
# catkin_install_python(PROGRAMS
#   scripts/another_script.py
#   scripts/third_script.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

catkin_install_python(PROGRAMS
  scripts/navigation_gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装启动文件（如果有）
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

# 安装配置文件（如果有）
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)
