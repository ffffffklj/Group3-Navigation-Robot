#!/usr/bin/env python3
import sys
import rospy
import actionlib
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                           QHBoxLayout, QPushButton, QLabel, QGroupBox, QGridLayout)
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from actionlib_msgs.msg import GoalStatus
import tf

class NavigationThread(QThread):
    # 定义信号
    navigation_status = pyqtSignal(str)
    navigation_complete = pyqtSignal(bool)

    def __init__(self, client, goal):
        super().__init__()
        self.client = client
        self.goal = goal

    def run(self):
        self.navigation_status.emit("导航开始...")
        self.client.send_goal(self.goal)
        
        # 等待结果，设置超时时间为60秒
        finished_within_time = self.client.wait_for_result(rospy.Duration(60))
        
        if not finished_within_time:
            self.navigation_status.emit("导航超时！")
            self.client.cancel_goal()
            self.navigation_complete.emit(False)
            return

        state = self.client.get_state()
        if state == GoalStatus.SUCCEEDED:
            self.navigation_status.emit("导航成功！")
            self.navigation_complete.emit(True)
        else:
            self.navigation_status.emit(f"导航失败，状态码: {state}")
            self.navigation_complete.emit(False)

class NavigationGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.initROS()
        self.initUI()

    def initROS(self):
        # 初始化ROS节点
        rospy.init_node('navigation_gui', anonymous=True)
        
        # 创建action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("等待move_base服务器...")
        self.client.wait_for_server()
        rospy.loginfo("连接到move_base服务器！")

        # 定义导航位置
        self.locations = {
            # Layer 1
            'elevator_l1': [(0.03, -1.2, 0), (0, 0, 1, 1)],
            'reception': [(-5, -1.8, 0), (0, 0, 1, 1)],
            'lecture_hall': [(3.58, 4.30, 0), (0, 0, 1, 1)],
            'admin_office': [(9.7, 0.1, 0), (0, 0, 0, 1)],
        }

        # 创建TF监听器
        self.tf_listener = tf.TransformListener()

    def initUI(self):
        self.setWindowTitle('TurtleBot3 导航控制面板')
        self.setGeometry(100, 100, 800, 400)

        # 创建主窗口部件和布局
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout()
        
        # 创建状态显示标签
        self.status_label = QLabel("就绪")
        self.status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.status_label)

        # 创建位置按钮组
        locations_group = QGroupBox("导航目标")
        grid_layout = QGridLayout()

        # 添加位置按钮
        for i, (name, coords) in enumerate(self.locations.items()):
            btn = QPushButton(name.replace('_', ' ').title())
            btn.clicked.connect(lambda checked, n=name: self.navigate_to(n))
            row = i // 2
            col = i % 2
            grid_layout.addWidget(btn, row, col)

        locations_group.setLayout(grid_layout)
        layout.addWidget(locations_group)

        # 添加当前位置显示
        self.position_label = QLabel("当前位置: 未知")
        layout.addWidget(self.position_label)

        # 添加停止按钮
        stop_btn = QPushButton("停止导航")
        stop_btn.clicked.connect(self.stop_navigation)
        stop_btn.setStyleSheet("background-color: #ff6b6b")
        layout.addWidget(stop_btn)

        main_widget.setLayout(layout)

        # 启动定时器更新位置
        self.position_timer = self.startTimer(1000)  # 每秒更新一次

    def timerEvent(self, event):
        # 更新当前位置
        try:
            self.tf_listener.waitForTransform("/map", "/base_footprint", rospy.Time(), rospy.Duration(0.1))
            (trans, rot) = self.tf_listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
            self.position_label.setText(f"当前位置: X: {trans[0]:.2f}, Y: {trans[1]:.2f}")
        except:
            pass

    def create_goal(self, position, orientation):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(
            Point(position[0], position[1], position[2]),
            Quaternion(orientation[0], orientation[1], orientation[2], orientation[3])
        )
        return goal

    def navigate_to(self, location_name):
        if location_name not in self.locations:
            self.status_label.setText(f"错误：未知位置 {location_name}")
            return

        position, orientation = self.locations[location_name]
        goal = self.create_goal(position, orientation)
        
        # 创建并启动导航线程
        self.nav_thread = NavigationThread(self.client, goal)
        self.nav_thread.navigation_status.connect(self.update_status)
        self.nav_thread.navigation_complete.connect(self.navigation_finished)
        self.nav_thread.start()

    def update_status(self, status):
        self.status_label.setText(status)

    def navigation_finished(self, success):
        if success:
            self.status_label.setStyleSheet("color: green")
        else:
            self.status_label.setStyleSheet("color: red")

    def stop_navigation(self):
        self.client.cancel_all_goals()
        self.status_label.setText("导航已停止")
        self.status_label.setStyleSheet("color: orange")

def main():
    app = QApplication(sys.argv)
    gui = NavigationGUI()
    gui.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
