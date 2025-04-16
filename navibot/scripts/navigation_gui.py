#!/usr/bin/env python3
import sys
import rospy
import actionlib
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                           QHBoxLayout, QPushButton, QLabel, QGroupBox, QGridLayout,
                           QComboBox)
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from actionlib_msgs.msg import GoalStatus
import tf
from button_approacher import ButtonDetector
from touch_button import ArmController
import time

class NavigationThread(QThread):
    # 定义信号
    navigation_status = pyqtSignal(str)
    navigation_complete = pyqtSignal(bool, str)  # 添加目标位置参数

    def __init__(self, client, goal, target_location):
        super().__init__()
        self.client = client
        self.goal = goal
        self.target_location = target_location

    def run(self):
        self.navigation_status.emit("导航开始...")
        self.client.send_goal(self.goal)
        
        # 等待结果，设置超时时间为60秒
        finished_within_time = self.client.wait_for_result(rospy.Duration(60))
        
        if not finished_within_time:
            self.navigation_status.emit("导航超时！")
            self.client.cancel_goal()
            self.navigation_complete.emit(False, self.target_location)
            return

        state = self.client.get_state()
        if state == GoalStatus.SUCCEEDED:
            self.navigation_status.emit("导航成功！")
            self.navigation_complete.emit(True, self.target_location)
        else:
            self.navigation_status.emit(f"导航失败，状态码: {state}")
            self.navigation_complete.emit(False, self.target_location)

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

        # 设置当前楼层，默认为1
        self.current_floor = 1

        # 定义所有楼层的导航位置
        self.all_locations = {
            1: {  # Layer 1
                'elevator_l1': [ (0.042, -2.086, 0), (0, 0, 1, 1)],
                'reception': [(-5, -1.8, 0), (0, 0, 1, 1)],
                'lecture_hall': [(3.58, 4.30, 0), (0, 0, 1, 1)],
                'admin_office': [(9.7, 0.1, 0), (0, 0, 0, 1)],
            },
            2: {  # Layer 2
                'elevator_l2': [ (-0.754, 0.05, 0), (0, 0, -1, 1)],
                'lab': [(-3.414, -3.633, 0), (0, 0, -1, 1)],
                'classroom': [(2.315, -3.346, 0), (0, 0, 0, 1)],
                'seminar_room': [(5.888, -1.195, 0), (0, 0, 0, 1)],
            },
            3: {  # Layer 3
                'elevator_l3': [ (-1.022, -1.73, 0), (0, 0, 1, 1)],
                'professor_office': [(-7.293, -5.396, 0), (0, 0, -1, 1)],
                'library': [(-3.267, 3.461, 0), (0, 0, 1, 1)],
                'study_room': [(5.319, 2.264, 0), (0, 0, 0, 1)],
            }
        }

        # 创建TF监听器
        self.tf_listener = tf.TransformListener()

        # 初始化机械臂控制器（但暂不执行任何动作）
        self.arm_controller = None
        self.button_detector = None

    def initUI(self):
        self.setWindowTitle('TurtleBot3 多楼层导航控制面板')
        self.setGeometry(100, 100, 1000, 600)

        # 创建主窗口部件和布局
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout()

        # 创建楼层选择部分
        floor_group = QGroupBox("当前楼层")
        floor_layout = QHBoxLayout()
        
        # 添加楼层按钮
        self.floor_buttons = []
        for floor in range(1, 4):
            btn = QPushButton(f"第{floor}层")
            btn.setCheckable(True)
            btn.clicked.connect(lambda checked, f=floor: self.set_current_floor(f))
            floor_layout.addWidget(btn)
            self.floor_buttons.append(btn)
        
        # 设置默认楼层
        self.floor_buttons[0].setChecked(True)
        
        floor_group.setLayout(floor_layout)
        layout.addWidget(floor_group)

        # 创建状态显示标签
        self.status_label = QLabel("就绪")
        self.status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.status_label)

        # 创建所有楼层的位置按钮
        for floor in range(1, 4):
            floor_group = QGroupBox(f"第{floor}层 - 导航目标")
            grid_layout = QGridLayout()

            # 添加该楼层的所有位置按钮
            for i, (name, coords) in enumerate(self.all_locations[floor].items()):
                btn = QPushButton(name.replace('_', ' ').title())
                btn.clicked.connect(lambda checked, f=floor, n=name: self.navigate_to(f, n))
                row = i // 2
                col = i % 2
                grid_layout.addWidget(btn, row, col)

            floor_group.setLayout(grid_layout)
            layout.addWidget(floor_group)

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
        self.position_timer = self.startTimer(1000)

    def set_current_floor(self, floor):
        """设置当前楼层"""
        self.current_floor = floor
        # 更新按钮状态
        for i, btn in enumerate(self.floor_buttons):
            btn.setChecked(i + 1 == floor)
        self.status_label.setText(f"当前楼层设置为: 第{floor}层")

    def navigate_to(self, target_floor, location_name):
        """导航到指定位置，如果目标楼层不是当前楼层，则先导航到电梯"""
        try:
            if target_floor == self.current_floor:
                # 同一楼层直接导航
                self.execute_navigation(target_floor, location_name)
            else:
                # 不同楼层，先导航到当前楼层的电梯
                elevator_name = f'elevator_l{self.current_floor}'
                self.execute_navigation(self.current_floor, elevator_name, 
                                     next_floor=target_floor, 
                                     final_destination=location_name)
        except Exception as e:
            rospy.logerr(f"导航出错: {str(e)}")
            self.status_label.setText(f"导航出错: {str(e)}")

    def execute_navigation(self, floor, location_name, next_floor=None, final_destination=None):
        """执行导航，并在到达后处理电梯操作"""
        position, orientation = self.all_locations[floor][location_name]
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(
            Point(position[0], position[1], position[2]),
            Quaternion(orientation[0], orientation[1], orientation[2], orientation[3])
        )

        # 创建并启动导航线程
        self.nav_thread = NavigationThread(self.client, goal, location_name)
        self.nav_thread.navigation_status.connect(self.update_status)
        self.nav_thread.navigation_complete.connect(
            lambda success, loc: self.handle_navigation_complete(success, loc, next_floor, final_destination))
        self.nav_thread.start()

    def handle_navigation_complete(self, success, location_name, next_floor=None, final_destination=None):
        """处理导航完成后的操作"""
        if not success:
            self.status_label.setText(f"导航到 {location_name} 失败")
            return

        if "elevator" in location_name:
            self.status_label.setText("到达电梯，准备操作按钮...")
            self.operate_elevator(next_floor, final_destination)
        else:
            self.status_label.setText(f"已到达目的地: {location_name}")

    def operate_elevator(self, next_floor=None, final_destination=None):
        """操作电梯并处理楼层切换"""
        try:
            # 初始化按钮检测器和机械臂控制器
            if self.button_detector is None:
                self.button_detector = ButtonDetector()
            if self.arm_controller is None:
                self.arm_controller = ArmController()

            # 等待按钮检测器和机械臂控制器初始化
            rospy.sleep(2)

            # 根据目标楼层决定上楼还是下楼
            if next_floor > self.current_floor:
                # 上楼
                self.status_label.setText("正在按上楼按钮...")
                self.arm_controller.move_to_state("up")
                rospy.sleep(2)  # 短暂停留
                self.arm_controller.move_to_state("home")
                self.status_label.setText(f"已按上楼按钮，模拟切换到第{next_floor}层")
            else:
                # 下楼
                self.status_label.setText("正在按下楼按钮...")
                self.arm_controller.move_to_state("down")
                rospy.sleep(2)  # 短暂停留
                self.arm_controller.move_to_state("home")
                self.status_label.setText(f"已按下楼按钮，模拟切换到第{next_floor}层")

            # 更新当前楼层
            self.set_current_floor(next_floor)
            
            # 在仿真环境中，到这里就停止，不再继续导航
            if final_destination:
                self.status_label.setText(f"已模拟切换到第{next_floor}层。在仿真环境中，无法继续导航到{final_destination}")

        except Exception as e:
            rospy.logerr(f"电梯操作出错: {str(e)}")
            self.status_label.setText(f"电梯操作出错: {str(e)}")

    def update_status(self, status):
        """更新状态显示"""
        self.status_label.setText(status)

    def stop_navigation(self):
        """停止导航"""
        self.client.cancel_all_goals()
        self.status_label.setText("导航已停止")
        self.status_label.setStyleSheet("color: orange")

    def timerEvent(self, event):
        """更新当前位置显示"""
        try:
            self.tf_listener.waitForTransform("/map", "/base_footprint", rospy.Time(), rospy.Duration(0.1))
            (trans, rot) = self.tf_listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
            self.position_label.setText(f"当前位置: X: {trans[0]:.2f}, Y: {trans[1]:.2f}")
        except:
            pass

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
