#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from math import radians
import sys

class PathPlanner:
    def __init__(self):
        rospy.init_node('path_planner')
        rospy.loginfo("初始化路径规划节点...")
        
        # 创建 move_base 动作客户端
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("等待 move_base 服务器...")
        self.client.wait_for_server()
        rospy.loginfo("连接到 move_base 服务器！")

        # 定义所有位置
        self.locations = {
            # Layer 1
            'elevator_l1': [(0.03, -1.2, 0), (0, 0, 1, 1)],
            'reception': [(-5, -1.8, 0), (0, 0, 1, 1)],
            'lecture_hall': [(3.58, 4.30, 0), (0, 0, 1, 1)],
            'admin_office': [(9.7, 0.1, 0), (0, 0, 0, 1)],
            
            # Layer 2
            'elevator_l2': [(-0.7, -0.65, 0), (0, 0, -1, 1)],
            'lab': [(-3.414, -3.633, 0), (0, 0, -1, 1)],
            'classroom': [(2.315, -3.346, 0), (0, 0, 0, 1)],
            'seminar_room': [(5.888, -1.195, 0), (0, 0, 0, 1)],
            
            # Layer 3
            'elevator_l3': [(-0.998, -1.223, 0), (0, 0, 1, 1)],
            'professor_office': [(-7.293, -5.396, 0), (0, 0, -1, 1)],
            'library': [(-3.267, 3.461, 0), (0, 0, 1, 1)],
            'study_room': [(5.319, 2.264, 0), (0, 0, 0, 1)]
        }

    def create_goal(self, position, orientation):
        """创建导航目标"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # 设置目标位置和方向
        goal.target_pose.pose = Pose(
            Point(position[0], position[1], position[2]),
            Quaternion(orientation[0], orientation[1], orientation[2], orientation[3])
        )
        
        return goal

    def go_to_location(self, location_name):
        """导航到指定位置"""
        if location_name not in self.locations:
            rospy.logerr(f"未知位置: {location_name}")
            return False
            
        location = self.locations[location_name]
        position = location[0]
        orientation = location[1]
        
        rospy.loginfo(f"导航到: {location_name}")
        rospy.loginfo(f"位置: {position}")
        rospy.loginfo(f"方向: {orientation}")
        
        # 创建并发送导航目标
        goal = self.create_goal(position, orientation)
        self.client.send_goal(goal)
        
        # 等待结果
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("动作服务器无响应")
            return False
            
        state = self.client.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"成功到达: {location_name}")
            return True
        else:
            rospy.logerr(f"导航失败，状态码: {state}")
            return False

    def print_available_locations(self):
        """打印所有可用的位置"""
        rospy.loginfo("\n可用的位置:")
        rospy.loginfo("Layer 1:")
        rospy.loginfo("  - elevator_l1 (电梯)")
        rospy.loginfo("  - reception (接待室)")
        rospy.loginfo("  - lecture_hall (教室)")
        rospy.loginfo("  - admin_office (行政办公室)")
        
        rospy.loginfo("\nLayer 2:")
        rospy.loginfo("  - elevator_l2 (电梯)")
        rospy.loginfo("  - lab (实验室)")
        rospy.loginfo("  - classroom (教室)")
        rospy.loginfo("  - seminar_room (研讨室)")
        
        rospy.loginfo("\nLayer 3:")
        rospy.loginfo("  - elevator_l3 (电梯)")
        rospy.loginfo("  - professor_office (教授办公室)")
        rospy.loginfo("  - library (图书室)")
        rospy.loginfo("  - study_room (自习室)")

def main():
    try:
        planner = PathPlanner()
        
        if len(sys.argv) < 2:
            rospy.loginfo("请指定目标位置!")
            planner.print_available_locations()
            rospy.loginfo("\n使用方法: rosrun navibot path_planning.py <location_name>")
            return
            
        location = sys.argv[1]
        planner.go_to_location(location)
        
    except rospy.ROSInterruptException:
        rospy.logerr("程序被中断")

if __name__ == '__main__':
    main()
