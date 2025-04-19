#!/usr/bin/env python3
import sys
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from math import pi

class ArmController:
    def __init__(self, skip_node_init=False):
        if not skip_node_init:
            rospy.init_node('arm_controller', anonymous=True)
        
        try:
            # 创建关节控制器的发布者
            self.joint_publishers = [
                rospy.Publisher(f'/om_with_tb3/joint{i+1}_position_controller/command', Float64, queue_size=10)
                for i in range(4)
            ]
            
            # 当前状态
            self.current_state = "home"
            
            # 预定义位置
            self.positions = {
                "start": [0.0, 0.0, 0.0, 0.0],          # 初始位置，全0
                "home": [0.0, -1, 0.45, 0.4],           # 向后收缩位置
                "up": [0.0, 0.3, -0.6, -0.4],            # 向前上方伸出
                "down": [0.0, 0.3, -0.3, 0.2]            # 向前下方伸出
            }
            
            rospy.loginfo("等待系统初始化...")
            rospy.sleep(3)  # 等待系统初始化
            
            # 确保从start状态开始
            self.move_joints(self.positions["home"])
            rospy.sleep(1)
            
            rospy.loginfo("机械臂控制器初始化完成")
            
        except Exception as e:
            rospy.logerr(f"初始化错误: {str(e)}")
            raise e

    def move_joints(self, positions, duration=0.6):
        """移动关节到指定位置"""
        try:
            rospy.loginfo(f"移动关节到位置: {positions}")
            
            # 发布关节位置命令
            for i, (pub, pos) in enumerate(zip(self.joint_publishers, positions)):
                rospy.loginfo(f"发布关节 {i+1} 位置: {pos}")
                pub.publish(Float64(pos))
            
            # 等待运动完成
            rospy.sleep(duration)
            return True
                
        except Exception as e:
            rospy.logerr(f"关节运动出错: {str(e)}")
            return False

    def move_to_state(self, target_state):
        """移动到指定状态"""
        if target_state not in self.positions:
            rospy.logerr(f"未知状态: {target_state}")
            return False
            
        try:
            rospy.loginfo(f"当前状态: {self.current_state}, 目标状态: {target_state}")
            
            if target_state != self.current_state:
                # 直接移动到目标位置
                rospy.loginfo(f"移动到{target_state}位置...")
                self.move_joints(self.positions[target_state])
                rospy.sleep(1)
                
                # 更新当前状态
                self.current_state = target_state
                rospy.loginfo(f"已切换到{target_state}状态")
            else:
                rospy.loginfo(f"已经在{target_state}状态，无需切换")
            
            return True
            
        except Exception as e:
            rospy.logerr(f"移动到{target_state}状态时出错: {str(e)}")
            return False

    def print_help(self):
        """打印帮助信息"""
        rospy.loginfo("\n可用命令:")
        rospy.loginfo("start  - 移动到初始位置（全0位置）")
        rospy.loginfo("home   - 移动到收缩位置（向后收缩）")
        rospy.loginfo("up     - 移动到向前上方位置")
        rospy.loginfo("down   - 移动到向前下方位置")
        rospy.loginfo("help   - 显示此帮助信息")
        rospy.loginfo("quit   - 退出程序")

def main():
    try:
        controller = ArmController()
        controller.print_help()
        
        # 等待命令
        while not rospy.is_shutdown():
            command = input("\n请输入命令: ").lower()
            
            if command == "quit":
                break
            elif command == "help":
                controller.print_help()
            elif command in controller.positions:
                controller.move_to_state(command)
            else:
                rospy.logwarn("无效命令，输入'help'查看可用命令")
                
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"程序出错: {str(e)}")

if __name__ == '__main__':
    main()
