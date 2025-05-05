#!/usr/bin/env python3
import os
os.environ['NNPACK_VERBOSE'] = '0'
import rospy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import time
import math
from tf.transformations import euler_from_quaternion

class ButtonDetector:
    def __init__(self, skip_node_init=False):
        if not skip_node_init:
            rospy.init_node('button_approacher')
        rospy.loginfo("节点初始化开始...")
        
        # 参数设置
        self.DETECTION_THRESHOLD = 0.9
        self.LINEAR_SPEED = 0.1       # 线速度（米/秒）
        self.ANGULAR_SPEED = 0.15      # 角速度（弧度/秒）
        self.FOV = math.radians(30)    # 相机视场角（弧度）
        
        # 状态变量
        self.target_found = False
        self.target_angle = None       # 存储为弧度
        self.movement_completed = False
        
        # 初始化YOLO模型
        rospy.loginfo("正在加载YOLO模型...")
        self.model = YOLO('/home/ubuntu/navibot_ws/src/navibot/scripts/simulation_button.pt')
        self.bridge = CvBridge()
        
        # 使用正确的话题名称
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        
        rospy.sleep(1)  # 等待发布者和订阅者初始化
        self.stop_robot()
        rospy.loginfo("初始化完成")

    def stop_robot(self):
        """停止机器人"""
        cmd_vel = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(cmd_vel)
            rospy.sleep(0.1)

    def image_callback(self, msg):
        """图像回调函数 - 只在未检测到目标时执行"""
        if self.target_found:
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            results = self.model(cv_image)
            
            if len(results) > 0 and len(results[0].boxes) > 0:
                boxes = results[0].boxes
                confidences = boxes.conf.cpu().numpy()
                max_conf_idx = np.argmax(confidences)
                
                if confidences[max_conf_idx] > self.DETECTION_THRESHOLD:
                    box = boxes[max_conf_idx].xywh.cpu().numpy()[0]
                    image_center = cv_image.shape[1] / 2
                    button_center_x = box[0]
                    
                    # 计算目标在图像中的相对位置（-1到1之间）
                    relative_pos = (button_center_x - image_center) / image_center
                    # 计算实际角度（弧度）
                    self.target_angle = -relative_pos * (self.FOV / 2)
                    
                    self.target_found = True
                    rospy.loginfo(f"检测到目标！角度偏移: {math.degrees(self.target_angle):.2f}度")
                    
                    # 取消订阅，不再处理图像
                    self.image_sub.unregister()
                    
                    # 直接开始移动
                    self.execute_movement()
                    
        except Exception as e:
            rospy.logerr(f"处理图像时出错: {str(e)}")

    def execute_movement(self):
        """执行移动 - 先旋转，再前进固定距离"""
        try:
            rospy.loginfo("开始执行移动...")
            
            # 1. 旋转到目标角度
            rospy.loginfo(f"旋转到目标角度: {math.degrees(self.target_angle):.2f}度")
            self.rotate(self.target_angle)
            rospy.sleep(1)  # 等待稳定
            
            # 2. 前进固定距离0.6米
            rospy.loginfo("前进距离: 0.7米")
            self.move_forward(0.7)
            
            self.movement_completed = True
            rospy.loginfo("移动完成！")
            
        except Exception as e:
            rospy.logerr(f"执行移动时出错: {str(e)}")
            self.stop_robot()

    def rotate(self, angle_rad):
        """旋转指定角度（弧度）"""
        vel_msg = Twist()
        
        # 计算旋转时间（降低补偿系数到1.1）
        angular_speed = self.ANGULAR_SPEED
        if abs(angle_rad) < math.radians(10):  # 小角度时更大幅度降低速度
            angular_speed *= 0.3
            
        rotation_time = abs(angle_rad / angular_speed) * 1.1
        
        # 设置旋转方向
        vel_msg.angular.z = angular_speed if angle_rad > 0 else -angular_speed
        
        # 执行旋转
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)
        
        while (rospy.Time.now() - start_time).to_sec() < rotation_time:
            self.cmd_vel_pub.publish(vel_msg)
            rate.sleep()
            
        self.stop_robot()

    def move_forward(self, distance):
        """向前移动指定距离"""
        vel_msg = Twist()
        
        # 计算移动时间（降低补偿系数到1.05）
        linear_speed = self.LINEAR_SPEED
        if distance < 0.3:  # 增加短距离的判定范围，更大幅度降低速度
            linear_speed *= 0.4
            
        movement_time = (distance / linear_speed) * 1.05
        
        # 设置移动速度
        vel_msg.linear.x = linear_speed if distance > 0 else 0
        
        # 执行移动
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)
        
        while (rospy.Time.now() - start_time).to_sec() < movement_time:
            self.cmd_vel_pub.publish(vel_msg)
            rate.sleep()
            
        self.stop_robot()

    def run(self):
        """主运行循环"""
        rate = rospy.Rate(1)  # 1Hz状态更新
        
        while not rospy.is_shutdown():
            if not self.target_found:
                rospy.loginfo_throttle(1.0, "等待检测目标...")
            elif not self.movement_completed:
                rospy.loginfo_throttle(1.0, "正在执行移动...")
            else:
                rospy.loginfo_throttle(1.0, "任务完成")
                break  # 任务完成后退出循环
            rate.sleep()

# 新增一个继承类，用于navigation_gui.py调用
class NavigationButtonDetector(ButtonDetector):
    def __init__(self, skip_node_init=True):
        # 调用父类的初始化方法
        super().__init__(skip_node_init=skip_node_init)

if __name__ == '__main__':
    try:
        detector = ButtonDetector()  # 使用原始类
        detector.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"程序出错: {str(e)}")
    finally:
        try:
            detector.stop_robot()
        except:
            pass
