#!/usr/bin/env python3

import rospy
import tkinter as tk
from tkinter import ttk
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from math import atan2, sqrt
from config import (
    TARGET_POSITIONS, 
    WINDOW_CONFIG, 
    TITLE_STYLE, 
    BUTTON_STYLE,
    BUTTON_HOVER_COLOR,
    BUTTON_NORMAL_COLOR,
    STATUS_BAR_STYLE,
    ROS_CONFIG
)

class RobotControlUI:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node(ROS_CONFIG["node_name"])
        
        self.root = tk.Tk()
        self.root.title(WINDOW_CONFIG["title"])
        self.root.geometry(WINDOW_CONFIG["geometry"])
        self.root.configure(bg=WINDOW_CONFIG["bg_color"])
        
        # 机器人当前位置
        self.current_position = Point()
        
        # 目标位置
        self.positions = TARGET_POSITIONS
        
        self.create_ui()
        
    def create_ui(self):
        title_label = tk.Label(
            self.root,
            **TITLE_STYLE
        )
        title_label.pack()
        
        self.button_frame = tk.Frame(self.root, bg=WINDOW_CONFIG["bg_color"])
        self.button_frame.pack(expand=True, fill='both', padx=20, pady=20)
        
        row = 0
        col = 0
        for pos_name, coords in self.positions.items():
            btn = tk.Button(
                self.button_frame,
                text=f"{pos_name}\n(x:{coords['x']:.2f}, y:{coords['y']:.2f}, z:{coords['z']:.2f})",
                command=lambda pos=coords: self.move_robot(pos),
                **BUTTON_STYLE
            )
            btn.grid(row=row, column=col, padx=10, pady=10)
            
            btn.bind('<Enter>', lambda e, b=btn: self.on_hover(b, True))
            btn.bind('<Leave>', lambda e, b=btn: self.on_hover(b, False))
            
            col += 1
            if col >= 3:  # 每行3个按钮
                col = 0
                row += 1

        self.status_bar = tk.Label(
            self.root,
            text="就绪",
            **STATUS_BAR_STYLE
        )
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)

    def on_hover(self, button, entering):
        if entering:
            button.configure(bg=BUTTON_HOVER_COLOR)
        else:
            button.configure(bg=BUTTON_NORMAL_COLOR)
    
    def odom_callback(self, msg):
        """接收里程计数据的回调函数"""
        self.current_position = msg.pose.pose.position
            
    def move_robot(self, position):
        # TODO
        # 根据目标位置position，调用自动导航的功能
        pass

    def run(self):
        self.root.mainloop()

if __name__ == '__main__':
    try:
        app = RobotControlUI()
        app.run()
    except rospy.ROSInterruptException:
        pass