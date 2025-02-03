#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk
from config import (
    TARGET_POSITIONS, 
    WINDOW_CONFIG, 
    TITLE_STYLE, 
    BUTTON_STYLE,
    BUTTON_HOVER_COLOR,
    BUTTON_NORMAL_COLOR,
    STATUS_BAR_STYLE
)

class RobotControlUI:
    def __init__(self):
        # 创建主窗口
        self.root = tk.Tk()
        self.root.title(WINDOW_CONFIG["title"])
        self.root.geometry(WINDOW_CONFIG["geometry"])
        self.root.configure(bg=WINDOW_CONFIG["bg_color"])
        
        # 预定义位置配置
        self.positions = TARGET_POSITIONS
        
        self.create_ui()
        
    def create_ui(self):
        # 创建标题
        title_label = tk.Label(
            self.root,
            **TITLE_STYLE
        )
        title_label.pack()
        
        # 创建按钮容器
        self.button_frame = tk.Frame(self.root, bg=WINDOW_CONFIG["bg_color"])
        self.button_frame.pack(expand=True, fill='both', padx=20, pady=20)
        
        # 使用网格布局创建按钮
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
            
            # 添加悬停效果
            btn.bind('<Enter>', lambda e, b=btn: self.on_hover(b, True))
            btn.bind('<Leave>', lambda e, b=btn: self.on_hover(b, False))
            
            col += 1
            if col >= 3:  # 每行3个按钮
                col = 0
                row += 1

        # 添加状态栏
        self.status_bar = tk.Label(
            self.root,
            text="就绪",
            **STATUS_BAR_STYLE
        )
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)

    def on_hover(self, button, entering):
        """按钮悬停效果"""
        if entering:
            button.configure(bg=BUTTON_HOVER_COLOR)
        else:
            button.configure(bg=BUTTON_NORMAL_COLOR)
            
    def move_robot(self, position):
        """处理机器人移动请求"""
        status_text = f"目标位置: x={position['x']:.2f}, y={position['y']:.2f}, z={position['z']:.2f}"
        self.status_bar.config(text=status_text)
        print(status_text)

    def run(self):
        self.root.mainloop()

if __name__ == '__main__':
    app = RobotControlUI()
    app.run()