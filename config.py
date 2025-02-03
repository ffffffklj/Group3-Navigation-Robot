# 目标位置配置
TARGET_POSITIONS = {
    "位置1": {"x": 1.0, "y": 0.0, "z": 1.0},
    "位置2": {"x": 0.0, "y": 1.0, "z": 1.0},
    "位置3": {"x": 1.0, "y": 1.0, "z": 1.0},
    "位置4": {"x": -1.0, "y": 0.0, "z": 1.0},
    "位置5": {"x": 0.0, "y": -1.0, "z": 1.0},
}

WINDOW_CONFIG = {
    "title": "Turtlebot控制界面",
    "geometry": "800x600",
    "bg_color": "#f0f0f0"
}

TITLE_STYLE = {
    "text": "Turtlebot控制面板",
    "font": ('Arial', 24, 'bold'),
    "bg": '#f0f0f0',
    "fg": '#333333',
    "pady": 20
}

BUTTON_STYLE = {
    "font": ('Times New Roman', 12),
    "width": 25,
    "height": 8,
    "bg": '#4a90e2',
    "fg": 'white',
    "relief": 'raised',
    "cursor": 'hand2'
}

BUTTON_HOVER_COLOR = '#357abd'
BUTTON_NORMAL_COLOR = '#4a90e2'

STATUS_BAR_STYLE = {
    "bd": 1,
    "relief": "sunken",
    "anchor": "w",
    "bg": '#e0e0e0',
    "pady": 5
}

# ROS配置
ROS_CONFIG = {
    "node_name": "robot_control_ui",
    "cmd_vel_topic": "/cmd_vel",
    "odom_topic": "/odom",
    "linear_speed": 0.2,
    "angular_speed": 0.0
}