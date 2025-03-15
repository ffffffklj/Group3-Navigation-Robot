import cv2
from ultralytics import YOLO
import time
import numpy as np
import math

# 加载训练好的YOLOv8模型
model = YOLO("BaD.pt")  # 修改为使用BaD.pt模型

# 打开摄像头
cap = cv2.VideoCapture(0)  # 0表示默认摄像头

# 检查摄像头是否成功打开
if not cap.isOpened():
    print("无法打开摄像头")
    exit()

# 设置窗口名称
window_name = "电梯按钮和门检测"
cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

# 电梯门状态判断的阈值（可根据实际情况调整）
DOOR_OPEN_THRESHOLD = 2.0  # 当两个门的相对距离大于此值时，认为门是开的

def calculate_door_distance(door1, door2):
    """计算两个电梯门之间的相对距离"""
    # 提取两个门的中心点
    x1 = (door1[0] + door1[2]) / 2
    y1 = (door1[1] + door1[3]) / 2
    
    x2 = (door2[0] + door2[2]) / 2
    y2 = (door2[1] + door2[3]) / 2
    
    # 计算欧氏距离
    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    # 计算两个门的平均宽度
    width1 = door1[2] - door1[0]
    width2 = door2[2] - door2[0]
    avg_width = (width1 + width2) / 2
    
    # 返回相对距离（距离除以平均宽度）
    return distance / avg_width if avg_width > 0 else 0

def calculate_iou(box1, box2):
    """计算两个边界框的IoU（交并比）"""
    # 提取坐标
    x1_1, y1_1, x2_1, y2_1 = box1
    x1_2, y1_2, x2_2, y2_2 = box2
    
    # 计算交集区域
    x1_i = max(x1_1, x1_2)
    y1_i = max(y1_1, y1_2)
    x2_i = min(x2_1, x2_2)
    y2_i = min(y2_1, y2_2)
    
    # 检查是否有交集
    if x2_i < x1_i or y2_i < y1_i:
        return 0.0
    
    # 计算交集面积
    intersection_area = (x2_i - x1_i) * (y2_i - y1_i)
    
    # 计算各自面积
    box1_area = (x2_1 - x1_1) * (y2_1 - y1_1)
    box2_area = (x2_2 - x1_2) * (y2_2 - y1_2)
    
    # 计算并集面积
    union_area = box1_area + box2_area - intersection_area
    
    # 返回IoU
    return intersection_area / union_area if union_area > 0 else 0.0

while True:
    # 读取摄像头帧
    ret, frame = cap.read()
    
    # 如果无法获取帧，退出循环
    if not ret:
        print("无法获取视频帧")
        break
    
    # 使用YOLOv8模型进行检测
    start_time = time.time()
    results = model(frame, conf=0.25, iou=0.45)  # 调整置信度和IOU阈值
    end_time = time.time()
    
    # 计算FPS
    time_diff = end_time - start_time
    fps = 1 / max(time_diff, 0.001)  # 确保分母不为零
    
    # 获取原始检测结果
    original_frame = frame.copy()
    
    # 处理检测结果
    elevator_buttons = []
    elevator_doors = []
    
    for r in results:
        boxes = r.boxes
        for box in boxes:
            # 获取类别ID和名称
            cls_id = int(box.cls[0].item())
            cls_name = model.names[cls_id]
            
            # 获取置信度
            conf = box.conf[0].item()
            
            # 获取边界框坐标
            x1, y1, x2, y2 = [int(val) for val in box.xyxy[0].tolist()]
            
            # 根据类别分类存储
            if cls_name == "Elevator Button":
                elevator_buttons.append((x1, y1, x2, y2, conf))
            elif cls_name == "Elevator Door":
                elevator_doors.append((x1, y1, x2, y2, conf))
    
    # 如果电梯门超过2个，只保留置信度最高的2个
    if len(elevator_doors) > 2:
        elevator_doors.sort(key=lambda x: x[4], reverse=True)  # 按置信度排序
        elevator_doors = elevator_doors[:2]  # 只保留前2个
    
    # 过滤掉与电梯门重叠的电梯按钮
    filtered_buttons = []
    for button in elevator_buttons:
        button_box = button[:4]
        is_overlapping = False
        
        for door in elevator_doors:
            door_box = door[:4]
            iou = calculate_iou(button_box, door_box)
            
            # 如果IoU大于阈值，认为有重叠
            if iou > 0.1:  # 可以调整这个阈值
                is_overlapping = True
                break
        
        if not is_overlapping:
            filtered_buttons.append(button)
    
    # 如果过滤后的电梯按钮超过1个，只保留置信度最高的1个
    if len(filtered_buttons) > 1:
        filtered_buttons.sort(key=lambda x: x[4], reverse=True)  # 按置信度排序
        filtered_buttons = filtered_buttons[:1]  # 只保留置信度最高的1个
    
    # 在原始帧上绘制结果
    annotated_frame = original_frame.copy()
    
    # 绘制电梯门并判断状态
    door_status = "未检测到电梯门"
    if len(elevator_doors) == 2:
        # 计算两个门之间的距离
        door1 = elevator_doors[0][:4]  # 第一个门的坐标
        door2 = elevator_doors[1][:4]  # 第二个门的坐标
        
        relative_distance = calculate_door_distance(door1, door2)
        
        # 判断门的状态
        if relative_distance > DOOR_OPEN_THRESHOLD:
            door_status = "Door Open"
            door_color = (0, 255, 255)  # 黄色
        else:
            door_status = "Door Close"
            door_color = (0, 0, 255)  # 红色
        
        # 绘制电梯门
        for x1, y1, x2, y2, conf in elevator_doors:
            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), door_color, 2)
            cv2.putText(annotated_frame, f"Door {conf:.2f}", (x1, y1 - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, door_color, 2)
        
        # 在两个门之间绘制连线
        center1 = ((door1[0] + door1[2]) // 2, (door1[1] + door1[3]) // 2)
        center2 = ((door2[0] + door2[2]) // 2, (door2[1] + door2[3]) // 2)
        cv2.line(annotated_frame, center1, center2, door_color, 2)
        
        # 显示相对距离
        mid_point = ((center1[0] + center2[0]) // 2, (center1[1] + center2[1]) // 2)
        cv2.putText(annotated_frame, f"Dist: {relative_distance:.2f}", mid_point, 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, door_color, 2)
    
    elif len(elevator_doors) == 1:
        # 只检测到一个门
        x1, y1, x2, y2, conf = elevator_doors[0]
        cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (255, 0, 0), 2)  # 蓝色
        cv2.putText(annotated_frame, f"Door {conf:.2f}", (x1, y1 - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        door_status = "One Door Detected"
    
    # 绘制电梯按钮（在门之后绘制，确保按钮不会覆盖门的标注）
    for x1, y1, x2, y2, conf in filtered_buttons:
        cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(annotated_frame, f"Button {conf:.2f}", (x1, y1 - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    # 添加状态信息
    cv2.putText(annotated_frame, door_status, (10, 60), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    # 添加FPS信息
    cv2.putText(annotated_frame, f"FPS: {fps:.2f}", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    # 显示检测到的对象数量
    cv2.putText(annotated_frame, f"Buttons: {len(filtered_buttons)}, Doors: {len(elevator_doors)}", 
                (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    # 显示结果
    cv2.imshow(window_name, annotated_frame)
    
    # 按'q'键退出
    if cv2.waitKey(1) == ord('q'):
        break

# 释放资源
cap.release()
cv2.destroyAllWindows()