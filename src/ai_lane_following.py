#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
AI巡线系统
完整的智能小车车道线跟随系统
"""

import time
import sys
import os
import cv2
import numpy as np
import threading
from simple_pid import PID

# 添加项目路径
sys.path.append(os.path.abspath(os.path.dirname(__file__)))

from vehicle import CarBase
from camera import Camera
from log_info import logger

class AILaneFollowingSystem:
    """
    AI巡线系统
    整合小车控制、摄像头和车道线检测
    """
    
    def __init__(self):
        """
        初始化AI巡线系统
        """
        print("正在初始化AI巡线系统...")
        
        # 初始化小车
        self.car = CarBase()
        print("小车初始化完成")
        
        # 初始化摄像头
        self.camera = Camera(1)  # 前摄像头
        print("摄像头初始化完成")
        
        # 初始化PID控制器
        self.pid_x = PID(Kp=0.001, Ki=0.0, Kd=0.0, setpoint=0)
        self.pid_x.output_limits = (-0.5, 0.5)  # 限制角速度范围
        
        # 系统状态
        self.running = False
        self.stop_flag = False
        
        # 控制参数
        self.base_speed = 0.2  # 基础速度
        self.max_speed = 0.4   # 最大速度
        self.min_speed = 0.1   # 最小速度
        
        # 车道线检测参数
        self.center_x = 320
        self.center_y = 480
        
        print("AI巡线系统初始化完成！")
    
    def preprocess_image(self, image):
        """
        图像预处理
        
        Args:
            image: 输入图像
            
        Returns:
            预处理后的图像
        """
        # 转换为灰度图
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 高斯模糊
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # 边缘检测
        edges = cv2.Canny(blurred, 50, 150)
        
        return edges
    
    def detect_lane_center(self, image):
        """
        检测车道中心
        
        Args:
            image: 输入图像
            
        Returns:
            error_x: 车道中心偏移量
        """
        # 预处理图像
        processed = self.preprocess_image(image)
        
        # 定义感兴趣区域（ROI）
        height, width = processed.shape
        roi_vertices = np.array([
            [(0, height), (0, height//2), (width, height//2), (width, height)]
        ], dtype=np.int32)
        
        # 应用ROI
        mask = np.zeros_like(processed)
        cv2.fillPoly(mask, roi_vertices, 255)
        roi_image = cv2.bitwise_and(processed, mask)
        
        # 查找轮廓
        contours, _ = cv2.findContours(roi_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) == 0:
            return 0  # 没有检测到车道线
        
        # 找到最大的轮廓（可能是车道线）
        largest_contour = max(contours, key=cv2.contourArea)
        
        # 计算轮廓的中心点
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
        else:
            cx = self.center_x
        
        # 计算偏移量
        error_x = cx - self.center_x
        
        return error_x
    
    def calculate_control_output(self, error_x):
        """
        计算控制输出
        
        Args:
            error_x: 车道线偏移量
            
        Returns:
            (linear_speed, angular_speed): 线速度和角速度
        """
        # PID控制
        angular_speed = self.pid_x(error_x)
        
        # 根据偏移量调整线速度
        # 偏移量越大，速度越慢
        speed_factor = 1.0 - abs(error_x) / 200.0  # 归一化偏移量
        speed_factor = np.clip(speed_factor, 0.5, 1.0)  # 限制速度因子范围
        
        linear_speed = self.base_speed * speed_factor
        
        return linear_speed, angular_speed
    
    def visualize_detection(self, image, error_x, linear_speed, angular_speed):
        """
        可视化检测结果
        
        Args:
            image: 原始图像
            error_x: X轴偏移量
            linear_speed: 线速度
            angular_speed: 角速度
            
        Returns:
            可视化后的图像
        """
        # 复制图像
        vis_image = image.copy()
        
        # 绘制图像中心线
        cv2.line(vis_image, (self.center_x, 0), (self.center_x, image.shape[0]), (0, 255, 0), 2)
        
        # 绘制检测到的车道中心
        detected_x = self.center_x + error_x
        cv2.circle(vis_image, (detected_x, self.center_y), 10, (0, 0, 255), -1)
        
        # 绘制控制信息
        cv2.putText(vis_image, f"Error X: {error_x:.1f}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(vis_image, f"Linear Speed: {linear_speed:.2f}", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(vis_image, f"Angular Speed: {angular_speed:.3f}", (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return vis_image
    
    def run_lane_following(self):
        """
        运行车道线跟随
        """
        print("开始AI巡线...")
        print("按 'q' 键退出")
        
        self.running = True
        
        try:
            while not self.stop_flag:
                # 获取图像
                frame = self.camera.read()
                if frame is None:
                    print("无法获取图像")
                    break
                
                # 检测车道线
                error_x = self.detect_lane_center(frame)
                
                # 计算控制输出
                linear_speed, angular_speed = self.calculate_control_output(error_x)
                
                # 设置小车速度
                self.car.set_velocity(linear_speed, 0, angular_speed)
                
                # 可视化结果
                vis_frame = self.visualize_detection(frame, error_x, linear_speed, angular_speed)
                
                # 显示图像
                cv2.imshow('AI Lane Following', vis_frame)
                
                # 检查按键
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                
                # 打印控制信息
                print(f"偏移量: {error_x:.1f}, 线速度: {linear_speed:.2f}, 角速度: {angular_speed:.3f}")
                
                time.sleep(0.05)  # 20Hz控制频率
        
        except Exception as e:
            print(f"巡线过程中出现错误: {e}")
            logger.error(f"巡线过程中出现错误: {e}")
        
        finally:
            # 停止小车
            self.car.stop()
            self.running = False
    
    def stop(self):
        """
        停止系统
        """
        print("正在停止AI巡线系统...")
        self.stop_flag = True
        self.car.stop()
        cv2.destroyAllWindows()
        self.camera.close()
        self.car.close()
        print("AI巡线系统已停止")

def test_ai_lane_following():
    """
    测试AI巡线系统
    """
    print("=== AI巡线系统测试 ===")
    
    try:
        # 创建AI巡线系统
        system = AILaneFollowingSystem()
        
        # 运行巡线
        system.run_lane_following()
        
        # 停止系统
        system.stop()
        
        print("AI巡线系统测试完成！")
        
    except Exception as e:
        print(f"AI巡线系统测试失败: {e}")
        logger.error(f"AI巡线系统测试失败: {e}")

def test_system_components():
    """
    测试系统组件
    """
    print("=== 系统组件测试 ===")
    
    try:
        # 测试小车
        print("测试小车组件...")
        car = CarBase()
        car.set_velocity(0.1, 0, 0)
        time.sleep(1)
        car.stop()
        car.close()
        print("小车组件测试完成")
        
        # 测试摄像头
        print("测试摄像头组件...")
        camera = Camera(1)
        frame = camera.read()
        if frame is not None:
            print(f"摄像头图像尺寸: {frame.shape}")
        camera.close()
        print("摄像头组件测试完成")
        
        print("所有系统组件测试完成！")
        
    except Exception as e:
        print(f"系统组件测试失败: {e}")
        logger.error(f"系统组件测试失败: {e}")

if __name__ == "__main__":
    print("开始AI巡线系统测试...")
    
    # 测试系统组件
    test_system_components()
    
    print("\n" + "="*50 + "\n")
    
    # 测试AI巡线系统
    test_ai_lane_following()
    
    print("\n所有测试完成！") 