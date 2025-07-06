#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
简单车道线检测程序
使用传统的图像处理方法进行车道线检测
"""

import time
import sys
import os
import cv2
import numpy as np

# 添加项目路径
sys.path.append(os.path.abspath(os.path.dirname(__file__)))

from camera import Camera
from log_info import logger

class SimpleLaneDetector:
    """
    简单车道线检测器
    使用传统的图像处理方法
    """
    
    def __init__(self):
        """
        初始化车道线检测器
        """
        # 车道线检测参数
        self.lane_width = 100  # 车道线宽度（像素）
        self.center_x = 320     # 图像中心x坐标
        self.center_y = 480     # 图像中心y坐标
        
        # 颜色阈值（HSV空间）
        self.lower_white = np.array([0, 0, 200])
        self.upper_white = np.array([180, 30, 255])
        
        # 感兴趣区域（ROI）
        self.roi_vertices = np.array([
            [(0, 480), (0, 240), (640, 240), (640, 480)]
        ], dtype=np.int32)
    
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
    
    def extract_white_lines(self, image):
        """
        提取白色车道线
        
        Args:
            image: 输入图像
            
        Returns:
            白色车道线掩码
        """
        # 转换为HSV颜色空间
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 创建白色车道线掩码
        white_mask = cv2.inRange(hsv, self.lower_white, self.upper_white)
        
        return white_mask
    
    def apply_roi(self, image):
        """
        应用感兴趣区域
        
        Args:
            image: 输入图像
            
        Returns:
            应用ROI后的图像
        """
        # 创建掩码
        mask = np.zeros_like(image)
        cv2.fillPoly(mask, self.roi_vertices, 255)
        
        # 应用掩码
        masked_image = cv2.bitwise_and(image, mask)
        
        return masked_image
    
    def detect_lane_center(self, image):
        """
        检测车道中心
        
        Args:
            image: 输入图像
            
        Returns:
            (error_x, error_y): 车道中心偏移量
        """
        # 预处理图像
        processed = self.preprocess_image(image)
        
        # 应用ROI
        roi_image = self.apply_roi(processed)
        
        # 查找轮廓
        contours, _ = cv2.findContours(roi_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) == 0:
            return 0, 0  # 没有检测到车道线
        
        # 找到最大的轮廓（可能是车道线）
        largest_contour = max(contours, key=cv2.contourArea)
        
        # 计算轮廓的中心点
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
        else:
            cx, cy = self.center_x, self.center_y
        
        # 计算偏移量
        error_x = cx - self.center_x
        error_y = cy - self.center_y
        
        return error_x, error_y
    
    def visualize_detection(self, image, error_x, error_y):
        """
        可视化检测结果
        
        Args:
            image: 原始图像
            error_x: X轴偏移量
            error_y: Y轴偏移量
            
        Returns:
            可视化后的图像
        """
        # 复制图像
        vis_image = image.copy()
        
        # 绘制图像中心线
        cv2.line(vis_image, (self.center_x, 0), (self.center_x, image.shape[0]), (0, 255, 0), 2)
        
        # 绘制检测到的车道中心
        detected_x = self.center_x + error_x
        detected_y = self.center_y + error_y
        cv2.circle(vis_image, (detected_x, detected_y), 10, (0, 0, 255), -1)
        
        # 绘制偏移量
        cv2.putText(vis_image, f"Error X: {error_x}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(vis_image, f"Error Y: {error_y}", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return vis_image

def test_lane_detection():
    """
    测试车道线检测功能
    """
    print("=== 车道线检测测试 ===")
    
    try:
        # 初始化摄像头和检测器
        print("正在初始化摄像头和车道线检测器...")
        camera = Camera(1)
        detector = SimpleLaneDetector()
        print("初始化成功！")
        
        print("开始车道线检测测试...")
        print("按 'q' 键退出")
        
        while True:
            # 获取图像
            frame = camera.read()
            if frame is None:
                print("无法获取图像")
                break
            
            # 检测车道线
            error_x, error_y = detector.detect_lane_center(frame)
            
            # 可视化结果
            vis_frame = detector.visualize_detection(frame, error_x, error_y)
            
            # 显示图像
            cv2.imshow('Lane Detection', vis_frame)
            
            # 检查按键
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            
            # 打印检测结果
            print(f"车道线偏移量: X={error_x}, Y={error_y}")
            
            time.sleep(0.1)
        
        # 清理
        cv2.destroyAllWindows()
        camera.close()
        print("车道线检测测试完成！")
        
    except Exception as e:
        print(f"车道线检测测试失败: {e}")
        logger.error(f"车道线检测测试失败: {e}")

def test_lane_detection_with_car():
    """
    结合小车的车道线检测测试
    """
    print("=== 结合小车的车道线检测测试 ===")
    
    try:
        from vehicle import CarBase
        
        # 初始化小车和摄像头
        print("正在初始化小车和摄像头...")
        car = CarBase()
        camera = Camera(1)
        detector = SimpleLaneDetector()
        print("初始化成功！")
        
        print("开始车道线巡航测试...")
        print("按 'q' 键退出")
        
        # 巡航参数
        base_speed = 0.2  # 基础速度
        kp = 0.001       # 比例系数
        
        while True:
            # 获取图像
            frame = camera.read()
            if frame is None:
                print("无法获取图像")
                break
            
            # 检测车道线
            error_x, error_y = detector.detect_lane_center(frame)
            
            # 计算控制输出
            angular_velocity = kp * error_x
            
            # 限制角速度范围
            angular_velocity = np.clip(angular_velocity, -0.5, 0.5)
            
            # 设置小车速度
            car.set_velocity(base_speed, 0, angular_velocity)
            
            # 可视化结果
            vis_frame = detector.visualize_detection(frame, error_x, error_y)
            
            # 显示图像
            cv2.imshow('Lane Following', vis_frame)
            
            # 检查按键
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            
            # 打印控制信息
            print(f"偏移量: X={error_x}, 角速度: {angular_velocity:.3f}")
            
            time.sleep(0.1)
        
        # 停止小车
        car.stop()
        
        # 清理
        cv2.destroyAllWindows()
        camera.close()
        car.close()
        print("车道线巡航测试完成！")
        
    except Exception as e:
        print(f"车道线巡航测试失败: {e}")
        logger.error(f"车道线巡航测试失败: {e}")

if __name__ == "__main__":
    print("开始车道线检测功能测试...")
    
    # 测试基础车道线检测
    test_lane_detection()
    
    print("\n" + "="*50 + "\n")
    
    # 测试结合小车的车道线检测
    test_lane_detection_with_car()
    
    print("\n所有车道线检测测试完成！") 