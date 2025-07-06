#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
摄像头测试程序
用于验证摄像头图像获取功能
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

def test_camera_basic():
    """
    测试摄像头基础功能
    """
    print("=== 摄像头基础测试 ===")
    
    try:
        # 测试前摄像头
        print("正在初始化前摄像头...")
        front_camera = Camera(1)  # 前摄像头设备号1
        print("前摄像头初始化成功！")
        
        # 获取图像
        print("获取前摄像头图像...")
        for i in range(5):
            frame = front_camera.read()
            if frame is not None:
                print(f"第{i+1}帧图像尺寸: {frame.shape}")
                
                # 显示图像
                cv2.imshow('Front Camera', frame)
                cv2.waitKey(1000)  # 显示1秒
            else:
                print(f"第{i+1}帧获取失败")
            time.sleep(0.5)
        
        cv2.destroyAllWindows()
        
        # 关闭摄像头
        front_camera.close()
        print("前摄像头测试完成！")
        
    except Exception as e:
        print(f"前摄像头测试失败: {e}")
        logger.error(f"前摄像头测试失败: {e}")

def test_dual_cameras():
    """
    测试双摄像头功能
    """
    print("=== 双摄像头测试 ===")
    
    try:
        # 初始化两个摄像头
        print("正在初始化双摄像头...")
        front_camera = Camera(1)  # 前摄像头
        side_camera = Camera(2)   # 侧摄像头
        print("双摄像头初始化成功！")
        
        # 同时获取两个摄像头的图像
        print("同时获取双摄像头图像...")
        for i in range(3):
            front_frame = front_camera.read()
            side_frame = side_camera.read()
            
            if front_frame is not None and side_frame is not None:
                print(f"第{i+1}帧 - 前摄像头: {front_frame.shape}, 侧摄像头: {side_frame.shape}")
                
                # 显示图像
                cv2.imshow('Front Camera', front_frame)
                cv2.imshow('Side Camera', side_frame)
                cv2.waitKey(2000)  # 显示2秒
            else:
                print(f"第{i+1}帧获取失败")
            time.sleep(1)
        
        cv2.destroyAllWindows()
        
        # 关闭摄像头
        front_camera.close()
        side_camera.close()
        print("双摄像头测试完成！")
        
    except Exception as e:
        print(f"双摄像头测试失败: {e}")
        logger.error(f"双摄像头测试失败: {e}")

def test_camera_parameters():
    """
    测试摄像头参数设置
    """
    print("=== 摄像头参数测试 ===")
    
    try:
        camera = Camera(1)
        print("摄像头初始化成功！")
        
        # 获取摄像头参数
        print("获取摄像头参数...")
        width = int(camera.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(camera.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = camera.cap.get(cv2.CAP_PROP_FPS)
        
        print(f"摄像头分辨率: {width}x{height}")
        print(f"摄像头帧率: {fps}")
        
        # 设置摄像头参数
        print("设置摄像头参数...")
        camera.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        camera.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # 验证设置
        new_width = int(camera.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        new_height = int(camera.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"设置后分辨率: {new_width}x{new_height}")
        
        # 获取一帧图像验证
        frame = camera.read()
        if frame is not None:
            print(f"实际图像尺寸: {frame.shape}")
            cv2.imshow('Test Frame', frame)
            cv2.waitKey(2000)
        
        cv2.destroyAllWindows()
        camera.close()
        print("摄像头参数测试完成！")
        
    except Exception as e:
        print(f"摄像头参数测试失败: {e}")
        logger.error(f"摄像头参数测试失败: {e}")

if __name__ == "__main__":
    print("开始摄像头功能测试...")
    
    # 测试基础摄像头功能
    test_camera_basic()
    
    print("\n" + "="*50 + "\n")
    
    # 测试双摄像头功能
    test_dual_cameras()
    
    print("\n" + "="*50 + "\n")
    
    # 测试摄像头参数
    test_camera_parameters()
    
    print("\n所有摄像头测试完成！") 