#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
基础小车测试程序
用于验证小车的基本移动功能
"""

import time
import sys
import os

# 添加项目路径
sys.path.append(os.path.abspath(os.path.dirname(__file__)))

from vehicle import CarBase
from log_info import logger

def test_basic_movement():
    """
    测试小车基础移动功能
    """
    print("=== 基础小车移动测试 ===")
    
    try:
        # 创建小车实例
        print("正在初始化小车...")
        car = CarBase()
        print("小车初始化成功！")
        
        # 测试停止
        print("测试停止功能...")
        car.stop()
        time.sleep(1)
        
        # 测试前进
        print("测试前进功能...")
        car.set_velocity(0.2, 0, 0)  # 前进速度0.2m/s
        time.sleep(2)
        car.stop()
        time.sleep(1)
        
        # 测试后退
        print("测试后退功能...")
        car.set_velocity(-0.2, 0, 0)  # 后退速度0.2m/s
        time.sleep(2)
        car.stop()
        time.sleep(1)
        
        # 测试左转
        print("测试左转功能...")
        car.set_velocity(0, 0, 0.5)  # 左转角速度0.5rad/s
        time.sleep(2)
        car.stop()
        time.sleep(1)
        
        # 测试右转
        print("测试右转功能...")
        car.set_velocity(0, 0, -0.5)  # 右转角速度0.5rad/s
        time.sleep(2)
        car.stop()
        
        print("基础移动测试完成！")
        
        # 获取里程计信息
        odom = car.get_odometry()
        print(f"当前里程计信息: {odom}")
        
        # 关闭小车
        car.close()
        print("小车已关闭")
        
    except Exception as e:
        print(f"测试过程中出现错误: {e}")
        logger.error(f"基础移动测试失败: {e}")

def test_odometry():
    """
    测试里程计功能
    """
    print("=== 里程计测试 ===")
    
    try:
        car = CarBase()
        print("小车初始化成功！")
        
        # 记录初始位置
        initial_odom = car.get_odometry()
        print(f"初始位置: {initial_odom}")
        
        # 前进一段距离
        print("前进0.5米...")
        car.set_velocity(0.2, 0, 0)
        time.sleep(2.5)  # 0.2m/s * 2.5s = 0.5m
        car.stop()
        
        # 查看位置变化
        current_odom = car.get_odometry()
        print(f"当前位置: {current_odom}")
        
        # 计算移动距离
        distance = car.get_dis_traveled()
        print(f"移动距离: {distance:.3f}米")
        
        car.close()
        print("里程计测试完成！")
        
    except Exception as e:
        print(f"里程计测试失败: {e}")
        logger.error(f"里程计测试失败: {e}")

if __name__ == "__main__":
    print("开始基础小车功能测试...")
    
    # 测试基础移动
    test_basic_movement()
    
    print("\n" + "="*50 + "\n")
    
    # 测试里程计
    test_odometry()
    
    print("\n所有测试完成！") 