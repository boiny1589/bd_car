#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
简化版小车启动程序
只包含基础移动功能，不依赖摄像头和AI推理
"""

import time
import sys
import os

# 添加项目路径
sys.path.append(os.path.abspath(os.path.dirname(__file__)))

from vehicle import CarBase
from log_info import logger

class SimpleCar:
    """
    简化版小车控制类
    只包含基础移动功能
    """
    
    def __init__(self):
        """初始化小车"""
        print("正在初始化小车...")
        self.car = CarBase()
        print("小车初始化完成！")
    
    def basic_movement_test(self):
        """基础移动测试"""
        print("=== 基础移动测试 ===")
        
        # 前进2秒
        print("前进2秒...")
        self.car.set_velocity(0.2, 0, 0)
        time.sleep(2)
        self.car.stop()
        time.sleep(1)
        
        # 后退2秒
        print("后退2秒...")
        self.car.set_velocity(-0.2, 0, 0)
        time.sleep(2)
        self.car.stop()
        time.sleep(1)
        
        # 左转2秒
        print("左转2秒...")
        self.car.set_velocity(0, 0, 0.5)
        time.sleep(2)
        self.car.stop()
        time.sleep(1)
        
        # 右转2秒
        print("右转2秒...")
        self.car.set_velocity(0, 0, -0.5)
        time.sleep(2)
        self.car.stop()
        
        print("基础移动测试完成！")
    
    def square_movement_test(self):
        """正方形移动测试"""
        print("=== 正方形移动测试 ===")
        
        # 前进
        print("前进...")
        self.car.set_velocity(0.2, 0, 0)
        time.sleep(2)
        self.car.stop()
        time.sleep(1)
        
        # 左转90度
        print("左转90度...")
        self.car.set_velocity(0, 0, 0.5)
        time.sleep(1.57)  # 90度 = π/2 ≈ 1.57秒
        self.car.stop()
        time.sleep(1)
        
        # 前进
        print("前进...")
        self.car.set_velocity(0.2, 0, 0)
        time.sleep(2)
        self.car.stop()
        time.sleep(1)
        
        # 左转90度
        print("左转90度...")
        self.car.set_velocity(0, 0, 0.5)
        time.sleep(1.57)
        self.car.stop()
        time.sleep(1)
        
        # 前进
        print("前进...")
        self.car.set_velocity(0.2, 0, 0)
        time.sleep(2)
        self.car.stop()
        time.sleep(1)
        
        # 左转90度
        print("左转90度...")
        self.car.set_velocity(0, 0, 0.5)
        time.sleep(1.57)
        self.car.stop()
        time.sleep(1)
        
        # 前进
        print("前进...")
        self.car.set_velocity(0.2, 0, 0)
        time.sleep(2)
        self.car.stop()
        
        print("正方形移动测试完成！")
    
    def circle_movement_test(self):
        """圆形移动测试"""
        print("=== 圆形移动测试 ===")
        
        # 以0.2m/s线速度和0.5rad/s角速度做圆周运动
        print("开始圆形移动...")
        self.car.set_velocity(0.2, 0, 0.5)
        time.sleep(12.57)  # 2π ≈ 6.28秒，转两圈
        self.car.stop()
        
        print("圆形移动测试完成！")
    
    def odometry_test(self):
        """里程计测试"""
        print("=== 里程计测试 ===")
        
        # 记录初始位置
        initial_odom = self.car.get_odometry()
        print(f"初始位置: {initial_odom}")
        
        # 前进1米
        print("前进1米...")
        self.car.set_velocity(0.2, 0, 0)
        time.sleep(5)  # 0.2m/s * 5s = 1m
        self.car.stop()
        
        # 查看位置变化
        current_odom = self.car.get_odometry()
        print(f"当前位置: {current_odom}")
        
        # 计算移动距离
        distance = self.car.get_dis_traveled()
        print(f"移动距离: {distance:.3f}米")
        
        print("里程计测试完成！")
    
    def close(self):
        """关闭小车"""
        print("正在关闭小车...")
        self.car.close()
        print("小车已关闭")

def main():
    """主函数"""
    print("=== 简化版小车控制程序 ===")
    print("注意：此版本不包含摄像头和AI推理功能")
    print("只测试基础移动功能")
    print()
    
    try:
        # 创建小车实例
        car = SimpleCar()
        
        # 测试基础移动
        car.basic_movement_test()
        
        print("\n" + "="*50 + "\n")
        
        # 测试里程计
        car.odometry_test()
        
        print("\n" + "="*50 + "\n")
        
        # 询问是否进行更多测试
        while True:
            print("请选择测试项目：")
            print("1. 基础移动测试")
            print("2. 正方形移动测试")
            print("3. 圆形移动测试")
            print("4. 里程计测试")
            print("5. 退出")
            
            try:
                choice = input("请输入选择 (1-5): ").strip()
                
                if choice == '1':
                    car.basic_movement_test()
                elif choice == '2':
                    car.square_movement_test()
                elif choice == '3':
                    car.circle_movement_test()
                elif choice == '4':
                    car.odometry_test()
                elif choice == '5':
                    break
                else:
                    print("无效选择，请重新输入")
                    
            except KeyboardInterrupt:
                print("\n程序被用户中断")
                break
            except Exception as e:
                print(f"发生错误: {e}")
        
        # 关闭小车
        car.close()
        
    except Exception as e:
        print(f"程序运行出错: {e}")
        logger.error(f"简化版小车程序运行出错: {e}")

if __name__ == "__main__":
    main() 