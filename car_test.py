#!/usr/bin/python
# -*- coding: utf-8 -*-
import time
import threading
import os
import numpy as np
from task_func import MyTask
from log_info import logger
from car_wrap import MyCar
from tools import CountRecord
import math
import sys, os
# 添加上本文件对应目录
sys.path.append(os.path.abspath(os.path.dirname(__file__))) 

if __name__ == "__main__":
    # kill_other_python()
    my_car = MyCar()
    my_car.STOP_PARAM = False
    # my_car.task.reset()
    

    def hanoi_tower_func():
        my_car.lane_dis_offset(0.3, 0.5)
        print(my_car.get_odometry())
        my_car.set_pose_offset([0.3,0,0], 1)
        print(my_car.get_odometry())
        det_side = my_car.lane_det_dis2pt(0.2, 0.19)
        side = my_car.get_card_side()
        print(side) 
        调整检测方向
        my_car.task.arm.switch_side(side*-1)
        
        # 调整车子朝向
        my_car.set_pose_offset([0, 0, math.pi/4*side], 1)
        logger.info(" set pose offset")
        
        # 第一个要抓取的圆柱
        cylinder_id = 1
        # 调整抓手位置，获取要抓取的圆柱信息
        logger.info("start pick up cylinder")
        pts = my_car.task.pick_up_cylinder(cylinder_id, True)
        # 走一段距离
        my_car.lane_dis_offset(0.3,0.66)
        
        # 第二次感应到侧面位置
        # my_car.lane_sensor(0.2, value_h=0.3, sides=side*-1)
        my_car.lane_sensor(0.2, value_h=0.3, sides=side*-1, stop=True)
        # return
        # 记录此时的位置
        pose_dict = {}
        pose_last = None
        for i in range(3):
            # 根据给定信息定位目标
            index = my_car.lane_det_location(0.2, pts, side=side*-1)
            my_car.beep()
            pose_dict[index] = my_car.get_odometry().copy()
            if i == 2:
                pose_last = my_car.get_odometry().copy()
            print(index)
            # pose_list.append([index, my_car.get_odometry().copy()])
            
            if i < 2:
                my_car.set_pose_offset([0.08, 0, 0])
                my_car.beep()
            
        print(pose_dict)
        # 根据识别到的位置调整方向位置
        # angle = math.atan((pose_dict[2][1] - pose_dict[0][1]) / (pose_dict[2][0] - pose_dict[0][0]))
        # print(angle)
        # my_car.set_pose_offset([0, 0, -angle])
        # 重新定位最后一个圆柱
        # my_car.lane_det_location(0.2, pts, side=side*-1)
        angle_det = my_car.get_odometry()[2]
        # 计算目的地终点坐标
        pose_end = [0, 0, angle_det]
        pose_end[0] = pose_last[0] + 0.12*math.cos(angle_det)
        pose_end[1] = pose_last[1] + 0.12*math.sin(angle_det)
        # print(det)
        # 调整到目的地
        # my_car.set_pose(det)
        for i in range(3):
            det = pose_dict[i]
            det[2] = angle_det
            my_car.set_pose(det)
            # my_car.lane_det_location(0.2, pts, side=side*-1)
            my_car.task.pick_up_cylinder(i)
            my_car.set_pose(pose_end)
            my_car.task.put_down_cylinder(i)
        # return
    functions = [hanoi_tower_func]
    my_car.manage(functions, 1)  # 注意这里数字改为1
