from vehicle import ArmBase, ScreenShow, Key4Btn, ServoBus, ServoPwm, MotorWrap, StepperWrap, PoutD
import cv2
import time
import numpy as np
import yaml, os, math
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
    # my_car.task.reset() # 注释掉或删除这行
    my_car.arm.reset_horiz_only() # 调用只复位水平方向的方法
    my_car.arm.set_arm_pose([0, 0, 0])
    logger.info("set_arm_pose")
    my_car.arm.set_arm_angle(0)
    my_car.arm.set_arm_dir(0)

    # my_car.task.reset()