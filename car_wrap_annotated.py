#!/usr/bin/python
# -*- coding: utf-8 -*-
import time
import threading
import os
import sys, os

# 添加上本地目录
sys.path.append(os.path.abspath(os.path.dirname(__file__))) 
import platform
import signal
from camera import Camera
import numpy as np
from vehicle import ArmBase, ScreenShow, Key4Btn, Infrared, LedLight,CarBase, Beep
from simple_pid import PID
import difflib
import cv2, math
from task_func import MyTask
from infer_cs import ClintInterface, Bbox
from ernie_bot import ErnieBotWrap, ActionPrompt, HumAttrPrompt
from tools import CountRecord, get_yaml, IndexWrap

from log_info import logger



class MyCar(CarBase):
    STOP_PARAM = True   #如果 stop=True，动作结束后会自动调用 self.set_velocity(0, 0, 0)，让小车停下来。
    def __init__(self):
        # 调用父类初始化，初始化底盘、传感器等基础硬件
        start_time = time.time()
        super(MyCar, self).__init__()
        logger.info("my car init ok {}".format(time.time() - start_time))
        # 初始化任务管理器
        self.task = MyTask()
        # 初始化显示屏
        self.display = ScreenShow()
        
        # 获取当前文件所在目录，便于后续加载配置文件
        self.path_dir = os.path.abspath(os.path.dirname(__file__))
        self.yaml_path = os.path.join(self.path_dir, "config_car.yml")
        # 加载车辆配置文件（如摄像头索引、PID参数等）
        cfg = get_yaml(self.yaml_path)
        # 初始化传感器（按配置文件设置IO口）
        self.sensor_init(cfg)
        # 初始化PID控制器
        self.car_pid_init(cfg)
        # 初始化蜂鸣器
        self.ring = Beep()
        # 初始化摄像头（前、侧），摄像头索引由配置文件决定
        # 操作说明：如遇摄像头无法打开，请检查config_car.yml中的camera.front和camera.side索引，
        # 并用测试脚本确认索引号是否正确，必要时更换USB口或摄像头
        self.camera_init(cfg)
        # 初始化AI推理接口（如车道线、前方检测、任务检测、OCR等）
        self.paddle_infer_init()
        # 文心一言分析初始化（如不需要可注释）
        # self.ernie_bot_init()

        # 相关临时变量设置
        # 程序结束标志
        self._stop_flag = False
        # 按键线程结束标志
        self._end_flag = False
        # 启动按键检测线程
        self.thread_key = threading.Thread(target=self.key_thread_func)
        self.thread_key.setDaemon(True)
        self.thread_key.start()
        
        # 上电提示音
        self.beep()
    
    def beep(self):
        self.ring.rings()
        time.sleep(0.2)

    def sensor_init(self, cfg):
        cfg_sensor = cfg['io']
        # 初始化按键、灯光、红外传感器
        self.key = Key4Btn(cfg_sensor['key'])
        self.light = LedLight(cfg_sensor['light'])
        self.left_sensor = Infrared(cfg_sensor['left_sensor'])
        self.right_sensor = Infrared(cfg_sensor['right_sensor'])
    
    def car_pid_init(self, cfg):
        # 初始化车道线和检测用的PID控制器
        self.lane_pid = PidCal2(**cfg['lane_pid'])
        self.det_pid = PidCal2(**cfg['det_pid'])

    def camera_init(self, cfg):
        # 初始化前后摄像头设置
        # cfg['camera']['front']和cfg['camera']['side']分别为前、侧摄像头的索引号
        # 操作说明：
        # 1. 若程序报“读取图像错误”，请先用cv2.VideoCapture(0/1/2)测试所有索引，找到能用的编号
        # 2. 修改config_car.yml中camera.front和camera.side为可用编号
        # 3. 确认摄像头在系统“相机”应用中能正常显示
        self.cap_front = Camera(cfg['camera']['front'])
        # 侧面摄像头
        self.cap_side = Camera(cfg['camera']['side'])

    def paddle_infer_init(self):
        # 初始化AI推理接口，分别用于车道线、前方、任务、OCR识别
        # 操作说明：如AI推理相关功能报错，需检查infer_cs/base/infer_front.py和后端推理脚本是否正常
        self.crusie = ClintInterface('lane')
        # 前置左右方向识别
        self.front_det = ClintInterface('front')
        # 任务识别
        self.task_det = ClintInterface('task')
        # ocr识别
        self.ocr_rec = ClintInterface('ocr')
        # 识别为None
        self.last_det = None

    # ... 其余方法保持原样 ...
    # 建议在 lane_base、debug 等摄像头/AI相关方法前也加类似注释
    def lane_base(self, speed, end_fuction, stop=STOP_PARAM):
        # 基于AI车道线识别的巡航控制
        # 操作说明：
        # 1. 读取前摄像头图像
        # 2. 送入AI推理接口，获得车道线偏移和角度误差
        # 3. 用PID控制器计算修正速度，控制底盘运动
        while True:
            if self._stop_flag:
                return
            image = self.cap_front.read()  # 读取前摄像头图像，若此处报错请优先排查摄像头索引和物理连接
            error_y, error_angle = self.crusie(image)
            y_speed, angle_speed = self.lane_pid.get_out(-error_y, -error_angle)
            self.set_velocity(speed, y_speed, angle_speed)
            if end_fuction():
                break
        if stop:
            self.stop()

    def debug(self):
        # 调试函数，循环读取摄像头、AI推理、传感器等信息并打印，便于排查问题
        while True:
            if self._stop_flag:
                return
            image = self.cap_front.read()  # 读取前摄像头图像
            res = self.crusie(image)       # AI车道线推理
            det_front = self.front_det(image)  # 前方物体检测
            error = res[0]
            angle = res[1]
            image = self.cap_side.read()   # 读取侧摄像头图像
            det_task = self.task_det(image)    # 任务检测
            # 日志输出各项信息，便于调试
            logger.info("")
            logger.info("--------------")
            logger.info("error:{} angle{}".format(error, angle))
            logger.info("front:{}".format(det_front))
            det_task.sort(key=lambda x: (x[4])**2 + (x[5])**2)
            logger.info("task:{}".format(det_task))
            if len(det_task) > 0:
                for det in det_task:
                    dis = self.det2pose(det[4:])
                    logger.info("det:{} dis:{}".format(det, dis))
            logger.info("left:{} right:{}".format(self.left_sensor.read(), self.right_sensor.read()))
            self.delay(1)

    # ================== 以下为常用测试与主控函数 ==================

    def walk_lane_test(self):
        """
        简单车道线巡航测试函数。
        用法：调用后小车会以0.3速度巡航，直到收到停止信号。
        适合用于基础调试和验证AI巡线功能。
        """
        end_function = lambda: True
        self.lane_base(0.3, end_function, stop=self.STOP_PARAM)

    def close(self):
        """
        关闭小车所有资源（线程、摄像头等）。
        用法：程序退出前务必调用，防止资源泄漏或摄像头无法再次打开。
        """
        self._stop_flag = False
        self._end_flag = True
        self.thread_key.join()
        self.cap_front.close()
        self.cap_side.close()
        # self.grap_cam.close()

    def manage(self, programs_list:list, order_index=0):
        """
        菜单管理主控函数。
        用法：
        - 传入一个函数列表，自动生成菜单，按键选择并执行。
        - 支持上下切换、确定执行、长按退出。
        - 可扩展：只需将自定义测试函数加入programs_list即可。
        """
        def all_task():
            time.sleep(4)
            for func in programs_list:
                func()
        
        def lane_test():
            self.lane_dis_offset(0.3, 30)

        # 菜单后缀：可选的内置测试项
        programs_suffix = [all_task, lane_test, self.task.arm.reset, self.debug]
        programs = programs_list.copy()
        programs.extend(programs_suffix)
        # print(programs)
        # 选中的python脚本序号
        # 当前选中的序号
        win_num = 5
        win_order = 0
        # 把programs的函数名转字符串
        logger.info(order_index)
        programs_str = [str(i.__name__) for i in programs]
        logger.info(programs_str)
        dis_str = sellect_program(programs_str, order_index, win_order)
        self.display.show(dis_str)

        self.stop()
        run_flag = False
        stop_flag = False
        stop_count = 0
        while True:
            # 按键检测主循环，支持上下切换、确定、退出
            btn = self.key.get_key()
            # 短按1=1,2=2,3=3,4=4
            # 长按1=5,2=6,3=7,4=8
            if btn != 0:
                # 长按1按键，退出
                if btn == 5:
                    self._stop_flag = True
                    self._end_flag = True
                    break
                else:
                    if btn == 4:
                        # 序号减1
                        self.beep()
                        if order_index == 0:
                            order_index = len(programs)-1
                            win_order = win_num-1
                        else:
                            order_index -= 1
                            if win_order > 0:
                                win_order -= 1
                        dis_str = sellect_program(programs_str, order_index, win_order)
                        self.display.show(dis_str)

                    elif btn == 2:
                        self.beep()
                        # 序号加1
                        if order_index == len(programs)-1:
                            order_index = 0
                            win_order = 0
                        else:
                            order_index += 1
                            if len(programs) < win_num:
                                win_num = len(programs)
                            if win_order != win_num-1:
                                win_order += 1
                        dis_str = sellect_program(programs_str, order_index, win_order)
                        self.display.show(dis_str)

                    elif btn == 3:
                        # 确定执行，调用选中程序
                        dis_str = "\n{} running......\n".format(str(programs_str[order_index]))
                        self.display.show(dis_str)
                        self.beep()
                        self._stop_flag = False
                        programs[order_index]()
                        self._stop_flag = True
                        dis_str = sellect_program(programs_str, order_index, win_order)
                        self.stop()
                        self.beep()
                        dis_str = sellect_program(programs_str, order_index, win_order)
                        self.display.show(dis_str)
                    logger.info(programs_str[order_index])
            else:
                self.delay(0.02)
            time.sleep(0.02)

        for i in range(2):
            self.beep()
            time.sleep(0.4)
        time.sleep(0.1)
        self.close()

# ================== 主程序入口 ==================
if __name__ == "__main__":
    # 启动主控程序，建议直接用 python car_wrap_annotated.py 运行
    # 如需自定义测试项，可在下方添加自定义函数并加入 manage 的列表
    # 常用扩展方式：
    # 1. 新建自己的测试函数（如 def my_test(): ...）
    # 2. 在 my_car.manage([...]) 里添加自己的函数名
    # 3. 运行后通过菜单选择执行
    #
    # 例：python car_wrap_annotated.py
    #
    # 注意：如需调试摄像头、AI等，建议先用 walk_lane_test、debug 等基础项排查

    # kill_other_python()  # 如需自动关闭其他python进程可取消注释
    my_car = MyCar()
    time.sleep(0.4)
    # my_car.task.get_ingredients(1, arm_set=True)

    def start_det_loc():
        """侧面摄像头定位测试"""
        det1 = [15, 60, "cylinder3", 0, 0, 0, 0.47, 0.7]
        det2 = [14, 80, "cylinder2", 0, 0, 0, 0.69, 0.7]
        det3 = [13, 100, "cylinder1", 0,  0, 0, 0.77, 0.7]          
        dets = [det1, det2, det3]
        my_car.lane_det_location(0.2, dets)

    def lane_det_test():
        """前摄像头检测距离测试"""
        my_car.lane_det_dis2pt(0.2, 0.16)

    def move_test():
        """底盘基础动作测试"""
        my_car.set_vel_time(0.3, 0, -0.6, 1)

    def ocr_test():
        """OCR识别测试"""
        print(my_car.get_ocr())

    # 将自定义测试函数加入菜单
    my_car.manage([start_det_loc, lane_det_test, move_test, ocr_test])
    # my_car.lane_time(0.3, 5)  # 其它测试可按需取消注释
    # my_car.lane_dis_offset(0.3, 1.2)
    # my_car.lane_sensor(0.3, 0.5)
    # my_car.debug() 

    # 其它高级用法和调试建议见下方注释
    # ...
