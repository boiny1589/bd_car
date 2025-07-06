#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
智能小车控制系统 - car_wrap_copy.py

这是一个完整的智能小车控制系统，包含以下主要功能：

1. 车辆运动控制
   - 基础移动控制（时间、距离、传感器触发）
   - 车道线巡航
   - 目标检测巡航
   - 精确定位

2. 视觉感知系统
   - 车道线检测
   - 目标检测和识别
   - OCR文字识别
   - 相机标定和坐标转换

3. 传感器系统
   - 红外传感器避障
   - 按键控制
   - LED指示灯
   - 蜂鸣器提示

4. 任务执行系统
   - 机械臂控制
   - 发射装置
   - 任务调度

5. 人机交互
   - 菜单式界面
   - 按键响应
   - 屏幕显示

6. AI推理系统
   - 基于PaddlePaddle的推理引擎
   - 文心一言AI分析（可选）

主要类：
- MyCar: 主控制类，继承自CarBase
- PidCal2: 双PID控制器
- LanePidCal: 车道线PID控制器
- DetPidCal: 检测PID控制器
- LocatePidCal: 定位PID控制器

配置文件：config_car.yml
"""

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

def sellect_program(programs, order, win_order):
    """
    程序选择显示函数
    
    生成菜单显示字符串，支持滚动显示
    
    Args:
        programs: 程序列表
        order: 当前选中序号
        win_order: 窗口显示起始序号
        
    Returns:
        格式化的显示字符串
    """
    dis_str = ''
    start_index = 0
    
    start_index = order - win_order
    for i, program in enumerate(programs):
        if i < start_index:
            continue

        now = str(program)
        if i == order:
            now = '>>> ' + now
        else:
            now = str(i+1) + '.' + now
        if len(now) >= 19:
            now = now[:19]
        else:
            now = now + '\n'
        dis_str += now
        if i-start_index == 4:
            break
    return dis_str

def kill_other_python():
    """
    杀死其他Python进程
    
    用于确保只有一个主程序在运行，避免资源冲突
    """
    import psutil
    pid_me = os.getpid()
    # logger.info("my pid ", pid_me, type(pid_me))
    python_processes = []
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                if 'python' in proc.info['name'].lower() and len(proc.info['cmdline']) > 1 and len(proc.info['cmdline'][1]) < 30:
                    python_processes.append(proc.info)
            # 出现异常的时候捕获 不存在的异常，权限不足的异常， 僵尸进程
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass
    for process in python_processes:
        # logger.info(f"PID: {process['pid']}, Name: {process['name']}, Cmdline: {process['cmdline']}")
        # logger.info("this", process['pid'], type(process['pid']))
        if int(process['pid']) != pid_me:
            os.kill(int(process['pid']), signal.SIGKILL)
            time.sleep(0.3)
            
def limit(value, value_range):
    """
    限制数值范围
    
    Args:
        value: 输入值
        value_range: 范围限制
        
    Returns:
        限制后的值
    """
    return max(min(value, value_range), 0-value_range)

# 两个pid集合成一个
class PidCal2():
    """
    双PID控制器类
    
    同时控制两个PID参数，用于车道线巡航和目标检测巡航
    """
    def __init__(self, cfg_pid_y=None, cfg_pid_angle=None):
        """
        初始化双PID控制器
        
        Args:
            cfg_pid_y: Y轴PID配置
            cfg_pid_angle: 角度PID配置
        """
        self.pid_y = PID(**cfg_pid_y)
        self.pid_angle = PID(**cfg_pid_angle)
    
    def get_out(self, error_y, error_angle):
        """
        获取PID输出
        
        Args:
            error_y: Y轴误差
            error_angle: 角度误差
            
        Returns:
            (pid_y_out, pid_angle_out): PID输出值
        """
        pid_y_out = self.pid_y(error_y)
        pid_angle_out = self.pid_angle(error_angle)
        return pid_y_out, pid_angle_out

class LanePidCal():
    """
    车道线PID控制器类
    
    专门用于车道线巡航的PID控制器
    """
    def __init__(self, cfg_pid_y=None, cfg_pid_angle=None):
        """
        初始化车道线PID控制器
        
        Args:
            cfg_pid_y: Y轴PID配置
            cfg_pid_angle: 角度PID配置
        """
        # y_out_limit = 0.7
        # self.pid_y = PID(5, 0, 0)
        # self.pid_y.setpoint = 0
        # self.pid_y.output_limits = (-y_out_limit, y_out_limit)
        # print(cfg_pid_y)
        # print(cfg_pid_angle)
        self.pid_y = PID(**cfg_pid_y)
        # print(self.pid_y)

        angle_out_limit = 1.5
        self.pid_angle = PID(3, 0, 0)
        self.pid_angle.setpoint = 0
        self.pid_angle.output_limits = (-angle_out_limit, angle_out_limit)
    
    def get_out(self, error_y, error_angle):
        """
        获取车道线PID输出
        
        Args:
            error_y: Y轴误差
            error_angle: 角度误差
            
        Returns:
            (pid_y_out, pid_angle_out): PID输出值
        """
        pid_y_out = self.pid_y(error_y)
        pid_angle_out = self.pid_angle(error_angle)
        return pid_y_out, pid_angle_out
    
class DetPidCal():
    """
    检测PID控制器类
    
    专门用于目标检测巡航的PID控制器
    """
    def __init__(self, cfg_pid_y=None, cfg_pid_angle=None):
        """
        初始化检测PID控制器
        
        Args:
            cfg_pid_y: Y轴PID配置
            cfg_pid_angle: 角度PID配置
        """
        y_out_limit = 0.7
        self.pid_y = PID(0.3, 0, 0)
        self.pid_y.setpoint = 0
        self.pid_y.output_limits = (-y_out_limit, y_out_limit)

        angle_out_limit = 1.5
        self.pid_angle = PID(2, 0, 0)
        self.pid_angle.setpoint = 0
        self.pid_angle.output_limits = (-angle_out_limit, angle_out_limit)
    
    def get_out(self, error_y, error_angle):
        """
        获取检测PID输出
        
        Args:
            error_y: Y轴误差
            error_angle: 角度误差
            
        Returns:
            (pid_y_out, pid_angle_out): PID输出值
        """
        pid_y_out = self.pid_y(error_y)
        pid_angle_out = self.pid_angle(error_angle)
        return pid_y_out, pid_angle_out
    

class LocatePidCal():
    """
    定位PID控制器类
    
    专门用于精确定位的PID控制器
    """
    def __init__(self):
        """
        初始化定位PID控制器
        """
        y_out_limit = 0.3
        self.pid_y = PID(0.5, 0, 0)
        self.pid_y.setpoint = 0
        self.pid_y.output_limits = (-y_out_limit, y_out_limit)

        x_out_limit = 0.3
        self.pid_x = PID(0.5, 0, 0)
        self.pid_x.setpoint = 0
        self.pid_x.output_limits = (-x_out_limit, x_out_limit)
    
    def set_target(self, x, y):
        """
        设置目标位置
        
        Args:
            x: X轴目标
            y: Y轴目标
        """
        self.pid_y.setpoint = y
        self.pid_x.setpoint = x

    def get_out(self, error_x, error_y):
        """
        获取定位PID输出
        
        Args:
            error_x: X轴误差
            error_y: Y轴误差
            
        Returns:
            (pid_x_out, pid_y_out): PID输出值
        """
        pid_y_out = self.pid_y(error_y)
        pid_x_out = self.pid_x(error_x)
        return pid_x_out, pid_y_out

"""
智能小车主控制类 - MyCar

这个类是整个智能小车系统的核心控制类，继承自CarBase基类。
主要功能包括：
1. 车辆运动控制（巡航、定位、避障等）
2. 视觉感知（车道线检测、目标检测、OCR识别）
3. 传感器数据处理（红外传感器、按键、LED等）
4. 任务执行（机械臂操作、发射装置等）
5. 人机交互（菜单管理、按键响应）

主要组件：
- 底盘控制：继承自CarBase，提供基础运动功能
- 视觉系统：前后摄像头，支持车道线检测和目标识别
- 传感器系统：红外传感器、按键、LED指示灯、蜂鸣器
- 任务系统：机械臂、发射装置等执行机构
- 推理系统：基于PaddlePaddle的AI推理引擎
- 人机交互：屏幕显示、按键管理

配置管理：
- 通过config_car.yml配置文件管理各种参数
- 支持PID参数、摄像头配置、传感器配置等
"""
class MyCar(CarBase):
    STOP_PARAM = True  # 停止参数标志
    
    def __init__(self):
        """
        初始化智能小车系统
        
        初始化顺序：
        1. 调用父类CarBase初始化底盘系统
        2. 初始化任务系统（机械臂、发射装置等）
        3. 初始化显示系统
        4. 加载配置文件
        5. 初始化传感器系统
        6. 初始化PID控制器
        7. 初始化摄像头系统
        8. 初始化AI推理系统
        9. 启动按键监听线程
        """
        # 调用继承的初始化
        start_time = time.time()
        super(MyCar, self).__init__()
        logger.info("my car init ok {}".format(time.time() - start_time))
        
        # 任务系统初始化
        self.task = MyTask()
        
        # 显示系统初始化
        self.display = ScreenShow()
        
        # 获取配置文件路径
        self.path_dir = os.path.abspath(os.path.dirname(__file__))
        self.yaml_path = os.path.join(self.path_dir, "config_car.yml")
        
        # 加载配置文件
        cfg = get_yaml(self.yaml_path)
        
        # 初始化传感器系统
        self.sensor_init(cfg)

        # 初始化PID控制器
        self.car_pid_init(cfg)
        
        # 初始化蜂鸣器
        self.ring = Beep()
        
        # 初始化摄像头系统
        self.camera_init(cfg)
        
        # 初始化AI推理系统
        self.paddle_infer_init()
        
        # 文心一言分析初始化（可选）
        # self.ernie_bot_init()

        # 线程控制标志
        self._stop_flag = False  # 程序结束标志
        self._end_flag = False   # 按键线程结束标志
        
        # 启动按键监听线程
        self.thread_key = threading.Thread(target=self.key_thread_func)
        self.thread_key.setDaemon(True)
        self.thread_key.start()
        
        # 初始化完成提示音
        self.beep()
    
    def beep(self):
        """
        蜂鸣器提示音
        """
        self.ring.rings()
        time.sleep(0.2)

    def sensor_init(self, cfg):
        """
        初始化传感器系统
        
        Args:
            cfg: 配置文件中的传感器配置
        """
        cfg_sensor = cfg['io']
        self.key = Key4Btn(cfg_sensor['key'])        # 4按键控制器
        self.light = LedLight(cfg_sensor['light'])    # LED指示灯
        self.left_sensor = Infrared(cfg_sensor['left_sensor'])   # 左红外传感器
        self.right_sensor = Infrared(cfg_sensor['right_sensor']) # 右红外传感器
    
    def car_pid_init(self, cfg):
        """
        初始化PID控制器
        
        Args:
            cfg: 配置文件中的PID参数
        """
        self.lane_pid = PidCal2(**cfg['lane_pid'])  # 车道线巡航PID
        self.det_pid = PidCal2(**cfg['det_pid'])    # 目标检测PID

    def camera_init(self, cfg):
        """
        初始化摄像头系统
        
        Args:
            cfg: 配置文件中的摄像头配置
        """
        # 初始化前后摄像头
        self.cap_front = Camera(cfg['camera']['front'])  # 前摄像头
        self.cap_side = Camera(cfg['camera']['side'])    # 侧摄像头

    def paddle_infer_init(self):
        """
        初始化AI推理系统
        
        包括：
        - 车道线检测推理
        - 前向目标检测推理
        - 任务目标检测推理
        - OCR文字识别推理
        """
        self.crusie = ClintInterface('lane')    # 车道线检测
        self.front_det = ClintInterface('front') # 前向目标检测
        self.task_det = ClintInterface('task')   # 任务目标检测
        self.ocr_rec = ClintInterface('ocr')     # OCR文字识别
        self.last_det = None  # 上次检测结果

    def ernie_bot_init(self):
        """
        初始化文心一言AI分析系统（可选功能）
        """
        self.hum_analysis = ErnieBotWrap()
        self.hum_analysis.set_promt(str(HumAttrPrompt()))

        self.action_bot = ErnieBotWrap()
        self.action_bot.set_promt(str(ActionPrompt()))

    @staticmethod
    def get_cfg(path):
        """
        读取配置文件（静态方法）
        
        Args:
            path: 配置文件路径
        """
        from yaml import load, Loader
        # 把配置文件读取到内存
        with open(path, 'r') as stream:
            yaml_dict = load(stream, Loader=Loader)
        port_list = yaml_dict['port_io']
        # 转化为int
        for port in port_list:
            port['port'] = int(port['port'])

    def delay(self, time_hold):
        """
        延时函数，支持中断
        
        Args:
            time_hold: 延时时间（秒）
        """
        start_time = time.time()
        while True:
            if self._stop_flag:
                return
            if time.time() - start_time > time_hold:
                break
            
    def key_thread_func(self):
        """
        按键监听线程函数
        
        监听按键输入，支持程序中断
        """
        while True:
            if not self._stop_flag:
                if self._end_flag:
                    return
                key_val = self.key.get_key()
                if key_val == 3:
                    self._stop_flag = True
                time.sleep(0.2)
    
    
    @staticmethod
    def get_list_by_val(list, index, val):
        """
        根据某个值获取列表中匹配的结果
        
        Args:
            list: 待搜索列表
            index: 搜索索引
            val: 搜索值
            
        Returns:
            匹配的结果，未找到返回None
        """
        for det in list:
            if det[index] == val:
                return det
        return None
    
    def move_base(self, sp, end_fuction, stop=STOP_PARAM):
        """
        基础移动控制函数
        
        Args:
            sp: 速度向量 [vx, vy, vw]
            end_fuction: 结束条件函数
            stop: 是否在结束时停止
        """
        self.set_velocity(sp[0], sp[1], sp[2])
        while True:
            if self._stop_flag:
                return
            if end_fuction():
                break
            self.set_velocity(sp[0], sp[1], sp[2])
        if stop:
            self.set_velocity(0, 0, 0)

    def move_advance(self, sp, value_h=None, value_l=None, times=1, sides=1, dis_out=0.2, stop=STOP_PARAM):
        """
        高级移动控制：基于传感器的高级移动
        
        Args:
            sp: 速度向量
            value_h: 传感器高阈值
            value_l: 传感器低阈值
            times: 重复次数
            sides: 传感器选择（1=左，-1=右）
            dis_out: 距离阈值
            stop: 是否停止
        """
        if value_h is None:
            value_h = 1200
        if value_l is None:
            value_l = 0
        _sensor_usr = self.left_sensor
        if sides == -1:
            _sensor_usr = self.right_sensor
        # 用于检测开始过渡部分的标记
        flag_start = False
        def end_fuction():
            nonlocal flag_start
            val_sensor = _sensor_usr.read()
            if val_sensor < value_h and val_sensor > value_l:
                return flag_start
            else:
                flag_start = True
                return False
        for i in range(times):
            self.move_base(sp, end_fuction, stop=False)
        if stop:
            self.stop()

    
    def move_time(self, sp, dur_time=1, stop=STOP_PARAM):
        """
        按时间移动
        
        Args:
            sp: 速度向量
            dur_time: 持续时间
            stop: 是否停止
        """
        end_time = time.time() + dur_time
        end_func = lambda: time.time() > end_time
        self.move_base(sp, end_func, stop)

    def move_distance(self, sp, dis=0.1, stop=STOP_PARAM):
        """
        按距离移动
        
        Args:
            sp: 速度向量
            dis: 移动距离
            stop: 是否停止
        """
        end_dis = self.get_dis_traveled() + dis
        end_func = lambda: self.get_dis_traveled() > end_dis
        self.move_base(sp, end_func, stop)

    def calculation_dis(self, pos_dst, pos_src):
        """
        计算两个坐标的距离
        
        Args:
            pos_dst: 目标位置
            pos_src: 起始位置
            
        Returns:
            两点间距离
        """
        return math.sqrt((pos_dst[0] - pos_src[0])**2 + (pos_dst[1] - pos_src[1])**2)
    
    def det2pose(self, det, w_r=0.06): 
        """
        检测结果转换为实际位置
        
        使用相机标定参数将检测框转换为实际世界坐标
        
        Args:
            det: 检测框 [x, y, w, h]
            w_r: 物体实际宽度
            
        Returns:
            (x_r, y_r, rf_dis): 实际位置和距离
        """
        # r 真实  v 成像  f 焦点
        # rf 真实到焦点的距离  vf 相到焦点的距离
        vf_dis = 1.445
        x_v, y_v, w_v, h_v = det
        
        rf_dis = vf_dis * w_r / w_v
        x_r = x_v * rf_dis / vf_dis
        y_r = y_v * rf_dis / vf_dis
        return x_r, y_r, rf_dis
    
    def lane_det_location(self, speed, pts_tar=[[0, 70, 'text_det',  0, 0, 0, 0.70, 0.70]], dis_out=0.05, side=1, time_out=2, det='task'):
        """
        基于视觉的目标定位功能
        
        使用侧摄像头检测目标，并通过PID控制实现精确定位
        
        Args:
            speed: 移动速度
            pts_tar: 目标参数列表 [id, width, label, score, bbox]
            dis_out: 距离阈值
            side: 方向（1=正向，-1=反向）
            time_out: 超时时间
            det: 检测类型
            
        Returns:
            目标索引，失败返回False
        """
        end_time = time.time() + time_out
        infer = self.task_det
        loc_pid = get_yaml(self.yaml_path)["location_pid"]
        pid_x = PID(**loc_pid["pid_x"])
        pid_x.output_limits = (-speed, speed)
        pid_y = PID(**loc_pid["pid_y"])
        pid_y.output_limits = (-0.15, 0.15)

        # 用于相同记录结果的计数类
        x_count = CountRecord(5)
        dis_count = CountRecord(5)
        
        out_x = speed
        out_y = 0
        
        # 记录初始位置
        x_st, y_st, _ = self.get_odometry()
        find_tar = False
        tar = []
        
        # 处理目标参数
        for pt_tar in pts_tar:
            # id, 物体宽度，置信度, 归一化bbox[x_c, y_c, w, h]
            tar_id, tar_width, tar_label, tar_score, tar_bbox = pt_tar[0], pt_tar[1], pt_tar[2], pt_tar[3], pt_tar[4:]
            tar_width *= 0.001
            tar_x, tar_y, tar_dis = self.det2pose(tar_bbox, tar_width)
            tar.append([tar_id, tar_width, tar_x, tar_y, tar_dis])
            
        tar_id, tar_width, tar_x, tar_y, tar_dis = tar[0]
        pid_x.setpoint = tar_x
        pid_y.setpoint = tar_dis
        tar_index = 0
        
        while True:
            if self._stop_flag:
                return
            if time.time() > end_time:
                logger.info("time out")
                self.set_velocity(0, 0, 0)
                return False
                
            _pos_x, _pos_y, _pos_omage = self.get_odometry()

            # 检查是否超出距离限制
            if abs(_pos_x-x_st) > dis_out or abs(_pos_y-y_st) > dis_out:
                if not find_tar:
                    logger.info("task location dis out")
                    self.set_velocity(0, 0, 0)
                    return False
                    
            # 获取侧摄像头图像并进行检测
            img_side = self.cap_side.read()
            dets_ret = infer(img_side)
            
            # 按距离排序检测结果
            dets_ret.sort(key=lambda x: (x[4])**2 + (x[5])**2)
            
            if len(dets_ret) > 0:
                det = dets_ret[0]
                det_id, obj_id , det_label, det_score, det_bbox = det[0], det[1], det[2], det[3], det[4:]
                
                # 匹配目标
                for index, tar_pt in enumerate(tar):
                    if det_id == tar_pt[0]:
                        tar_index = index
                        tar_id, tar_width, tar_x, tar_y, tar_dis = tar_pt
                        pid_x.setpoint = tar_x
                        pid_y.setpoint = tar_dis
                        find_tar = True
                        break
                        
                if det_id == tar_id:
                    _x, _y, _dis = self.det2pose(det_bbox, tar_width)
                    out_x = pid_x(_x) * side
                    out_y = pid_y(_dis) * side
                    
                    # 检测偏差值连续小于阈值时，跳出循环
                    flag_x = x_count(abs(_x - tar_x) < 0.01)
                    flag_dis = dis_count(abs(_dis - tar_dis) < 0.01)
                    if flag_x:
                        out_x = 0
                    if flag_dis:
                        out_y = 0
                    if flag_x and flag_dis:
                        logger.info("location{} ok".format(tar_id))
                        self.set_velocity(0, 0, 0)
                        return tar_index
            else:
                x_count(False)
                dis_count(False)
            self.set_velocity(out_x, out_y, 0)
        
            
    def lane_base(self, speed, end_fuction, stop=STOP_PARAM):
        """
        基础车道线巡航功能
        
        Args:
            speed: 巡航速度
            end_fuction: 结束条件函数
            stop: 是否停止
        """
        while True:
            if self._stop_flag:
                return
            image = self.cap_front.read()
            error_y, error_angle = self.crusie(image)
            y_speed, angle_speed = self.lane_pid.get_out(-error_y, -error_angle)
            self.set_velocity(speed, y_speed, angle_speed)
            if end_fuction():
                break
        if stop:
            self.stop()

    def lane_det_base(self, speed, end_fuction, stop=STOP_PARAM):
        """
        基于目标检测的巡航功能
        
        Args:
            speed: 巡航速度
            end_fuction: 结束条件函数
            stop: 是否停止
        """
        y_speed = 0
        angle_speed = 0
        w_r=0.06
        
        while True:
            image = self.cap_front.read()
            dets_ret = self.front_det(image)
            
            if len(dets_ret)>0:
                det = dets_ret[0]
                det_cls, det_id, det_label, det_score, det_bbox = det[0], det[1], det[2], det[3], det[4:]
                _x, _y, _dis = self.det2pose(det_bbox, w_r)
                
                if end_fuction(_dis):
                    break
                error_angle = _x /_dis
                y_speed, angle_speed = self.det_pid.get_out(_x, error_angle)
                
            self.set_velocity(speed, y_speed, angle_speed)
        if stop:
            self.stop()
            
    def lane_det_time(self, speed, time_dur, stop=STOP_PARAM):
        """
        按时间进行目标检测巡航
        
        Args:
            speed: 巡航速度
            time_dur: 巡航时间
            stop: 是否停止
        """
        time_end = time.time() + time_dur
        end_fuction = lambda x: time.time() > time_end
        self.lane_det_base(speed, end_fuction, stop=stop)

    def lane_det_dis2pt(self, speed, dis_end, stop=STOP_PARAM):
        """
        按距离进行目标检测巡航
        
        Args:
            speed: 巡航速度
            dis_end: 目标距离
            stop: 是否停止
        """
        end_fuction = lambda x: x < dis_end and x != 0
        self.lane_det_base(speed, end_fuction, stop=stop)

    def lane_time(self, speed, time_dur, stop=STOP_PARAM):
        """
        按时间进行车道线巡航
        
        Args:
            speed: 巡航速度
            time_dur: 巡航时间
            stop: 是否停止
        """
        time_end = time.time() + time_dur
        end_fuction = lambda: time.time() > time_end
        self.lane_base(speed, end_fuction, stop=stop)
    
    def lane_dis(self, speed, dis_end, stop=STOP_PARAM):
        """
        按距离进行车道线巡航
        
        Args:
            speed: 巡航速度
            dis_end: 目标距离
            stop: 是否停止
        """
        end_fuction = lambda: self.get_dis_traveled() > dis_end
        self.lane_base(speed, end_fuction, stop=stop)

    def lane_dis_offset(self, speed, dis_hold, stop=STOP_PARAM):
        """
        按偏移距离进行车道线巡航
        
        Args:
            speed: 巡航速度
            dis_hold: 偏移距离
            stop: 是否停止
        """
        dis_start = self.get_dis_traveled()
        dis_stop = dis_start + dis_hold
        self.lane_dis(speed, dis_stop, stop=stop)

    def lane_sensor(self, speed, value_h=None, value_l=None, dis_offset=0.0, times=1, sides=1, stop=STOP_PARAM):
        """
        基于传感器的车道线巡航
        
        Args:
            speed: 巡航速度
            value_h: 传感器高阈值
            value_l: 传感器低阈值
            dis_offset: 偏移距离
            times: 重复次数
            sides: 传感器选择
            stop: 是否停止
        """
        if value_h is None:
            value_h = 1200
        if value_l is None:
            value_l = 0
        _sensor_usr = self.left_sensor
        if sides == -1:
            _sensor_usr = self.right_sensor
            
        flag_start = False
        def end_fuction():
            nonlocal flag_start
            val_sensor = _sensor_usr.read()
            if val_sensor < value_h and val_sensor > value_l:
                return flag_start
            else:
                flag_start = True
                return False

        for i in range(times):
            self.lane_base(speed, end_fuction, stop=False)
        self.lane_dis_offset(speed, dis_offset, stop=stop)

    def get_card_side(self):
        """
        检测卡片左右指示
        
        Returns:
            1: 左转, -1: 右转
        """
        count_side = CountRecord(3)
        while True:
            if self._stop_flag:
                return
            image = self.cap_front.read()
            dets_ret = self.front_det(image)
            if len(dets_ret) == 0:
                count_side(-1)
                continue
            det = dets_ret[0]
            det_cls, det_id, det_label, det_score, det_bbox = det[0], det[1], det[2], det[3], det[4:]
            # 连续检测超过3次
            if count_side(det_label):
                if det_label == 'turn_right':
                    return -1
                elif det_label == 'turn_left':
                    return 1
    

    def get_ocr(self, time_out=3):
        """
        OCR文字识别功能
        
        Args:
            time_out: 超时时间
            
        Returns:
            识别到的文字，失败返回None
        """
        time_stop = time.time() + time_out
        # 简单滤波,三次检测到相同的值，认为稳定并返回
        text_count = CountRecord(3)
        text_out = None
        while True:
            if self._stop_flag:
                return
            if time.time() > time_stop:
                return None
            img = self.cap_side.read()
            response = self.task_det(img)
            if len(response) > 0:
                for det in response:
                    det_cls_id, det_id, det_label, det_score, det_bbox = det[0], det[1], det[2], det[3], det[4:]
                    if det_cls_id == 0:
                        x1, y1, w, h = det_bbox
                        # 计算裁剪区域
                        x1 = img.shape[1] * (1+x1) / 2 - img.shape[1] * w / 4
                        x2 = x1 + img.shape[1] * w / 2
                        y1 = img.shape[0] * (1+y1) / 2 - img.shape[0] * w / 4
                        y2 = y1 + img.shape[0] * h / 2
                        x1 = 0 if x1 < 0 else int(x1)
                        x2 = img.shape[1] if x2 > img.shape[1] else int(x2)
                        y1 = 0 if y1 < 0 else int(y1)
                        y2 = img.shape[0] if y2 > img.shape[0] else int(y2)
                        
                        img_txt = img[y1:y2, x1:x2]
                        text = self.ocr_rec(img_txt)
                        if text_out==None:
                            text_out = text
                        else:
                            # 文本相似度比较
                            matcher = difflib.SequenceMatcher(None, text_out, text).ratio()
                            if text_count(matcher > 0.85):
                                return text_out
                            else:
                                text_out = text
            
    def yiyan_get_humattr(self, text):
        """
        使用文心一言获取人物属性分析
        
        Args:
            text: 输入文本
            
        Returns:
            分析结果
        """
        return self.hum_analysis.get_res_json(text)
    
    def yiyan_get_actions(self, text):
        """
        使用文心一言获取动作分析
        
        Args:
            text: 输入文本
            
        Returns:
            分析结果
        """
        return self.action_bot.get_res_json(text)
    
    def debug(self):
        """
        调试功能：实时显示各种传感器和检测结果
        """
        while True:
            if self._stop_flag:
                return
            image = self.cap_front.read()
            res = self.crusie(image)
            det_front = self.front_det(image)
            error = res[0]
            angle = res[1]
            image = self.cap_side.read()
            det_task = self.task_det(image)
            
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

    def walk_lane_test(self):
        """
        车道线巡航测试功能
        """
        end_function = lambda: True
        self.lane_base(0.3, end_function, stop=self.STOP_PARAM)

    def close(self):
        """
        关闭系统，释放资源
        """
        self._stop_flag = False
        self._end_flag = True
        self.thread_key.join()
        self.cap_front.close()
        self.cap_side.close()

    def manage(self, programs_list:list, order_index=0):
        """
        程序管理功能：提供菜单式的人机交互界面
        
        实现一个交互式菜单系统，允许用户通过按键选择和执行不同的程序。
        支持上下翻页、执行程序、退出等功能。
        
        Args:
            programs_list: 用户自定义程序列表
            order_index: 初始选中的程序索引
            
        按键说明：
        - 按键2: 向下选择程序
        - 按键4: 向上选择程序  
        - 按键3: 执行选中的程序
        - 长按1: 退出程序
        """
        def all_task():
            """执行所有任务函数"""
            time.sleep(4)
            for func in programs_list:
                func()
        
        def lane_test():
            """车道线测试函数"""
            self.lane_dis_offset(0.3, 30)

        # 添加系统默认程序到菜单
        programs_suffix = [all_task, lane_test, self.task.arm.reset, self.debug]
        programs = programs_list.copy()
        programs.extend(programs_suffix)
        # print(programs)
        # 选中的python脚本序号
        # 当前选中的序号
        win_num = 5  # 窗口显示的程序数量
        win_order = 0  # 窗口起始序号
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
            # self.button_all.event()
            btn = self.key.get_key()
            # 短按1=1,2=2,3=3,4=4
            # 长按1=5,2=6,3=7,4=8
            # logger.info(btn)
            # button_num = car.button_all.clicked()
            
            if btn != 0:
                # logger.info(btn)
                # 长按1按键，退出
                if btn == 5:
                    # run_flag = True
                    self._stop_flag = True
                    self._end_flag = True
                    break
                else:
                    if btn == 4:
                        # 序号减1（向上选择）
                        self.beep()
                        if order_index == 0:
                            order_index = len(programs)-1
                            win_order = win_num-1
                        else:
                            order_index -= 1
                            if win_order > 0:
                                win_order -= 1
                        # res = sllect_program(programs, num)
                        dis_str = sellect_program(programs_str, order_index, win_order)
                        self.display.show(dis_str)

                    elif btn == 2:
                        self.beep()
                        # 序号加1（向下选择）
                        if order_index == len(programs)-1:
                            order_index = 0
                            win_order = 0
                        else:
                            order_index += 1
                            if len(programs) < win_num:
                                win_num = len(programs)
                            if win_order != win_num-1:
                                win_order += 1
                        # res = sllect_program(programs, num)
                        dis_str = sellect_program(programs_str, order_index, win_order)
                        self.display.show(dis_str)

                    elif btn == 3:
                        # 确定执行选中的程序
                        dis_str = "\n{} running......\n".format(str(programs_str[order_index]))
                        self.display.show(dis_str)
                        self.beep()
                        self._stop_flag = False
                        programs[order_index]()
                        self._stop_flag = True
                        dis_str = sellect_program(programs_str, order_index, win_order)
                        self.stop()
                        self.beep()

                        # 自动跳转下一条（可选功能）
                        # if order_index == len(programs)-1:
                        #     order_index = 0
                        #     win_order = 0
                        # else:
                        #     order_index += 1
                        #     if len(programs) < win_num:
                        #         win_num = len(programs)
                        #     if win_order != win_num-1:
                        #         win_order += 1
                        # res = sllect_program(programs, num)
                        dis_str = sellect_program(programs_str, order_index, win_order)
                        self.display.show(dis_str)
                    logger.info(programs_str[order_index])
            else:
                self.delay(0.02)
                
            time.sleep(0.02)

        # 退出提示音
        for i in range(2):
            self.beep()
            time.sleep(0.4)
        time.sleep(0.1)
        self.close()

if __name__ == "__main__":
    """
    主程序入口
    
    程序启动流程：
    1. 可选：杀死其他Python进程（避免资源冲突）
    2. 创建MyCar实例并初始化
    3. 定义测试函数
    4. 启动菜单管理系统
    5. 程序结束，释放资源
    """
    # kill_other_python()  # 可选：杀死其他Python进程
    my_car = MyCar()  # 创建并初始化智能小车
    time.sleep(0.4)   # 等待初始化完成
    
    # 定义测试函数
    def start_det_loc():
        """
        目标定位测试函数
        
        测试车辆对圆柱体的精确定位功能
        """
        det1 = [15, 60, "cylinder3", 0, 0, 0, 0.47, 0.7]  # 小圆柱
        det2 = [14, 80, "cylinder2", 0, 0, 0, 0.69, 0.7]  # 中圆柱
        det3 = [13, 100, "cylinder1", 0,  0, 0, 0.77, 0.7] # 大圆柱
        dets = [det1, det2, det3]
        my_car.lane_det_location(0.2, dets)

    def lane_det_test():
        """
        目标检测巡航测试函数
        
        测试车辆跟随目标物体的巡航功能
        """
        my_car.lane_det_dis2pt(0.2, 0.16)

    def move_test():
        """
        基础移动测试函数
        
        测试车辆的基础移动控制功能
        """
        my_car.set_vel_time(0.3, 0, -0.6, 1)

    def ocr_test():
        """
        OCR文字识别测试函数
        
        测试车辆的文字识别功能
        """
        print(my_car.get_ocr())

    # 启动菜单管理系统，传入测试函数列表
    my_car.manage([start_det_loc, lane_det_test, move_test, ocr_test])
    
    # 以下是一些被注释的测试代码示例
    # my_car.lane_time(0.3, 5)  # 车道线巡航5秒
    
    # my_car.lane_dis_offset(0.3, 1.2)  # 车道线巡航1.2米
    # my_car.lane_sensor(0.3, 0.5)      # 基于传感器的巡航
    # my_car.debug()                     # 调试模式

    # 文心一言AI分析测试（需要先初始化）
    # text = "犯人没有带着眼镜，穿着短袖"
    # criminal_attr = my_car.hum_analysis.get_res_json(text)
    # print(criminal_attr)
    # my_car.task.reset()
    # pt_tar = my_car.task.punish_crimall(arm_set=True)
    # hum_attr = my_car.get_hum_attr(pt_tar)
    # print(hum_attr)
    # res_bool = my_car.compare_humattr(criminal_attr, hum_attr)
    # print(res_bool)
    # pt_tar = [0, 1, 'pedestrian',  0, 0.02, 0.4, 0.22, 0.82]
    # for i in range(4):
    #     my_car.set_pos_offset([0.07, 0, 0])
    #     my_car.lane_det_location(0.1, pt_tar, det="mot", side=-1)
    # my_car.close()
    # text = my_car.get_ocr()
    # print(text)
    # pt_tar = my_car.task.pick_up_ball(arm_set=True)
    # my_car.lane_det_location(0.1, pt_tar)
    
    my_car.close()  # 程序结束，释放资源
    
    # 其他测试代码示例
    # my_car.debug()
    # while True:
    #     text = my_car.get_ocr()
    #     print(text)

    # my_car.task.reset()
    # my_car.lane_advance(0.3, dis_offset=0.01, value_h=500, sides=-1)
    # my_car.lane_task_location(0.3, 2)
    # my_car.lane_time(0.3, 5)
    # my_car.debug()
    
    # my_car.debug()

            
    # my_car.task.pick_up_block()
    # my_car.task.put_down_self_block()
    # my_car.lane_time(0.2, 2)
    # my_car.lane_advance(0.3, dis_offset=0.01, value_h=500, sides=-1)
    # my_car.lane_task_location(0.3, 2)
    # my_car.task.pick_up_block()
    # my_car.close()
    # logger.info(time.time())
    # my_car.lane_task_location(0.3, 2)


    # my_car.debug()
    # programs = [func1, func2, func3, func4, func5, func6]
    # my_car.manage(programs)
    # import sys
    # test_ord = 0
    # if len(sys.argv) >= 2:
    #     test_ord = int(sys.argv[1])
    # logger.info("test:", test_ord)
    # car_test(test_ord)
