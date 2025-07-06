#!/usr/bin/python3
# -*- coding: utf-8 -*-
# 开始编码格式和运行环境选择

import math, threading
import numpy as np


from threading import Thread
import yaml, os, sys

import time
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))) 
# 导入自定义log模块
from log_info import logger
from vehicle import MotorConvert, Motors, WheelWrap
from tools import PID

# 把该文件夹目录加入环境变量
sys.path.append(os.path.abspath(os.path.dirname(__file__)))

def get_path_relative(*args):
    local_dir = os.path.abspath(os.path.dirname(__file__))
    return os.path.join(local_dir, *args)
    
class OdometryBase:
    def __init__(self) -> None:
        # x, y, theta
        self.pose = np.array([.0, .0, .0])
        self.twist = np.array([.0, .0, .0])
        # 车子整体前进的路程变量
        self.dis_traveled = 0

    # odometry update,间隔时间不宜过长
    def odom_update(self, d_vect):
        # 位置变化矩阵
        z_angle = self.pose[2]
        d_pose_transform = np.array([[math.cos(z_angle), math.sin(z_angle)], 
                                    [-math.sin(z_angle), math.cos(z_angle)]])
        # print("z_angle:", z_angle, "d_pose_transform:", d_pose_transform)
        # 车子坐标变化转为世界坐标变化
        d_pose_xy = np.dot(d_vect[:2], d_pose_transform)
        # print("vect:",d_vect, "tans:",d_pose_xy)
        # 更新路程
        self.dis_traveled += np.sum(d_vect[:2]**2, keepdims=True)**0.5
        # 增加角度变化量
        d_pose = np.append(d_pose_xy, values=d_vect[2])
        # self.twist[2] += d_pose[2]
        # 更新世界坐标位置
        self.pose += d_pose
            
# 底盘功能抽象
class ChassisBase:
    def __init__(self):
        self.odom = OdometryBase()
        self.wheel_radius = 0.1
    
    def params_init(self):
        # 运动正解逆解转换矩阵参数初始化,可能包含尺寸参数
        self.transform_forward = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        self.transform_inverse = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

    def get_velocity(self, linear_vx, linear_vy, angular_v):
        """
        设置小车的线速度和角速度.
        参数:
        linear_vx (float): x轴线速度,
        linear_vy (float): y轴线速度
        angular_v (float): 角速度,范围为 [-max_angular_speed, max_angular_speed]
        """
        pass

    def forward_cal(self, wheel_vel):
        return np.dot(wheel_vel, self.transform_forward)
    
    def inverse_cal(self, car_vel):
        return np.dot(car_vel, self.transform_inverse)

    def get_velocity(self, linear_vx, linear_vy, angular_v):
        # 计算小车每个轮子的线速度
        wheel_vel = self.inverse_cal(np.array([linear_vx, linear_vy, angular_v]))
        return wheel_vel
        # 计算每个小车轮子的角速度
        wheel_angular = wheel_vel / self.wheel_radius
        return wheel_angular

    def updata_odom(self, wheel_vect):
        # 计算小车的位置变化
        car_vect = self.forward_cal(wheel_vect)
        # print("car_vect:", car_vect)
        # 更新小车的位姿
        self.odom.odom_update(car_vect)

class TricycleChassis(ChassisBase):
    '''
    wheel list
        *****A*****
        ***********
        ***B***C***
    '''
    def __init__(self, raduis_base=0.1) -> None:
        super().__init__()
        # 三轮全向车轮子到中心的距离半径
        self.radius = raduis_base
        self.params_init()

    def params_init(self):
        r = self.radius
        cos_a = math.cos(math.pi/6)
        sin_a = math.sin(math.pi/6)
        # 根据小车三轮的运动计算小车运动， 正解
        self.transform_forward = np.array([[           0,  2/3, 1/(3*r)],
                                           [-1/(2*cos_a), -1/3, 1/(3*r)],
                                           [ 1/(2*cos_a), -1/3, 1/(3*r)]])
        # 根据小车运动计算小车三轮的运动， 逆解
        self.transform_inverse = np.array([[0, -cos_a,  cos_a],
                                           [1, -sin_a, -sin_a],
                                           [r,      r,      r]])
        

# 差速车定义
class Diff2Chassis(ChassisBase):
    '''
    ********
    1******1
    ********
    差速二轮序号定义, 轮子速度定义为轮子顺时针转动为正
    '''
    def __init__(self, track=0.2) -> None:
        logger.info("diff init")
        super().__init__()
        # 配置文件存储路径
        # 轮距
        self.track = track
        self.params_init()
    
    def params_init(self):
        r = self.track
        # 根据小车差速二轮运动计算小车运动， 正解
        self.transform_forward = np.array([[-1/2, 0, 1/r],
                                           [ 1/2, 0, 1/r]])
    
        # 根据小车运动计算小车差速二轮运动， 逆解
        self.transform_inverse = np.array([[ -1,  1],
                                           [ 0,    0],
                                           [ r/2, r/2]])

# 差速车定义
class Diff4Chassis(ChassisBase):
    '''
    2******1
    ********
    ********
    3******4
    差速四轮序号定义, 轮子速度定义为轮子顺时针转动为正
    '''
    def __init__(self, track=0.2) -> None:
        logger.info("diff4 init")
        super().__init__()
        # 配置文件存储路径
        self.track = track
        self.params_init()
    
    def params_init(self):
        r = self.track / 2
        # 根据小车差速四轮运动计算小车运动， 正解
        self.transform_forward = np.array([[ 1/4, 0, 1/4/r],
                                           [-1/4, 0, 1/4/r],
                                           [-1/4, 0, 1/4/r],
                                           [ 1/4, 0, 1/4/r]])
    
        # 根据小车运动计算小车差速四轮运动， 逆解
        self.transform_inverse = np.array([[ 1, -1, -1,  1],
                                           [ 0,  0,  0,  0],
                                           [ r,  r,  r, r]])

# 麦克纳姆轮定义
class MecanumChassis(ChassisBase):
    '''
    2******1
    ********
    ********
    3******4
    麦克纳姆轮序号定义, 从上方往下方看是x形排布, 轮子接触地面是O形排布
    轮子速度定义为轮子顺时针转动为正
    '''
    def __init__(self, track=0.17, wheel_base=0.2) -> None:
        logger.info("mecanum init")
        super().__init__()
        # 轴距 轮距
        self.rx = wheel_base / 2
        self.ry = track / 2
        self.params_init()
    
    def params_init(self):
        r = self.rx + self.ry
        # 根据小车四轮运动计算小车运动， 正解
        self.transform_forward = np.array([[1/4,  1/4, 1/r/4],
                                          [-1/4,  1/4, 1/r/4],
                                          [-1/4, -1/4, 1/r/4],
                                          [ 1/4, -1/4, 1/r/4]])

        # 根据小车运动计算小车四轮运动， 逆解
        self.transform_inverse = np.array([[1, -1, -1,  1],
                                           [1,  1, -1, -1],
                                           [r,  r,  r,  r]])

# 四轮全向轮定义
class QuadricycleChassis(ChassisBase):
    '''
    2/****\1
    ********
    ********
    3\****/4
    四轮全向序号定义, 轮子速度定义为轮子顺时针转动为正
    '''
    def __init__(self, raduis_base=0.1115) -> None:
        logger.info("Quadricycle init")
        super().__init__()
        # 轴距 轮距 轮直径转周长
        self.rx = raduis_base
        self.ry = raduis_base
        self.params_init()
    
    def params_init(self):
        s_th = math.sin(math.pi/4)
        r = (self.rx + self.ry) / 2
        # 根据小车四轮运动计算小车运动, 正解
        self.transform_forward = np.array([[ 1/4/s_th,  1/4/s_th, 1/r/4],
                                           [-1/4/s_th,  1/4/s_th, 1/r/4],
                                           [-1/4/s_th, -1/4/s_th, 1/r/4],
                                           [ 1/4/s_th, -1/4/s_th, 1/r/4]])

        # 根据小车运动计算小车四轮运动, 逆解
        self.transform_inverse = np.array([[s_th, -s_th, -s_th,  s_th],
                                           [s_th,  s_th, -s_th, -s_th],
                                           [   r,     r,     r,     r]])
        
class MapWrap():
    def __init__(self) -> None:
        pass

    def load_map(self, path):
        import json
        with open(path, "r", encoding='utf-8') as f:
            self.map = json.load(f)
        self.map_length = len(self.map)
        self.map_index = 0

    def get_path(self, pose_start, pose_end):
        path = [[1, 1], [2, 2]]
        return path

class RoadMap():
    def __init__(self, path):
        self.path = path
        self.road_map = []
        self.road_map_index = 0
        self.road_map_length = 0
        self.load_road_map()
    
    def load_road_map(self):
        import json
        with open(self.path, "r", encoding='utf-8') as f:
            self.road_map = json.load(f)
    
class Pos2VelPid():
    def __init__(self, cfg):
        self.pid_x = PID(**cfg["pid_x_params"])
        self.pid_y = PID(**cfg["pid_y_params"])
        self.pid_yaw = PID(**cfg["pid_yaw_params"])
    
    def calculate(self, pose_start, pose_end, time_dur):
        err = pose_end - pose_start
        vel = (np.abs(err)) / time_dur
        vel_x = self.pid_x.calculate(err[0])
        vel_y = self.pid_y.calculate(err[1])
        vel_yaw = self.pid_yaw.calculate(err[2])
        return vel_x, vel_y, vel_yaw


class CarBase():
    def __init__(self):
        path = get_path_relative("cfg_vehicle.yaml")
        cfg = yaml.load(open(path, "r", encoding='utf-8'), Loader=yaml.FullLoader)
        
        self.chassis_init(cfg)
        # self.world_odom = OdometryBase()

        # self.motor_convert = MotorConvert(math.pi * cfg["vehicle_cfg"]["wheel_diameter"])
        self.end_flag = False
        self.odom_process = Thread(target=self.odomery_update)
        self.odom_process.daemon = True
        self.odom_process.start()

    def chassis_init(self, cfg):
        try:
            # 获取底盘类型
            chassis_type = cfg["vehicle_cfg"]["chassis_type"]
            wheel_params1 = cfg["vehicle_cfg"]["wheel"]

            logger.info(chassis_type)
            chassis_params = cfg["vehicle_cfg"][chassis_type]["size"]
            motor_ports = cfg["vehicle_cfg"][chassis_type]["wheel"]["port_list"]
            # 获取底盘参数
            self.chassis:ChassisBase = eval(chassis_type)(**chassis_params)
            # 获取电机接口
            # self.motors_chassis = Motors(cfg["vehicle_cfg"][chassis_type]["motor_ports"])
            self.wheels_chassis = WheelWrap(motor_ports, **wheel_params1)
            self.pid_pos = Pos2VelPid(cfg["pid_vel_params"])
            # self.pid_x = PID(**cfg["pid_x_params"])
            # self.pid_y = PID(**cfg["pid_y_params"])
            # self.pid_yaw = PID(**cfg["pid_yaw_params"])
        except:
            logger.error("chassis cfg error")
            while True:
                time.sleep(1)

    @staticmethod
    def sp_world2car(vel_world, angle_car):
        '''
        世界坐标系到车坐标系
        '''
        sin_car = np.sin(angle_car)
        cos_car = np.cos(angle_car)
        # print(sin_car, cos_car)
        transform = np.array([[cos_car, -sin_car, 0], 
                            [sin_car,  cos_car, 0],
                            [       0,       0, 1]])
        vel_car = np.array(vel_world).dot(transform)
        return vel_car
    
    @staticmethod
    def sp_car2world(vel_car, angle_car):
        '''
        车坐标系到世界坐标系
        '''
        sin_car = np.sin(angle_car)
        cos_car = np.cos(angle_car)
        # print(sin_car, cos_car)
        transform = np.array([[cos_car, -sin_car, 0], 
                            [sin_car,  cos_car, 0],
                            [      0,        0, 1]])
        vel_world= np.array(vel_car).dot(transform)
        return vel_world
    def set_velocity(self, linear_vx, linear_vy, angular_v):
        # 根据速度计算四个轮子速度
        wheel_linear = self.chassis.get_velocity(linear_vx, linear_vy, angular_v)
        # print(wheel_linear)
        # 根据线速度计算轮胎的角速度, 计算A,B,C的电机速度
        # sp_motors = self.motor_convert.sp2angluar(wheel_linear)
        # sp_motors = self.motor_convert.sp2virtual(wheel_linear)
        # print(sp_motors)
        self.wheels_chassis.set_linear(wheel_linear)
    
    def odomery_update(self):
        # encoders_last = np.array(self.motors_chassis.get_encoder())
        linear_wheel_last = np.array(self.wheels_chassis.get_linear())
        # print(linear_wheel_last)
        while True:
            if self.end_flag:
                break
            '''
            
            # 计算编码器变化
            encoders_now = np.array(self.motors_chassis.get_encoder())
            encoders_d = encoders_now - encoders_last
            encoders_last = encoders_now
            d_vect_wheel = self.motor_convert.dis2true(encoders_d)
            '''
            linear_wheel_now = self.wheels_chassis.get_linear()
            # 获取每个轮子的位移
            linear_wheel_d = linear_wheel_now - linear_wheel_last
            linear_wheel_last = linear_wheel_now
            # print(linear_wheel_last)
            # 里程计根据轮子的位置变化更新
            self.chassis.updata_odom(linear_wheel_d)
            # print(self.chassis.odom.pose)
            time.sleep(0.05)
        
    def get_odometry(self):
        return self.chassis.odom.pose, self.chassis.odom.dis_traveled
    
    def stop(self):
        self.set_velocity(0, 0, 0)

    def close(self):
        self.end_flag = True
        self.odom_process.join()
    
    def move_to(self, pose):
        start_pos = np.array(self.chassis.odom.pose)
        tar_pos = np.array(pose)

        while True:
            vel_out = self.pid_x(tar_pos[0] - start_pos[0])
            self.set_velocity(vel_out)

    def move_test(self, pose_tar, during=None, vel=[0.1, 0.1, math.pi/3]):
        
        if during is None:
            during = np.max(np.abs((np.array(pose_tar) - self.chassis.odom.pose) / vel))
        pose_threshold = np.array([0.005, 0.005, 0.02])
        tim_st = time.time()
        count_exit = 0
        while True:
            # pose_now = np.array(self.chassis.odom.pose)
            time_cost = time.time() - tim_st
            if time_cost > during + 0.2:
                print("time out")
                break
            pose_err = pose_tar - self.chassis.odom.pose
            err = np.abs(pose_err)
            # print("err:{}, threshold:{}".format(err, pose_threshold))
            err_ret = (err < pose_threshold)
            # print(err_ret)
            if err_ret.all():
                count_exit += 1
                if count_exit > 20:
                    break
            else:
                count_exit = 0
            
            vel_car = self.pid_pos.calculate(pose_err, )
            # 世界坐标速度转换车子坐标速度
            vel_out = self.sp_world2car(vel_car, self.chassis.odom.pose[2])
            self.set_velocity(*vel_out)
            # time.sleep(0.05)
        self.set_velocity(0, 0, 0)

    def set_pos(self, pos, time_dur=None, sp=[0.2, 0.2, math.pi/4]):
        start_pos = np.array(self.chassis.odom.pose)
        tar_pos = np.array(pos)
        
        if time_dur is not None:
            sp = (tar_pos - start_pos) / time_dur
        else:
            time_dur = np.max(abs((tar_pos - start_pos) / sp))
            sp = (tar_pos - start_pos) / time_dur

        # print("time_dur:{}".format(time_dur))
        # print("start{}, tar{}, sp{}".format(start_pos, tar_pos, sp))
        
        self.pid_dis_x.setpoint = tar_pos[0]
        self.pid_dis_x.output_limits = (-abs(sp[0]), abs(sp[0]))

        self.pid_dis_y.setpoint = tar_pos[1]
        self.pid_dis_y.output_limits = (-abs(sp[1]), abs(sp[1]))

        self.pid_turn.setpoint = tar_pos[2]
        self.pid_turn.output_limits = (-abs(sp[2]), abs(sp[2]))

        start_time = time.time()
        count_flag = 0
        while True:
            current_time = time.time()
            dt = current_time - start_time
            if dt > time_dur + 0.3:
                logger.debug("move pos time out")
                break
        
            now_pos = np.array(self.get_odom())
            # print("now_pos:{}".format(now_pos))
            # 获取剩余距离值和角度值
            error_dis = tar_pos - now_pos
            # print("error_dis:{}".format(error_dis))
            if abs(error_dis[0]) < 0.01 and abs(error_dis[1]) < 0.01 and abs(error_dis[2]) < 0.05:
                count_flag += 1
                if count_flag > 20:
                    break
            else:
                count_flag = 0
            out_x = self.pid_dis_x(now_pos[0])
            out_y = self.pid_dis_y(now_pos[1])
            out_omega = self.pid_turn(now_pos[2])
            # 世界输出转为机器人输出
            angle_robot = now_pos[2]
            # dis_omega = error_dis[2]
            # 根据误差角度计算实际运行速度和角度值
            sp_x = math.cos(angle_robot) * out_x + math.sin(angle_robot) * out_y
            sp_y = 0 - math.sin(angle_robot) * out_x + math.cos(angle_robot) * out_y
            sp_omega = out_omega

            # print("pid in_x:{}, in_y:{}, in_omega:{}".format(now_pos[0], now_pos[1], now_pos[2]))
            # print("pid out_x:{}, out_y:{}, out_omega:{}".format(out_x, out_y, out_omega))
            # print("out_x", out_x, "out_y", out_y, "out_omega", out_omega)

            self.mecanum_wheel(sp_x, sp_y, sp_omega)
        # print(self.get_odom())
        self.run4([0, 0, 0, 0])


if __name__ == '__main__':

    # logger.info("init ok")
    # car = CarBase()
    # for i in range(10):
    #     car.set_velocity(0, 0, math.pi/2)
    #     time.sleep(0.1)
    #     pose, traveled = car.get_odometry()
    #     # print(pose)
    # car.set_velocity(0, 0, 0)
    # pose, traveled = car.get_odometry()
    # logger.info("pose:{}, traveled{}".format(pose, traveled))
    # diff = Diff2Chassis()
    # diff4 = Diff4Chassis()
    # tric = TricycleChassis()
    # mecanum = MecanumChassis()
    # quad = QuadricycleChassis()
    car = CarBase()
    
    # for i in range(20):
        # pose, trav = car.get_odometry()
        # angle = pose[2]
        
        # vel_car = sp_world2car([0.1, 0, math.pi/2], angle)
        # print(vel_car, angle)
        # car.set_velocity(*vel_car)
        # car.set_velocity(0, 0, math.pi/2)
        # car.set_velocity(0, -0.1, 0)
        # time.sleep(0.089)
    car.move_test([0,0,math.pi], 1.5)
    # st = time.time()
    # while True:
    #     if time.time()-st > 2:
    #         break
    #     car.set_velocity(0, 0, math.pi/2)
    #     time.sleep(0.05)
    # car.set_velocity(0, 0, 0)
    print(car.get_odometry())
    car.move_test([0,0.2,-math.pi/2], 1)
    print(car.get_odometry())
    print(time.time())
    car.close()
    '''
    for i in range(20):
        # pose, trav = car.get_odometry()
        # angle = pose[2]
        
        # vel_car = sp_world2car([0.1, 0, math.pi/2], angle)
        # print(vel_car, angle)
        # car.set_velocity(*vel_car)
        car.set_velocity(0, 0, math.pi/2)
        # car.set_velocity(0, -0.1, 0)
        time.sleep(0.089)
    # print(time.time())
    pose, traveled = car.get_odometry()
    car.set_velocity(0, 0, 0)
    print(pose, traveled)
    '''
    # print(pose)
    # car.set_velocity(0, 0, 0)
    # /dev/video9
    # print("vel:", car.set_velocity(0, 0, math.pi/2))
