from vehicle import ArmBase, ScreenShow, Key4Btn, ServoBus, ServoPwm, MotorWrap, StepperWrap, PoutD
import cv2
import time
import numpy as np
import yaml, os, math

class Ejection():
    def __init__(self, portm=5, portd=4, port_step=1) -> None:
        self.motor = MotorWrap(portm, -1, type="motor_280", perimeter=0.06/15*8)
        self.pout = PoutD(portd)
        self.step1 = StepperWrap(port_step)
        self.step_rad_st = self.step1.get_rad()
        self.step1_rad_cnt = 0

    def reset(self, vel=0.05):
        rad_last = self.motor.get_rad()
        
        while True:
            self.motor.set_linear(vel)
            time.sleep(0.02)
            rad_now = self.motor.get_rad()
            if abs(rad_now - rad_last) < 0.02:
                break
            rad_last = rad_now
        
        self.motor.set_linear(0)
        
    def eject(self, x=0.1, vel=0.05):
        self.reset()
        self.pout.set(1)
        self.motor.reset()
        self.motor.set_linear(0-abs(vel))
        length = 0.11
        while True:
            self.motor.set_linear(0-abs(vel))
            if abs(self.motor.get_dis()) > length:
                break
        self.motor.set_linear(0)                  
        self.step1_rad_cnt += 1
        self.step1.set_rad(math.pi/5*2*self.step1_rad_cnt + self.step_rad_st)
        self.pout.set(1)
        time.sleep(0.5)
        while True:
            self.motor.set_linear(abs(vel))
            if abs(self.motor.get_dis()) < x:
                break
        self.motor.set_linear(0)
        self.pout.set(0)

    
class MyTask:
    def __init__(self):
        # 旋转舵机
        self.servo_bmi = ServoBus(2)
        # self.servo_rotate.set_angle(90, 0)
        
        # 发射装置
        self.ejection = Ejection()
        time.sleep(0.3)

        # 机械臂
        self.arm = ArmBase()

    def reset(self):
        self.arm.reset()

    # 抓圆柱，选则大小
    def pick_up_cylinder(self, radius, arm_set=False):
        print("pick_up_cylinder开始,arm_set:",arm_set,"False是真抓")
        # 定位目标的参数 label_id, obj_width, label, prob, err_x, err_y, width, height
        # id号码（数据集对应的），目标宽度，目标名称，目标概率，目标中心误差x，目标中心误差y，目标宽度占比，目标高度占比
        tar_list =  [[13, 100, "cylinder1", 0,  0, 0.28, 0.75, 0.97], [14, 80, "cylinder2", 0, 0, 0.3, 0.61, 0.9], 
                     [15, 60, "cylinder3", 0, 0, 0.2, 0.45, 0.7]]
        # pt_tar = tar_list[radius-1]
        height_list = [0.08, 0.08, 0.15]
        tar_height = height_list[0]
        tar_horiz = self.arm.horiz_mid
        # 手爪方向向下
        self.arm.set_hand_angle(48)
        print("pick_up_cylinder设置小舵机方向48")
        if arm_set:
            tar_height = 0.08
            # 到达目标位置
            self.arm.set(tar_horiz, tar_height)
            print("pick_up_cylinder到达目标位置tar_height: %f" % tar_height)
            return tar_list
        # 抓取圆柱
        print("pick_up_cylinder准备抓取圆柱")
        self.arm.grap(1)
        # 到圆柱的位置
        horiz_offset = 0.14 * self.arm.side
        if radius == 0:
            # self.arm.set(tar_horiz, tar_height+0.04)
            self.arm.set_offset(0, 0.04)
        tar_horiz = self.arm.horiz_mid + horiz_offset
        self.arm.set(tar_horiz, tar_height)
        print("pick_up_cylinder移动到准备抓取的位置tar_horiz: %f, tar_height: %f" % (tar_horiz, tar_height))
        # 往下放,抓住
        self.arm.set_offset(0, -0.015)
        time.sleep(0.5)
        # 抬起一定高度
        # height_offset = 0.07
        height_offset = 0.12
        if radius == 2:
            height_offset += 0.06
        self.arm.set_offset(0, height_offset)
        # self.arm.set_offset(0, 0.08, 1.3)

    def put_down_cylinder(self, radius):
        # tar_height = 0.02
        height_offset = 0.02
        if radius==0:
            height_offset = 0.12
        # 下放放开物块
        self.arm.set_offset(0, 0-height_offset)
        print("put_down_cylinder 0-height_offset：%s" % (0-height_offset))
        # time.sleep(0.2)
        self.arm.grap(0)
        print("put_down_cylinder grap 放")
        time.sleep(0.5)
        # 抬起
        self.arm.set_offset(0, 0.02)
        # horiz_offset = 0.1 * self.arm.side * -1
        # self.arm.set_offset(horiz_offset, 0)
    
    def bmi_set(self, num=0, arm_set=False):
        tar = [[0, 70, 'text_det', 0, 0, -0.31, 0.85, 1.0]]
        bmi = {0:0, 1:-45, 2: -135, 3:45, 4:135}
        tar_height = 0.045
        tar_horiz = 0.15
        if arm_set:
            self.arm.set_hand_angle(48)
            self.arm.set(tar_horiz, tar_height)
            return tar
        self.servo_bmi.set_angle(bmi[num])
        # self.servo_bmi.set_angle(0)

    def get_ingredients(self, side=1, ocr_mode=False, arm_set=False):
        tar = [[0, 70, 'text_det', 0, 0, 0.5, 0.5, 0.5]]
        if ocr_mode:
            tar_height = 0.0
        else:
            tar_height = 0.07

        tar_horiz = self.arm.horiz_mid
        self.arm.set_hand_angle(48)
        self.arm.switch_side(side)
        self.arm.set(tar_horiz, tar_height)

        if arm_set:
            return tar
        # self.arm.switch_side(side)
        # self.arm.set_offset(0.1, 0)

    def pick_ingredients(self, num=1, row=1, arm_set=False):
        tar = [[4, 30, 'chicken', 0, 0, 0.03, 0.25, 0.24], [3, 30, 'tomato', 0, 0, 0.03, 0.25, 0.24], [7, 30, 'egg', 0, 0, -0.15, 0.22, 0.22] ]
        # 计算高度，手臂根据高度设置位置
        tar_height = 0.02 + (row-1)*0.09
        horiz_offset = 0 * self.arm.side
        tar_horiz = self.arm.horiz_mid + horiz_offset

        self.arm.set(tar_horiz, tar_height)
        # 准备抓取
        self.arm.grap(1)
        # 如果是进行识别，这里手向下
        if arm_set:
            self.arm.set_hand_angle(45)
            return tar
        # 手水平
        self.arm.set_hand_angle(-45)
        time.sleep(0.5)
        # 手臂向外伸，去抓取物块
        horiz_offset = 0.13*self.arm.side
        self.arm.set_offset(horiz_offset, 0)
        
        # self.arm.set(0.26, 0.10)
        if num > 1:
            # 第二块保持住不动
            self.arm.set_offset(-0.18*self.arm.side, 0.02, speed=[0.12, 0.04])
            return tar
        # 收回手臂
        # self.arm.set_offset(-0.14*self.arm.side, 0.03, speed=[0.12, 0.04])
        self.arm.set(tar_horiz, 0.05)
        # 手向下
        self.arm.set_hand_angle(49)
        # 放下物块
        self.arm.set_offset(-0.13*self.arm.side, 0, speed=[0.12, 0.04])
        # self.arm.set(0.14-self.arm.side*0.14, 0.04, speed=[0.08, 0.04])
        
        self.arm.set_offset(0, -0.045, speed=[0.12, 0.04])
        # time.sleep(0.5)
        self.arm.grap(0)
        time.sleep(0.5)
        self.arm.set_offset(0, 0.045, speed=[0.12, 0.04])

    def get_answer(self, arm_set=False):
        tar = [[0, 70, 'text_det', 0, 0, -0.5, 0.44, 0.6]]
        self.arm.grap(1)
        self.arm.switch_side(1)
        self.arm.set_hand_angle(48)
        tar_height = 0.1
        tar_horiz = self.arm.horiz_mid
        
        self.arm.set(tar_horiz, tar_height)
        if arm_set:
            return tar
        # 竖着向下为-45
        self.arm.set_hand_angle(-45)
        self.arm.set_offset(0.09, 0)
        self.arm.set_offset(-0.09, 0)


    def set_food(self, num=1, row=1, arm_set=False):
        # 定位目标的参数 label_id, obj_id, label, prob, err_x, err_y, width, height
        tar = [[0, 70, 'text_det', 0, 0, -0.14, 0.46, 0.53]]
        # 气泵吸气并关闭阀门，调整手臂方向向右

        self.arm.grap(1)
        self.arm.switch_side(-1)

        if arm_set:
            # 准备识别的位置，手朝向下
            self.arm.set_hand_angle(45)
            tar_height = 0.12
            tar_horiz = self.arm.horiz_mid
            # 到达准备位置
            self.arm.set(tar_horiz, tar_height)
            return tar
        

        if num > 1:
            # 如果放的不是第一个，需要先抓取，手朝向下
            self.arm.set_hand_angle(45)
            # 到达抓取位置，准备抓取
            self.arm.set(self.arm.horiz_mid+0.14, 0.04, speed=[0.12, 0.04])
            # 向下移动抓取
            self.arm.grap(1)
            self.arm.set_offset(0, -0.04, speed=[0.12, 0.04])
            time.sleep(0.5)
            # 向上移动
            self.arm.set_offset(0, 0.05, speed=[0.12, 0.04])
            # 手臂指向方向调整水平
            self.arm.set_hand_angle(-45)
        self.arm.set_hand_angle(-45)
        # 根据目标位置调整手臂位置
        tar_height = 0.02 + (row-1)*0.1
        horiz_offset = 0 * self.arm.side
        #  准备放食材到指定位置
        tar_horiz = self.arm.horiz_mid + horiz_offset
        self.arm.set(tar_horiz, tar_height)
        # 手臂向前伸运动0.14m
        self.arm.set_offset(0.13*self.arm.side, 0, speed=[0.12, 0.04])
        # self.arm.set()
        # self.arm.set_hand_angle(-45)
        # self.arm.set_offset(-0.09, 0)
        self.arm.grap(0)
        time.sleep(0.5)
        self.arm.set_offset(-0.1*self.arm.side, 0, speed=[0.12, 0.04])

    def eject(self, area=1):
        dis_list = {1:0.0845, 2:0.063}
        self.ejection.eject(dis_list[area])
   
    def help_peo(self, arm_set=False):
        # 调整方向向左
        self.arm.switch_side(1)
        # 调整手水平
        self.arm.set_hand_angle(-45)
        tar_height = 0.08
        tar_horiz = self.arm.horiz_mid
        self.arm.set(tar_horiz, tar_height)
        if arm_set:
            return
        # 伸长手臂
        self.arm.set_offset(0.1, 0)
        self.arm.set_offset(-0.1, 0)



def task_reset():
    task = MyTask()
    task.reset()
    time.sleep(0.1)

def bmi_test():
    task = MyTask()
    task.bmi_set(0)

def cylinder_test():
    task = MyTask()
    key = Key4Btn(1)
    # task.arm.reset()
    i = 0
    tar = task.pick_up_cylinder(i, arm_set=True)
    '''
    调用 task.pick_up_cylinder(i, arm_set=True) ，这里 arm_set=True 表示只进行准备动作：
    - 设置手臂角度为48度（ self.arm.set_hand_angle(48) ）
    - 将机械臂移动到预设高度 tar_height = 0.045
    - 将机械臂移动到水平中间位置 tar_horiz = self.arm.horiz_mid
    '''
    while True:
        if key.get_key()!=0:
            '''
            - 进入无限循环，等待按键输入
            - 当检测到按键按下（ key.get_key()!=0 ）时：
            - 执行抓取动作： task.pick_up_cylinder(i) ，此时 arm_set=False （默认值），会执行完整的抓取流程：
            - 打开抓手（ self.arm.grap(1) ）
            - 移动到目标位置
            - 下降抓取
            - 抬起机械臂
            - 等待1秒
            - 执行放下动作： task.put_down_cylinder(i)
            - 下降机械臂
            - 释放抓手（ self.arm.grap(0) ）
            - 抬起机械臂
            - 等待1秒
            - 增加圆柱体索引 i ，准备测试下一个圆柱体
            '''
            
            time.sleep(1)
            task.pick_up_cylinder(i)
            time.sleep(1)
            task.put_down_cylinder(i)
            time.sleep(1)
            i = i+1
    # for i in range(3):
    #     tar = task.pick_up_cylinder(i+1, arm_set=True)
    #     time.sleep(0.8)
    #     task.pick_up_cylinder(i+1)
    #     time.sleep(0.5)
    #     task.put_down_cylinder(i+1)
    #     time.sleep(0.5)

# 定义一个函数highball_test
def ingredients_test():
    task = MyTask()
    task.get_ingredients(1, arm_set=True)

def pick_ingredients_test():
    task = MyTask()
    task.get_ingredients(1, ocr_mode=True, arm_set=True)
    task.arm.switch_side(1)
    task.pick_ingredients(1)
    task.arm.switch_side(-1)
    task.pick_ingredients(2, 2)

def answer_test():
    task = MyTask()
    task.get_answer(arm_set=True)
    # time.sleep(1)
    # task.get_answer()

def food_test():
    task = MyTask()
    task.set_food(arm_set=True)
    time.sleep(1)
    task.set_food()
    task.set_food(2)
    
def eject_test():
    task = MyTask()
    task.eject(2)

if __name__ == "__main__":
    
    import argparse
    args = argparse.ArgumentParser()
    args.add_argument('--op', type=str, default="none")
    args = args.parse_args()
    print(args)
    if args.op == "reset":
        task_reset()

    # eject_test()
    cylinder_test()
    # bmi_test()
    # ingredients_test()
    # pick_ingredients_test()
    # answer_test()
    # food_test()
