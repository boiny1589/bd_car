#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
键盘控制车辆程序
使用上下左右键控制车辆的前进、后退、左右移动
支持组合键控制和速度调节
"""

import time
import threading
import sys
import os

# 添加项目根目录到路径
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from vehicle import CarBase, Key4Btn, ScreenShow, Beep
from log_info import logger

# 尝试导入键盘输入库
try:
    import msvcrt  # Windows系统
    KEYBOARD_AVAILABLE = True
    logger.info("使用msvcrt键盘库")
except ImportError:
    try:
        import keyboard  # 跨平台
        KEYBOARD_AVAILABLE = True
        logger.info("使用keyboard库")
    except ImportError:
        KEYBOARD_AVAILABLE = False
        logger.warning("未找到键盘输入库，将使用模拟键盘")


class ChedipanControl:
    """键盘控制车辆类"""
    
    def __init__(self):
        """初始化车辆控制"""
        logger.info("初始化车辆控制...")
        
        # 初始化车辆
        self.car = CarBase()
        
        # 初始化键盘控制 - 修复端口参数问题
        try:
            # 尝试使用端口1初始化键盘
            self.key = Key4Btn(1)
            logger.info("硬件键盘初始化成功")
        except Exception as e:
            logger.warning(f"硬件键盘初始化失败，使用软件键盘: {e}")
            # 如果键盘初始化失败，创建一个软件键盘类
            self.key = self.create_software_keyboard()
        
        # 初始化显示 - 添加异常处理
        try:
            self.display = ScreenShow()
        except Exception as e:
            logger.warning(f"显示初始化失败，使用模拟显示: {e}")
            self.display = self.create_mock_display()
        
        # 初始化蜂鸣器
        self.beep = Beep()
        
        # 控制参数
        self.linear_speed = 0.3    # 线速度
        self.angular_speed = 1.0   # 角速度
        self.stop_flag = False     # 停止标志
        
        # 当前运动状态
        self.current_linear_x = 0.0
        self.current_linear_y = 0.0
        self.current_angular = 0.0
        
        # 按键状态记录
        self.key_states = {1: False, 2: False, 3: False, 4: False}
        
        # 按键映射
        self.key_map = {
            1: "前进",      # 上键
            2: "后退",      # 下键  
            3: "左转",      # 左键
            4: "右转",      # 右键
            5: "停止",      # 长按1键退出
        }
        
        # 速度等级
        self.speed_levels = [0.1, 0.2, 0.3, 0.4, 0.5]
        self.current_speed_level = 2  # 默认中等速度
        
        logger.info("车辆控制初始化完成")
    
    def create_software_keyboard(self):
        """创建软件键盘类"""
        class SoftwareKeyboard:
            def __init__(self):
                self.last_key = 0
                self.key_press_time = 0
                self.key_thread = None
                self.running = False
                logger.info("软件键盘初始化完成")
                
            def get_key(self):
                """获取键盘输入"""
                if KEYBOARD_AVAILABLE:
                    try:
                        if 'msvcrt' in sys.modules:
                            # Windows系统
                            if msvcrt.kbhit():
                                key = msvcrt.getch()
                                result = self.translate_key(key)
                                if result != 0:
                                    logger.info(f"检测到按键: {key}, 映射为: {result}")
                                return result
                        elif 'keyboard' in sys.modules:
                            # 跨平台键盘库
                            if keyboard.is_pressed('up') or keyboard.is_pressed('w'):
                                return 1
                            elif keyboard.is_pressed('down') or keyboard.is_pressed('s'):
                                return 2
                            elif keyboard.is_pressed('left') or keyboard.is_pressed('a'):
                                return 3
                            elif keyboard.is_pressed('right') or keyboard.is_pressed('d'):
                                return 4
                            elif keyboard.is_pressed('q'):
                                return 5
                    except Exception as e:
                        logger.warning(f"键盘输入错误: {e}")
                
                return 0
            
            def translate_key(self, key):
                """翻译键盘按键"""
                try:
                    if isinstance(key, bytes):
                        key = key.lower()
                        if key in [b'w', b'\xe0H']:  # W键或上箭头
                            return 1
                        elif key in [b's', b'\xe0P']:  # S键或下箭头
                            return 2
                        elif key in [b'a', b'\xe0K']:  # A键或左箭头
                            return 3
                        elif key in [b'd', b'\xe0M']:  # D键或右箭头
                            return 4
                        elif key == b'q':  # Q键退出
                            return 5
                        else:
                            logger.debug(f"未识别的按键: {key}")
                    else:
                        # 处理字符串类型的按键
                        key = key.lower()
                        if key in ['w', 'up']:
                            return 1
                        elif key in ['s', 'down']:
                            return 2
                        elif key in ['a', 'left']:
                            return 3
                        elif key in ['d', 'right']:
                            return 4
                        elif key == 'q':
                            return 5
                except Exception as e:
                    logger.warning(f"按键翻译错误: {e}")
                
                return 0
                
            def read(self):
                return self.get_key()
        
        return SoftwareKeyboard()
    
    def create_mock_display(self):
        """创建模拟显示类"""
        class MockDisplay:
            def __init__(self):
                self.last_message = ""
                
            def show(self, message):
                # 模拟显示，只打印到控制台
                if message != self.last_message:
                    print(f"[显示] {message}")
                    self.last_message = message
        
        return MockDisplay()
        
    def beep_alert(self, times=1):
        """蜂鸣器提示"""
        for _ in range(times):
            self.beep.rings()
            time.sleep(0.1)
    
    def show_status(self, status):
        """显示状态信息 - 使用英文避免编码问题"""
        try:
            speed_info = f"Speed: {self.current_speed_level + 1}/5"
            status_info = f"Status: {status}"
            control_info = "Keys: W/S/A/D or Arrow Keys"
            exit_info = "Q: Exit"
            display_text = f"{status_info}\n{speed_info}\n{control_info}\n{exit_info}"
            self.display.show(display_text)
        except Exception as e:
            logger.warning(f"显示失败: {e}")
            # 如果显示失败，只打印到日志
            logger.info(f"状态: {status}, 速度等级: {self.current_speed_level + 1}/5")
    
    def update_speed(self):
        """更新当前速度"""
        self.linear_speed = self.speed_levels[self.current_speed_level]
        self.angular_speed = self.linear_speed * 2.0  # 角速度是线速度的2倍
    
    def increase_speed(self):
        """增加速度"""
        if self.current_speed_level < len(self.speed_levels) - 1:
            self.current_speed_level += 1
            self.update_speed()
            logger.info(f"速度增加到等级 {self.current_speed_level + 1}")
            self.beep_alert(2)
    
    def decrease_speed(self):
        """减少速度"""
        if self.current_speed_level > 0:
            self.current_speed_level -= 1
            self.update_speed()
            logger.info(f"速度减少到等级 {self.current_speed_level + 1}")
            self.beep_alert(3)
    
    def move_forward(self):
        """前进"""
        self.current_linear_x = self.linear_speed
        self.current_linear_y = 0.0
        self.current_angular = 0.0
        self.car.set_velocity(self.current_linear_x, self.current_linear_y, self.current_angular)
    
    def move_backward(self):
        """后退"""
        self.current_linear_x = -self.linear_speed
        self.current_linear_y = 0.0
        self.current_angular = 0.0
        self.car.set_velocity(self.current_linear_x, self.current_linear_y, self.current_angular)
    
    def turn_left(self):
        """左转"""
        self.current_linear_x = 0.0
        self.current_linear_y = 0.0
        self.current_angular = self.angular_speed
        self.car.set_velocity(self.current_linear_x, self.current_linear_y, self.current_angular)
    
    def turn_right(self):
        """右转"""
        self.current_linear_x = 0.0
        self.current_linear_y = 0.0
        self.current_angular = -self.angular_speed
        self.car.set_velocity(self.current_linear_x, self.current_linear_y, self.current_angular)
    
    def move_forward_left(self):
        """左前移动"""
        self.current_linear_x = self.linear_speed * 0.7
        self.current_linear_y = 0.0
        self.current_angular = self.angular_speed * 0.5
        self.car.set_velocity(self.current_linear_x, self.current_linear_y, self.current_angular)
    
    def move_backward_left(self):
        """左后移动"""
        self.current_linear_x = -self.linear_speed * 0.7
        self.current_linear_y = 0.0
        self.current_angular = self.angular_speed * 0.5
        self.car.set_velocity(self.current_linear_x, self.current_linear_y, self.current_angular)
    
    def move_forward_right(self):
        """右前移动"""
        self.current_linear_x = self.linear_speed * 0.7
        self.current_linear_y = 0.0
        self.current_angular = -self.angular_speed * 0.5
        self.car.set_velocity(self.current_linear_x, self.current_linear_y, self.current_angular)
    
    def move_backward_right(self):
        """右后移动"""
        self.current_linear_x = -self.linear_speed * 0.7
        self.current_linear_y = 0.0
        self.current_angular = -self.angular_speed * 0.5
        self.car.set_velocity(self.current_linear_x, self.current_linear_y, self.current_angular)
    
    def stop_movement(self):
        """停止移动"""
        self.current_linear_x = 0.0
        self.current_linear_y = 0.0
        self.current_angular = 0.0
        self.car.set_velocity(0, 0, 0)
    
    def handle_combined_keys(self):
        """处理组合键"""
        # 检查组合键状态
        if self.key_states[1] and self.key_states[3]:  # 前进 + 左转
            self.move_forward_left()
            return "Forward-Left"
        elif self.key_states[1] and self.key_states[4]:  # 前进 + 右转
            self.move_forward_right()
            return "Forward-Right"
        elif self.key_states[2] and self.key_states[3]:  # 后退 + 左转
            self.move_backward_left()
            return "Backward-Left"
        elif self.key_states[2] and self.key_states[4]:  # 后退 + 右转
            self.move_backward_right()
            return "Backward-Right"
        elif self.key_states[1]:  # 仅前进
            self.move_forward()
            return "Forward"
        elif self.key_states[2]:  # 仅后退
            self.move_backward()
            return "Backward"
        elif self.key_states[3]:  # 仅左转
            self.turn_left()
            return "Turn-Left"
        elif self.key_states[4]:  # 仅右转
            self.turn_right()
            return "Turn-Right"
        else:  # 无按键
            self.stop_movement()
            return "Stop"
    
    def handle_key_press(self, key_value):
        """处理按键事件"""
        if key_value == 0:
            return
        
        # 获取按键对应的动作
        action = self.key_map.get(key_value, "未知")
        logger.info(f"按键: {key_value}, 动作: {action}")
        
        # 处理特殊按键
        if key_value == 5:  # 长按1键退出
            logger.info("收到退出信号")
            self.stop_flag = True
            return
        
        # 更新按键状态
        if key_value in [1, 2, 3, 4]:
            self.key_states[key_value] = True
        
        # 处理组合键
        status = self.handle_combined_keys()
        self.show_status(status)
        
        # 蜂鸣器提示
        self.beep_alert()
    
    def handle_key_release(self):
        """处理按键释放"""
        # 检查是否有按键释放
        for key in [1, 2, 3, 4]:
            if self.key_states[key]:
                # 模拟按键释放检测（这里简化处理）
                # 实际应用中需要更复杂的按键状态检测
                pass
    
    def simple_keyboard_control(self):
        """简单的键盘控制循环"""
        logger.info("开始简单键盘控制模式")
        logger.info("控制说明:")
        logger.info("W/↑: 前进")
        logger.info("S/↓: 后退") 
        logger.info("A/←: 左转")
        logger.info("D/→: 右转")
        logger.info("Q: 退出")
        logger.info("请确保控制台窗口处于活动状态")
        
        self.update_speed()
        self.show_status("Ready")
        self.beep_alert()
        
        try:
            while not self.stop_flag:
                # 直接使用msvcrt检测键盘输入
                if 'msvcrt' in sys.modules and msvcrt.kbhit():
                    key = msvcrt.getch()
                    logger.info(f"检测到按键: {key}")
                    
                    # 处理按键
                    if key in [b'w', b'W']:
                        self.move_forward()
                        self.show_status("Forward")
                        logger.info("前进")
                    elif key in [b's', b'S']:
                        self.move_backward()
                        self.show_status("Backward")
                        logger.info("后退")
                    elif key in [b'a', b'A']:
                        self.turn_left()
                        self.show_status("Turn-Left")
                        logger.info("左转")
                    elif key in [b'd', b'D']:
                        self.turn_right()
                        self.show_status("Turn-Right")
                        logger.info("右转")
                    elif key in [b'q', b'Q']:
                        logger.info("收到退出信号")
                        self.stop_flag = True
                        break
                    else:
                        logger.info(f"未识别的按键: {key}")
                        self.stop_movement()
                        self.show_status("Stop")
                    
                    # 蜂鸣器提示
                    self.beep_alert()
                
                # 短暂延时
                time.sleep(0.05)
                
        except KeyboardInterrupt:
            logger.info("收到键盘中断信号")
        except Exception as e:
            logger.error(f"控制循环异常: {e}")
        finally:
            # 清理资源
            self.cleanup()
    
    def run(self):
        """运行主控制循环"""
        # 使用简单键盘控制
        self.simple_keyboard_control()
    
    def cleanup(self):
        """清理资源"""
        logger.info("清理资源...")
        self.stop_movement()
        self.car.close()
        logger.info("资源清理完成")


def main():
    """主函数"""
    try:
        # 创建控制实例
        controller = ChedipanControl()
        
        # 运行控制循环
        controller.run()
        
    except Exception as e:
        logger.error(f"程序异常: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
