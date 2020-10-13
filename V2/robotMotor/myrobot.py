# coding=utf-8

import time
import smbus
import threading
from robot import Robot

bus = smbus.SMBus(1)
robot = Robot()
timer = None


'''
PID 控制器
输入变量：p,i,d 系数根据实际情况调节大小
         feedback_value 编码器测量的速度值
输出变量：output 根据PID控制器计算出来的电机PWM
该PID控制器没有设置时间变量，因此使用的时候需要设置定制器，
定时刷新update函数，时间越小反馈更及时
'''


class PID:
    def __init__(self, p=1.0, i=0.0, d=0.0):  # 转入pid参数
        self.setpoint = 0.0  # 目标值
        self.windup = 1.0  # 积分最大值
        self.Kp = p  # 比例系数
        self.Ki = i  # 积分系数
        self.Kd = d  # 微分系数
        self.ITerm = 0.0  # 积分项
        self.last_error = 0.0  # 上一次偏差值
        self.output = 0.0  # 计算结果

    def update(self, feedback_value):  # 传入实际测量值
        if self.setpoint >= 1.0:
            self.setpoint = 1.0
        if self.setpoint <= -1.0:
            self.setpoint = -1.0
        if self.setpoint >= 0:
            error = self.setpoint - feedback_value / 100.0  # 计算偏差值
        else:
            error = -self.setpoint - feedback_value / 100.0  # 计算偏差值
        self.ITerm += error  # 计算积分项
        if self.ITerm < -self.windup:  # 限制积分项大小，抗积分饱和
            self.ITerm = -self.windup
        elif self.ITerm > self.windup:
            self.ITerm = self.windup
        self.output = self.Kp * error + self.Ki * self.ITerm + self.Kd * (error - self.last_error)  # PID公式
        if self.output >= 1.0:
            self.output = 1.0
        elif self.output <= 0:
            self.output = 0
        if self.setpoint < 0:
            self.output = -self.output
        self.last_error = error  # 保存当前偏差值
        return self.output  # 输出


class MyRobot:
    def __init__(self):
        self.pid_right = PID(p=2.0, i=1.0, d=1.0)  # 使用PID控制器分别控制左右电机
        self.pid_left = PID(p=2.0, i=1.0, d=1.0)
        self.start()

    def __del__(self):
        # 在timer未触发时，结束计时器线程
        while True:
            if not timer.is_alive():
                timer.cancel()
                break

    def start(self):
        # 分别获取左电机速度，右电机速度，电池电压值
        left_speed, right_speed, voltage = bus.read_byte(0x30), bus.read_byte(0x30), bus.read_byte(0x30)
        #print("%d, %d, %d" % (left_speed, right_speed, voltage))
        # 输入编码器速度，经过PID计算，控制左右轮的速度
        robot.set_motors(self.pid_left.update(left_speed), self.pid_right.update(right_speed))

        global timer
        timer = threading.Timer(0.05, self.start)  # PID 控制周期 50ms
        timer.start()

    def set_motors(self, left_speed, right_speed):
        self.pid_left.setpoint = left_speed
        self.pid_right.setpoint = right_speed


    def forward(self, speed=0):
        self.pid_left.setpoint = speed
        self.pid_right.setpoint = speed

    def backward(self, speed=0):
        self.pid_left.setpoint = -speed
        self.pid_right.setpoint = -speed

    def left(self, speed=0):
        self.pid_left.setpoint = -speed
        self.pid_right.setpoint = speed

    def right(self, speed=0):
        self.pid_left.setpoint = speed
        self.pid_right.setpoint = -speed

    def stop(self, speed=0):
        self.pid_left.setpoint = 0
        self.pid_right.setpoint = 0
