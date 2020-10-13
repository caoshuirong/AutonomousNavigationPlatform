from myrobot import MyRobot
import time

# 常数定义
PI = 3.14159265
WHEEL_DIAMETER = 66 # mm，车轮的直径
WHEEL_DISTANCE = 134 # mm，两轮的轴距

# 输入参数
V_lin = WHEEL_DIAMETER*PI # mm/s，需要达到的线速度
V_ang = 0 # rad/s，需要达到的角速度

# 主程序
r = MyRobot() # 初始化对象

V_left = V_lin - V_ang*WHEEL_DISTANCE/2 # mm/s，换算左轮线速度
V_right = V_lin + V_ang*WHEEL_DISTANCE/2 # mm/s，换算右轮线速度

RPS_left = V_left / (WHEEL_DIAMETER*PI) # 每秒圈数，换算左轮转速
RPS_right = V_right / (WHEEL_DIAMETER*PI) # 每秒圈数，换算右轮转速

r.set_motors(RPS_left*0.1, RPS_right*0.1) # 以规定转速动作，0.1为常系数
time.sleep(99)

r.stop() # 动作结束，停止运动（实际运用时，可以只在需要停下来时使用，其余时候根据指令改变转速即可）

r.__del__() # 销毁对象（不再用电机时调用）

# 主程序结束