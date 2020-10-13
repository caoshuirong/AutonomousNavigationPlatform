#!/usr/bin/env python
# -*- coding:utf-8 -*-


import serial
import time
import numpy as np
import math

class SerialController:
    class Instruction:
        def __init__(self):
            self.velLeft = 0  # 左轮速度，0~64，无单位
            self.velRight = 0  # 右轮速度，0~64，无单位
            self.dirLeft = 0  # 左轮方向，0--前进，1--后退
            self.dirRight = 0  # 右轮方向，0--前进，1--后退
            self.disLeft = 0  # 左轮编码器脉冲数，x1.8后可以作为厘米距离
            self.disRight = 0  # 右轮编码器脉冲数，x1.8后可以作为厘米距离
            self.isBrakeLeft = 0  # 左轮是否刹车，无作用
            self.isBrakeRight = 0  # 右轮是否刹车，无作用
            self.resetEncoderLeft = 0  # 左轮编码器清零
            self.resetEncoderRight = 0  # 右轮编码器清零

    def __init__(self, port_path, baudrate=9600, timeout=0.01):
        """
        初始化串口设置，初始化待发送指令
        :param port_path: linux--串口绝对路径；windows--COMXX，均为字符串
        :param baudrate: 波特率
        """
        self.ser = serial.Serial(port=port_path, baudrate=baudrate, timeout=timeout)
        self.port = port_path

        self.instr = self.Instruction()
        self.FRAME_LEN = 12  # 数据帧长度
        self.recv_buffer = np.zeros(self.FRAME_LEN, dtype='uint8')

    def write_instruction(self, instr_array):
        """
        写入待发送指令
        :param instr_array: 指令数组，定义见本函数体，默认数据类型
        """
        self.instr.velLeft = instr_array[0]
        self.instr.velRight = instr_array[1]
        self.instr.dirLeft = instr_array[2]
        self.instr.dirRight = instr_array[3]
        self.instr.disLeft = instr_array[4]
        self.instr.disRight = instr_array[5]
        self.instr.isBrakeLeft = instr_array[6]
        self.instr.isBrakeRight = instr_array[7]
        self.instr.resetEncoderLeft = instr_array[8]
        self.instr.resetEncoderRight = instr_array[9]

    def send_instruction(self):
        """
        向串口发送一个指令，调用一次则发送一次
        :return: 0--发送成功；1--发送失败
        """
        # 修正速度
        if self.instr.velLeft > 64:
            self.instr.velLeft = 64
        if self.instr.velRight > 64:
            self.instr.velRight = 64

        BUFFER_SIZE = 6
        send_buffer = np.zeros(BUFFER_SIZE, dtype='uint8')
        send_buffer[0] = 0x5A
        send_buffer[5] = 0xA5
        send_buffer[1] = self.instr.velLeft
        send_buffer[2] = self.instr.velRight
        send_buffer[3] = self.instr.disLeft
        send_buffer[4] = self.instr.disRight
        send_buffer[3] <<= 2
        send_buffer[4] <<= 2

        # 左电机
        if self.instr.velLeft == 0:
            if self.instr.isBrakeLeft:
                send_buffer[3] |= 0x01
            else:
                send_buffer[3] &= 0xFE
        else:
            if self.instr.dirLeft == 1:
                send_buffer[3] |= 0x01
            else:
                send_buffer[3] &= 0xFE

        # 右电机
        if self.instr.velRight == 0:
            if self.instr.isBrakeRight:
                send_buffer[4] |= 0x01
            else:
                send_buffer[4] &= 0xFE
        else:
            if self.instr.dirRight == 1:
                send_buffer[4] |= 0x01
            else:
                send_buffer[4] &= 0xFE

        # 左右编码器
        if self.instr.resetEncoderLeft == 1:
            send_buffer[3] |= 0x02
        else:
            send_buffer[3] &= 0xFD
        if self.instr.resetEncoderRight == 1:
            send_buffer[4] |= 0x02
        else:
            send_buffer[4] &= 0xFD

        # 发送send_buffer
        if self.ser.write(send_buffer.tobytes()) == BUFFER_SIZE:
            return 0  # 正常返回

        # 异常返回
        return 1

    def receive_encoder_data(self, timeout):
        """
        接收串口传来的编码器数据，调用一次则接收一次。规定时间内未收到有效数据则报错。
        :return: 左电机编码器数据（x1.8后可作为厘米距离），右电机编码器数据，下位机系统时间（x100后可作为毫秒时间）
        """

        buffer = []
        time_start = time.time()
        time_run = 0
        while time_run < timeout:
            time_run = time.time() - time_start
            waiting = self.ser.in_waiting  # find num of bytes currently waiting in hardware
            buffer += self.ser.read(waiting)  # read them, convert to ascii
            if buffer.__len__() == 12:
                break

        self.recv_buffer = buffer
        if len(self.recv_buffer) < 12 or self.recv_buffer[0] != 0xA5 or self.recv_buffer[-1] != 0x5A:
            return None, None, None

        # 读取数据帧，帧包含三个数据
        # 左、右编码器累计脉冲数，各四个字节，x1.8后可作为厘米距离
        encoder_left = (self.recv_buffer[1] << 24) + (self.recv_buffer[2] << 16) \
                       + (self.recv_buffer[3] << 8) + self.recv_buffer[4]
        encoder_right = (self.recv_buffer[5] << 24) + (self.recv_buffer[6] << 16) \
                        + (self.recv_buffer[7] << 8) + self.recv_buffer[8]
        # 下位机系统时间，x100后可作为毫秒时间
        systime = (self.recv_buffer[9] << 8) + self.recv_buffer[10]
        return encoder_left, encoder_right, systime  # 正常退出

    def __del__(self):
        self.ser.__del__()


class MotorControl:
    def __init__(self, port, buadrate=9600):
        # 初始化串口
        self.WHEEL_DISTANCE = 466  # 轮距，mm
        self.sc = SerialController(port_path=port, baudrate=buadrate)

    def modifySpeed(self, LineSpeed, AngleSpeed, dirLeft=1, dirRight=0, disLeft=20, disRight=20):
        # 由线速度和角速度计算左右轮的速度
        vLeft = int(LineSpeed + 0.5 * AngleSpeed * self.WHEEL_DISTANCE)
        vRight = int(LineSpeed - 0.5 * AngleSpeed * self.WHEEL_DISTANCE)
        # 左右轮符号不一样，就原地旋转
        if vLeft < 0 and vRight >= 0:
            dirLeft = 1
            dirRight = 1
        elif vLeft >= 0 and vRight < 0:
            dirLeft = 1
            dirRight = 1
        # 都小于零，后退
        elif vLeft < 0 and vRight < 0:
            dirLeft = 0
            dirRight = 1
        # 都大于零，前进
        else:
            dirLeft = 1
            dirRight = 0

        # 设置指令
        # [1.左、右轮速度,3.左、右轮前进后退(不能相同，否则原地打转)，5.左、右轮一次指令最长运动距离，7,8无作用，9,10左右轮里程计清零，但方向改变是清零]
        instruction = [math.fabs(vLeft), math.fabs(vRight), dirLeft, dirRight, disLeft, disRight, 0, 0, 0, 0]
        # 写指令
        self.sc.write_instruction(instruction)
        # 发送指令
        self.sc.send_instruction()
        # # # # 接收回传的数据
        encoder_left_1, encoder_right_1, systime_1 = self.sc.receive_encoder_data(timeout=0.5)
        if encoder_left_1 is None :
            return 0,0
        # ****
        # time.sleep(0.1)
        # ****
        self.sc.send_instruction()
        encoder_left_2, encoder_right_2, systime_2 = self.sc.receive_encoder_data(timeout=0.5)
        if encoder_left_2 is None :
            return 0,0

        # 计算车体线速度、角速度
        distance_left = (encoder_left_2 - encoder_left_1) * 18  # mm
        distance_right = (encoder_right_2 - encoder_right_1) * 18  # mm
        d_time = (systime_2 - systime_1) * 0.1  # s
        v_lin_left = distance_left / d_time  # mm/s
        v_lin_right = distance_right / d_time  # mm/s
        V = (v_lin_left + v_lin_right) / 2  # mm/s
        W = (v_lin_left - v_lin_right) / self.WHEEL_DISTANCE  # 顺时针为正，弧度/s
        return V,W

    def modifySpeedForPathSimulation(self, LineSpeed, AngleSpeed, dirLeft=1, dirRight=0, disLeft=20, disRight=20):
        # 由线速度和角速度计算左右轮的速度
        vLeft = LineSpeed
        vRight = AngleSpeed
        # 左右轮符号不一样，就原地旋转
        if vLeft < 0 and vRight >= 0:
            dirLeft = 1
            dirRight = 1
        elif vLeft >= 0 and vRight < 0:
            dirLeft = 1
            dirRight = 1
        # 都小于零，后退
        elif vLeft < 0 and vRight < 0:
            dirLeft = 0
            dirRight = 1
        # 都大于零，前进
        else:
            dirLeft = 1
            dirRight = 0

        # 设置指令
        # [1.左、右轮速度,3.左、右轮前进后退(不能相同，否则原地打转)，5.左、右轮一次指令最长运动距离，7,8无作用，9,10左右轮里程计清零，但方向改变是清零]
        instruction = [math.fabs(vLeft), math.fabs(vRight), dirLeft, dirRight, disLeft, disRight, 0, 0, 0, 0]
        # 写指令
        self.sc.write_instruction(instruction)
        # 发送指令
        self.sc.send_instruction()
        # # # # 接收回传的数据
        encoder_left_1, encoder_right_1, systime_1 = self.sc.receive_encoder_data(timeout=0.5)
        if encoder_left_1 is None:
            return 0, 0
        # ****
        # time.sleep(0.1)
        # ****
        self.sc.send_instruction()
        encoder_left_2, encoder_right_2, systime_2 = self.sc.receive_encoder_data(timeout=0.5)
        if encoder_left_2 is None:
            return 0, 0

        # 计算车体线速度、角速度
        distance_left = (encoder_left_2 - encoder_left_1) * 18  # mm
        distance_right = (encoder_right_2 - encoder_right_1) * 18  # mm
        d_time = (systime_2 - systime_1) * 0.1  # s
        v_lin_left = distance_left / d_time  # mm/s
        v_lin_right = distance_right / d_time  # mm/s
        V = (v_lin_left + v_lin_right) / 2  # mm/s
        W = (v_lin_left - v_lin_right) / self.WHEEL_DISTANCE  # 顺时针为正，弧度/s
        return V, W

    def stop(self):
        instruction = [0, 0, 1, 0, 1, 1, 0, 0, 0, 0]
        # 写指令
        self.sc.write_instruction(instruction)
        # 发送指令
        self.sc.send_instruction()
        # 接收回传的数据
        encoder_left, encoder_right, systime = self.sc.receive_encoder_data(timeout=0.5)


if __name__ == '__main__':
    # 初始化串口
    sc = SerialController(port_path='COM8', baudrate=9600)
    # 设置指令
    # [1.vLeft,2.vRight,3.左轮方向，4.右轮方向，5.左轮走过距离，6.右轮走过距离，7,8无作用，9,10左右轮里程计清零]
    instruction = [39, 39, 1, 0, 32, 32, 0, 0, 0, 0]
    # 写指令
    sc.write_instruction(instruction)
    # 发送指令
    sc.send_instruction()

    # 接收回传的数据
    encoder_left_1, encoder_right_1, systime_1 = sc.receive_encoder_data(timeout=0.5)

    time.sleep(0.3)
    sc.send_instruction()
    encoder_left_2, encoder_right_2, systime_2 = sc.receive_encoder_data(timeout=0.5)
    # 计算车体线速度、角速度
    distance_left = (encoder_left_2 - encoder_left_1) * 18  # mm
    distance_right = (encoder_right_2 - encoder_right_1) * 18  # mm
    d_time = (systime_2 - systime_1) * 0.1  # s
    v_lin_left = distance_left / d_time  # mm/s
    v_lin_right = distance_right / d_time  # mm/s

    WHEEL_DISTANCE = 466  # 轮距，mm
    V = (v_lin_left + v_lin_right) / 2  # mm/s
    W = (v_lin_left - v_lin_right) / WHEEL_DISTANCE  # 顺时针为正，弧度/s
    print([V, W])
