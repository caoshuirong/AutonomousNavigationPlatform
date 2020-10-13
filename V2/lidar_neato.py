#!/usr/bin/env python 
# -*- coding:utf-8 -*-

import serial
import time
import numpy as np


def checksum(data):
    """
    Compute and return the checksum as an int.
    :param data: list of 20 bytes, in the order they arrived in
    :return: The checksum computed.
    """
    # group the data by word, little-endian
    data_list = []
    for t in range(10):
        data_list.append(data[2 * t] + (data[2 * t + 1] << 8))

    # compute the checksum on 32 bits
    chk32 = 0
    for d in data_list:
        chk32 = (chk32 << 1) + d

    # return a value wrapped around on 15bits, and truncated to still fit into 15 bits
    check_sum = (chk32 & 0x7FFF) + (chk32 >> 15)  # wrap around to fit into 15 bits
    check_sum &= 0x7FFF  # truncate to 15 bits
    return int(check_sum)


class Lidar:

    def __init__(self, port_path):
        """尝试打开给出的串口
        输入：串口的绝对路径字符串"""
        self.ser = serial.Serial(port_path, 115200)
        self.port = port_path

    def activate(self):
        """尝试开启雷达，开启后雷达应当回传数据"""
        self.ser.write('startlds$'.encode('utf-8'))

        MAX_TRIAL = 500
        trial = 1
        while trial <= MAX_TRIAL:
            if ord(self.ser.read(1)) == 0xFA:
                print("Lidar successfully activated! Lidar starts working!")
                return True
            trial += 1
        print("Fail to activate the lidar. Too many failed trials.")
        return False

    def get_data(self):
        """获取雷达旋转一圈的数据，正常情况下应当获取到360个距离信息，两个距离间距一度
        雷达每转一圈会发送90个数据帧，每帧包含4个距离信息。数据帧长22字节，共1980字节
        输出：距离数组（毫米）、角度数组（弧度）"""
        SIZE_RAW_BYTES = 1980  # DO NOT CHANGE! 1980 = 90 x 22
        FRAME_LEN = 22  # DO NOT CHANGE!

        raw_bytes = np.zeros(SIZE_RAW_BYTES, dtype='uint8')
        data_acquired = False
        while not data_acquired:
            # 等待数据帧同步标志的到来: 0xFA (数据帧开始)
            raw_bytes[0] = ord(self.ser.read(1))
            if raw_bytes[0] != 0xFA:
                continue
            raw_bytes[1] = ord(self.ser.read(1))
            if raw_bytes[1] != 0xA0:
                continue

            # 已识别到数据帧，开始读取数据
            for t in range(2, SIZE_RAW_BYTES):
                raw_bytes[t] = ord(self.ser.read(1))

            # 读取每个数据帧，每帧包含四个距离数据
            distance = np.zeros(360)
            angle = np.linspace(0, 2 * np.pi, 360, endpoint=False)
            index = 0
            # 读取每一帧，外循环
            for i in range(0, SIZE_RAW_BYTES, FRAME_LEN):
                # 检查每一帧的校验和,若不同则全帧作废，四个距离置为0
                # expected_checksum = raw_bytes[i + 21] << 8 | raw_bytes[i + 20]
                # actual_checksum = raw_bytes[0] + raw_bytes[1] + raw_bytes[range(2, 22)]
                if False:  # 校验和函数有问题，暂不使用
                    distance[range(index, index + 4)] = 0
                else:
                    # 读取每帧的每一个距离数据，内循环
                    for j in range(i + 4, i + 20, 4):
                        index = (4 * i) // 22 + (j - 4 - i) // 4
                        byte0 = raw_bytes[j]
                        byte1 = raw_bytes[j + 1]
                        # byte2 = raw_bytes[j + 2]
                        # byte3 = raw_bytes[j + 3]
                        # 检查无效数据旗标，若为1则表示该距离无效，置为0
                        flag_inval = (byte1 & 0x80) >> 7
                        if flag_inval == 1:
                            distance[index] = 0
                        else:
                            # 提取距离信息，毫米
                            distance[index] = ((byte1 & 0x3F) << 8) + byte0
                index += 1
            data_acquired = True
        return distance, angle

    def __del__(self):
        self.ser.__del__()


# if __name__ == '__main__':
#     L = Lidar('COM17')
#     L.activate()
#     distance, angle = L.get_data()

