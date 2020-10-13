import os
import sys
import cv2
import time
import math
import ctypes
import threading
import numpy as np
import numpy.ctypeslib as npct
from ctypes import *
from platform1 import Ui_Platform
from PyQt5.QtWidgets import QApplication, QMainWindow, QFileDialog, QMessageBox, QAction, QStatusBar
from PyQt5.QtWidgets import QGridLayout
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QCoreApplication, Qt, QUrl
from PyQt5.QtGui import QImage, QPixmap
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from lidar_neato import Lidar
# 雷达数据
from DealRadarData import deal_radar_data2, get_laser_data2

# 条件编译
WIN64 = 1
LINUX = 0
# 选择平台类型
flag = WIN64
# 激光雷达的端口
port = "/dev/ttyUSB0"
# 雷达是否用文件数据来模拟
isSIMULATION = True
# 电机是否是自动模式，即启动DWA算法
isAUTOMODE = True
# 申请线程锁
lock = threading.Lock()
theta = []
thro = []

# 定义DWA返回值为结构体是各部分的类型
class DWAResult(Structure):
    _fields_ = [("resV", c_double), ("resW", c_double)]

# 调用动态链接库
if flag:
    # 打开图片处理模块
    CUR_PATH = os.path.dirname(__file__)
    dllPath = os.path.join(CUR_PATH, "DynamicLinkLibrary/testDLL.dll")
    pDll = ctypes.WinDLL(dllPath)
    pDll.TransmitImage.argtypes = [npct.ndpointer(dtype=np.uint8, ndim=1, flags="C_CONTIGUOUS"),
                                   c_int, c_int, c_int]

    # 打开路径规划模块
    CUR_PATH = os.path.dirname(__file__)
    dllPath = os.path.join(CUR_PATH, "DynamicLinkLibrary/DWAport.dll")
    pDWA = ctypes.WinDLL(dllPath)
    pDWA.DWA_port.argtypes = [c_double, c_double, c_double, c_double, c_double, c_double, c_double,
                              npct.ndpointer(dtype=np.int32, ndim=1, flags="C_CONTIGUOUS"),
                              c_bool]  # ,npct.ndpointer(dtype = np.float64, ndim = 1, flags="C_CONTIGUOUS"),npct.ndpointer(dtype = np.float64, ndim = 1, flags="C_CONTIGUOUS")]
    # 设置返回值为结构体的类型
    pDWA.DWA_port.restype = (DWAResult)

else:
    CUR_PATH = os.path.dirname(__file__)
    soPath = os.path.join(CUR_PATH, "DynamicLinkLibrary/libtest.so")
    pDll = cdll.LoadLibrary(soPath)
    pDll.TransmitImage.argtypes = [npct.ndpointer(dtype=np.uint8, ndim=1, flags="C_CONTIGUOUS"),
                                   c_int, c_int, c_int]

    CUR_PATH = os.path.dirname(__file__)
    soPath = os.path.join(CUR_PATH, "DynamicLinkLibrary/libDWA.so")
    pDWA = cdll.LoadLibrary(soPath)
    pDWA.DWA_port.argtypes = [c_double, c_double, c_double, c_double, c_double, c_double, c_double,
                              npct.ndpointer(dtype=np.int32, ndim=1, flags="C_CONTIGUOUS"),
                              c_bool]
    pDWA.DWA_port.restype = (DWAResult)
    # 导入总线模块
    import smbus
    # 电机控制
    from robotMotor.myrobot import MyRobot
    if flag == LINUX:
        r = MyRobot()  # 初始化电机对象
    # 初始化smbus对象和设备地址
    motor_bus = smbus.SMBus(1)
    motor_addr = 0x30
'''
    申明函数onedemiarr, twodemiarr的传入参数类型；ndpointer为numpy.ctypeslib扩展库中提供的函数，
    可将numpy数组转为指针形式被C函数识别, 其中ndim表示数组维数，还有一个shape参数（可选）表示数组格式，
    flags = "C_CONTIGUOUS" 表示是和C语言相似的数组存放方式。
'''

# 电机参数初始化
# 常数定义
PI = 3.1416
WHEEL_DIAMETER = 66  # mm，车轮的直径
WHEEL_DISTANCE = 134  # mm，两轮的轴距


class platform(QMainWindow, Ui_Platform):
    def __init__(self, port, parent=None):
        super(platform, self).__init__(parent)
        self.OPEN = True          # 控制所有线程关闭
        self.setupUi(self)
        self.PrepWidgets()        # 初始化控件
        self.MenuInit()           # 初始化菜单
        self.CallBackFunctions()  # 各种控件的回调函数
        self.openCamera()         # 默认打开摄像头
        self.openMapImg()         # 默认加载地图
        # self.openBaiduMap()     # 打开百度地图
        self.openRadar(port)      # 启动线程准备读雷达数据

    def PrepWidgets(self):
        self.PrepCamera()
        self.pushButton_closeCamera.setEnabled(False)
        self.pushButton_closeRadar.setEnabled(False)
        # 创建一个关闭事件并设为未触发
        self.stopEvent = threading.Event()
        self.stopEvent.clear()
        # self.label_information.setText("初始化\r\n成功")
        # self.label_information.setText("初始化成功\r\n")
        self.PrepareLineCanvas()

    def PrepareLineCanvas(self):
        # 先生成画布
        self.LineFigure = Figure_Canvas()
        # 添加一个栅格布局
        self.LineFigureLayout = QGridLayout(self.LineDisplayGB)
        # 将画板添加到布局中去
        self.LineFigureLayout.addWidget(self.LineFigure)

    def MenuInit(self):
        # 创建一个菜单栏
        menu = self.menuBar()
        # # 创建一个状态栏
        # self.statusBar = QStatusBar()
        # self.setStatusBar(self.statusBar)
        # 创建具体菜单
        file_menu = menu.addMenu("file")
        file_menu.addSeparator()
        edit_menu = menu.addMenu('edit')
        # edit_menu.addSeparator()

        # 创建摄像头行为
        camera_action = QAction('打开/关闭摄像头', self)
        camera_action.setStatusTip("点击打开/关闭摄像头")
        camera_action.triggered.connect(self.changeCameraState)
        edit_menu.addAction(camera_action)

        # 创建退出行为
        exit_action = QAction('退出', self)
        # 退出操作
        exit_action.setStatusTip("点击退出应用程序")
        # 点击关闭程序
        exit_action.triggered.connect(self.exitApp)
        # # 设置退出快捷键
        # exit_action.setShortcut('Ctrl+z')
        # 添加退出行为到菜单上
        file_menu.addAction(exit_action)

        # 打开本地地图图片
        openMap_action = QAction('打开地图图片', self)
        openMap_action.setStatusTip("点击选择打开本地地图图片")
        openMap_action.triggered.connect(self.openMapImgSelfDefine)
        file_menu.addAction(openMap_action)

        # 打开在线地图
        openonlineMap_action = QAction('打开在线地图', self)
        openonlineMap_action.setStatusTip("点击打开在线地图（百度地图）")
        openonlineMap_action.triggered.connect(self.openBaiduMap)
        file_menu.addAction(openonlineMap_action)

    def PrepCamera(self):  # 测试摄像头是否正常工作
        try:
            self.camera = cv2.VideoCapture(0)
            # QMessageBox.information(self, "Tips", "初始化摄像头成功",
            #                         QMessageBox.Yes)
        except Exception as e:
            QMessageBox.warning(self, "warning", "cant find the camera",
                                QMessageBox.Cancel)
        self.camera.release()  # 检测完成释放摄像头

    def CallBackFunctions(self):
        self.pushButton_openImg.clicked.connect(self.openMapImgSelfDefine)  # 可以加载新的地图
        self.pushButton_exit.clicked.connect(self.exitApp)
        self.pushButton_openCamera.clicked.connect(self.openCamera)
        self.pushButton_closeCamera.clicked.connect(self.closeCamera)
        self.pushButton_editSpeed.clicked.connect(self.editSpeed)

    #  是否可更改成直接以地图作为背景，或者说有地图的接口可以显示地图
    def openMapImg(self):  # 选择本地图片显示

        self.label_mapImg.raise_()  # 把控件置到最顶层
        imgName = "image/njustmap.jpg"  # 这里为了方便别的地方引用图片路径，我们把它设置为全局变量
        map = QtGui.QPixmap(imgName).scaled(self.label_mapImg.width(),
                                            self.label_mapImg.height())  # 通过文件路径获取图片文件，并设置图片长宽为label控件的长宽
        self.label_mapImg.setPixmap(map)  # 在label控件上显示选择的图片
        # self.label_information.setText("已选择路径"+imgName+"\r\n打开图片成功")

    def openMapImgSelfDefine(self):  # 选择本地图片显示
        self.label_mapImg.raise_()  # 把控件置到最顶层
        global imgName  # 这里为了方便别的地方引用图片路径，我们把它设置为全局变量
        imgName, imgType = QFileDialog.getOpenFileName(self.centralwidget, "打开图片", "",
                                                       "*.jpg;;*.png;;All Files(*)")  # 弹出一个文件选择框，第一个返回值imgName记录选中的文件路径+文件名，第二个返回值imgType记录文件的类型
        map = QImage(imgName)
        result = map.scaled(self.label_mapImg.width(), self.label_mapImg.height(), Qt.IgnoreAspectRatio,
                            Qt.SmoothTransformation)
        self.label_mapImg.setPixmap(QPixmap.fromImage(result))  # 在label控件上显示选择的图片

        self.label_information.setText("已选择路径" + imgName + "\r\n打开图片成功")

    def openBaiduMap(self):
        self.browser.raise_()  # 把控件置到最顶层
        self.browser.load(QUrl(
            "https://map.baidu.com/search/%E5%8D%97%E4%BA%AC%E7%90%86%E5%B7%A5%E5%A4%A7%E5%AD%A6%E6%99%BA%E8%83%BD%E6%A5%BC/@13232298.72262961,3744402.220495053,19.15z?querytype=s&da_src=shareurl&wd=%E5%8D%97%E4%BA%AC%E7%90%86%E5%B7%A5%E5%A4%A7%E5%AD%A6%E6%99%BA%E8%83%BD%E6%A5%BC&c=315&src=0&pn=0&sug=0&l=19&b=(13231888.125,3744229.08;13232656.125,3744566.08)&from=webmap&biz_forward=%7B%22scaler%22:2,%22styles%22:%22pl%22%7D&device_ratio=2"))
        # self.label_information.setText("打开百度地图成功")

    def exitApp(self):

        self.OPEN = False
        if flag == LINUX:
            r.stop()  # 电机结束工作，停止运动（实际运用时，可以只在需要停下来时使用，其余时候根据指令改变转速即可）
            r.__del__()  # 析构电机对象
        QCoreApplication.quit()

    if flag == LINUX:
        def setLineSpeedAndAngleSpeed(self, float_lineSpeed, float_angleSpeed):
            V_lin = float_lineSpeed  # mm/s，需要达到的线速度
            V_ang = float_angleSpeed  # rad/s，需要达到的角速度

            V_left = V_lin - V_ang * WHEEL_DISTANCE / 2  # mm/s，换算左轮线速度
            V_right = V_lin + V_ang * WHEEL_DISTANCE / 2  # mm/s，换算右轮线速度

            RPS_left = V_left / (WHEEL_DIAMETER * PI)  # 每秒圈数，换算左轮转速
            RPS_right = V_right / (WHEEL_DIAMETER * PI)  # 每秒圈数，换算右轮转速
            r.set_motors(RPS_left * 0.1, RPS_right * 0.1)  # 以规定转速动作，0.1为常系数

    def editSpeed(self):
        # 读取文本框的值
        global edit_lineSpeed
        global edit_angleSpeed
        global isAUTOMODE
        edit_angleSpeed = edit_lineSpeed = 0.0
        edit_angleSpeed = self.lineEdit_angleSpeed.text()
        edit_lineSpeed = self.lineEdit_lineSpeed.text()
        if edit_angleSpeed == '':
            edit_angleSpeed = 0.0
        if edit_lineSpeed == '':
            edit_lineSpeed = 0.0

        isAUTOMODE = False
        # self.label_information.setText("设置线速度"+str(edit_lineSpeed)+"\r\n设置角速度"+str(edit_angleSpeed))
        # 设置电机线速度，角速度
        if flag == LINUX:
            self.setLineSpeedAndAngleSpeed(float(edit_lineSpeed), float(edit_angleSpeed))
        time.sleep(0.1)

    # 多线程更新UI数据，一个是摄像头，一个是雷达，在雷达里调用路径规划，并传输线速度和角速度。

    def changeCameraState(self):
        if self.cap.isOpened():
            self.closeCamera()
            # self.label_information.setText("关闭摄像头成功")
        else:
            self.openCamera()
            # self.label_information.setText("打开摄像头成功")

    # 每一次调用openCamera都会重新创建一个线程，最简单粗暴的办法是把这两个按钮取消掉
    def openCamera(self):
        # 根据自己相机改
        # self.cap = cv2.VideoCapture("rtsp://username:passport@ip:port/Streaming/Channels/1")
        self.cap = cv2.VideoCapture(0)  # , cv2.CAP_DSHOW

        # 创建视频显示线程
        th = threading.Thread(target=self.Display)
        th.start()

    def closeCamera(self):
        # 关闭事件设为触发，关闭视频播放
        self.stopEvent.set()
        # self.camera.release()  # 检测完成释放摄像头

    def openRadar(self, port):
        # ****
        # 初始化小车的线速度、角速度、起始坐标
        self.curV = 0
        self.curW = 0
        self.resV = 0
        self.resW = 0
        self.curX = 0
        self.curY = 0
        # ****目标的位置应该从外边传入
        self.tarX = 6000
        self.tarY = 6000
        self.findTarget = False
        self.curTheta = 0

        # 初始化雷达 ****
        if isSIMULATION == False:
            self.L = Lidar(port)
            self.L.activate()
        # 启动雷达数据可视化线程
        self.threadDisplayRadar = threading.Thread(target=self.DisplayRadar)
        self.threadDisplayRadar.start()
        # 启动线速度，角速度更新线程，调用DWA模块
        if flag == LINUX:
            self.threadRenewSpeed = threading.Thread(target=self.RenewSpeed)
            self.threadRenewSpeed.start()

    if flag == LINUX:
        def RenewSpeed(self):
            global theta,thro
            while self.OPEN and isAUTOMODE:
                # if isAUTOMODE == False:
                #     # self.label_information.setText("exit autoMode!")
                #     break
                # 读取雷达数据
                if isSIMULATION == False:
                    with lock:
                        thro, theta = self.L.get_data()
                        # [pi,2*pi] -> [-pi,0]
                        obstacle = theta[180:360] + theta[0:180]

                        # 获得当前的线速度，角速度
                        # 通过sm总线读取电机转速，单位--0.1rps
                        left_speed, right_speed, voltage = motor_bus.read_byte(motor_addr), motor_bus.read_byte(
                            motor_addr), motor_bus.read_byte(motor_addr)
                        # 转速换算至线速度
                        v_lin_left = 0.1 * left_speed * (WHEEL_DIAMETER * PI)
                        v_lin_right = 0.1 * right_speed * (WHEEL_DIAMETER * PI)
                        self.curV = (v_lin_left + v_lin_right) / 2
                        self.curW = (v_lin_left - v_lin_right) / WHEEL_DISTANCE

                        # 计算当前坐标，再传入。
                        self.curX = self.curX + self.resV * 0.2 * math.cos(self.curTheta)
                        self.curY = self.curY + self.resW * 0.2 * math.sin(self.curTheta)
                        self.curTheta = self.curTheta + self.curW * 0.2

                        # 判断是否找到目标，可以再讨论****
                        if math.fabs((self.curX - self.tarX)) < 300 and math.fabs((self.curY - self.tarY)) < 300:
                            self.findTarget = True

                        DWAres = pDWA.DWA_port(self.curV, self.curW, self.curX, self.curY, self.curTheta, self.tarX,
                                               self.tarY,
                                               np.array(obstacle, dtype=c_int32),
                                               self.findTarget)
                        self.resV = DWAres.resV
                        self.resW = DWAres.resW
                        # 传输线速度角速度,控制小车.
                        self.setLineSpeedAndAngleSpeed(DWAres.resV, DWAres.resW)
                else:
                    with lock:
                        thro, theta = get_laser_data2()
                        # [pi,2*pi] -> [-pi,0]
                        obstacle = theta[180:360] + theta[0:180]

                        # 获得当前的线速度，角速度
                        # 通过sm总线读取电机转速，单位--0.1rps
                        left_speed, right_speed, voltage = motor_bus.read_byte(motor_addr), motor_bus.read_byte(
                            motor_addr), motor_bus.read_byte(motor_addr)
                        # 转速换算至线速度
                        v_lin_left = 0.1 * left_speed * (WHEEL_DIAMETER * PI)
                        v_lin_right = 0.1 * right_speed * (WHEEL_DIAMETER * PI)
                        self.curV = (v_lin_left + v_lin_right) / 2
                        self.curW = (v_lin_left - v_lin_right) / WHEEL_DISTANCE

                        # 计算当前坐标，再传入。
                        self.curX = self.curX + self.resV * 0.2 * math.cos(self.curTheta)
                        self.curY = self.curY + self.resW * 0.2 * math.sin(self.curTheta)
                        self.curTheta = self.curTheta + self.curW * 0.2

                        # 判断是否找到目标，可以再讨论****
                        if math.fabs((self.curX - self.tarX)) < 300 and math.fabs((self.curY - self.tarY)) < 300:
                            self.findTarget = True

                        DWAres = pDWA.DWA_port(self.curV, self.curW, self.curX, self.curY, self.curTheta, self.tarX, self.tarY,
                                               np.array(obstacle, dtype=c_int32),
                                               self.findTarget)
                        self.resV = DWAres.resV
                        self.resW = DWAres.resW
                        # 传输线速度角速度,控制小车.
                        self.setLineSpeedAndAngleSpeed(DWAres.resV, DWAres.resW)

    def DisplayRadar(self):
        global theta,thro
        while self.OPEN:
            # if not self.open:
            #     break
            # 读取真实雷达数据
            if isSIMULATION == False:
                with lock:
                    thro, theta = self.L.get_data()
            else:
                # 模拟数据
                with lock:
                    thro, theta = get_laser_data2()
            # [pi,2*pi] -> [-pi,0]
            # obstacle = theta[180:360] + theta[0:180]
            # # 获得当前的线速度，角速度
            # left_speed, right_speed, voltage = r.getMotorSpeed(motor_bus, motor_addr)
            # self.curV = (left_speed + right_speed)/2
            # self.curW = (left_speed - right_speed)/WHEEL_DISTANCE
            #
            # # 计算当前坐标，再传入。
            # self.curX = self.curX + self.resV * 0.2 * math.cos(self.curTheta)
            # self.curY = self.curY + self.resW * 0.2 * math.sin(self.curTheta)
            # self.curTheta = self.curTheta + self.curW * 0.2
            #
            # # 判断是否找到目标，可以再讨论****
            # if math.fabs((self.curX - self.tarX)) < 300 and math.fabs((self.curY - self.tarY)) < 300:
            #     self.findTarget = True
            #
            # DWAres = pDWA.DWA_port(self.curV,self.curW,self.curX,self.curY,self.curTheta,self.tarX,self.tarY,np.array(obstacle,dtype = c_int32),self.findTarget) # ,np.array(self.resV,dtype = c_double),np.array(self.resW,dtype = c_double))
            # self.resV = DWAres.resV
            # self.resW = DWAres.resW
            # # 传输线速度角速度,控制小车.
            # self.setLineSpeedAndAngleSpeed(DWAres.resV, DWAres.resW)

            # 显示雷达数据到radarLabel ****
            # 坐标转换
            # 极坐标转换成直角坐标，并求出正前方的点

            tmpx, tmpy, midx, midy = deal_radar_data2(theta, thro)

            # 获得当前的线速度，角速度
            if flag == LINUX:
                # 通过sm总线读取电机转速，单位--0.1rps
                left_speed, right_speed, voltage = motor_bus.read_byte(motor_addr), motor_bus.read_byte(
                    motor_addr), motor_bus.read_byte(motor_addr)
                # 转速换算至线速度
                v_lin_left = 0.1 * left_speed * (WHEEL_DIAMETER * PI)
                v_lin_right = 0.1 * right_speed * (WHEEL_DIAMETER * PI)
                self.curV = (v_lin_left + v_lin_right) / 2
                self.curW = (v_lin_left - v_lin_right) / WHEEL_DISTANCE
            # 显示线速度，角速度
            self.label_lineSpeed.setText(str(self.curV) + 'mm/s')
            self.label_angleSpeed.setText(str(self.curW) + 'rad/s')

            # 先清空画布再绘制散点图
            self.LineFigure.ax.clear()
            self.LineFigure.ax.scatter(tmpx, tmpy)
            self.line = Line2D([midx, 0], [midy, 0], color='red', linewidth=5)
            self.LineFigure.ax.add_line(self.line)
            self.LineFigure.ax.axis("off")
            self.LineFigure.draw()
            time.sleep(0.1)
            if not self.OPEN:
                break
    # # 废弃
    # def deal_radar_data(self, angle, thro):
    #     count = 0
    #     limit_num = 1024
    #     factor = math.pi / 180
    #     mid_pointx = []
    #     mid_pointy = []
    #     x_pos = []
    #     y_pos = []
    #     for v in range(0, len(angle)):
    #         x_pos.append(thro[v] * math.cos(angle[v] * factor + math.pi / 2))
    #         y_pos.append(thro[v] * math.sin(angle[v] * factor + math.pi / 2))
    #         count += 1
    #         if angle[v] >= 0 and angle[v] < 10 and thro[v] > 0:
    #             mid_pointx.append(x_pos[len(x_pos) - 1])
    #             mid_pointy.append(y_pos[len(y_pos) - 1])
    #         if count > limit_num:
    #             break
    #     # 求正前方的点
    #     num = len(mid_pointx)
    #     for i in range(1, num):
    #         mid_pointx[0] += mid_pointx[i]
    #         mid_pointy[0] += mid_pointy[i]
    #     mid_pointx[0] /= num
    #     mid_pointy[0] /= num
    #     return x_pos, y_pos, mid_pointx[0], mid_pointy[0]

    # 每隔一帧检测是否点击关闭按钮

    def Display(self):
        self.pushButton_closeCamera.setEnabled(True)
        self.pushButton_openCamera.setEnabled(False)

        while self.OPEN and self.cap.isOpened():

            # 次数改成调用链接库，不断读取数组。
            if self.OPEN and self.cap.isOpened():
                success, frame = self.cap.read()
                if (frame is None) or (len(frame)<1):
                    break
            # RGB转BGR
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            # **** 模拟调用第四组的代码
            ary = np.array(frame, dtype=c_uint8)
            width, height, depth = ary.shape
            # 注意C++开的数组可能不够大，发生越界错，480*640*3
            pDll.TransmitImage(ary.flatten(), width, height, depth)
            # ****
            img = QImage(frame.data, frame.shape[1], frame.shape[0], QImage.Format_RGB888)
            self.label_Camera.setPixmap(QPixmap.fromImage(img))

            # 判断关闭事件是否已触发
            if (not self.OPEN) or self.stopEvent.is_set():
                # 关闭事件置为未触发，清空显示label
                self.stopEvent.clear()
                self.label_Camera.clear()
                self.pushButton_closeCamera.setEnabled(False)
                self.pushButton_openCamera.setEnabled(True)
                self.cap.release()  # 检测完成释放摄像头
                break



# 定义画布
class Figure_Canvas(FigureCanvas):
    def __init__(self, parent=None, width=3.9, height=2.7, dpi=40):
        self.fig = Figure(figsize=(width, height), dpi=40)
        # self.fig.figimage([0,1,2,3],cmap = plt.get_cmap('autumn'))
        super(Figure_Canvas, self).__init__(self.fig)
        self.ax = self.fig.add_subplot(111)

    def test(self):
        x = [1, 2, 3, 4, 5, 6, 7]
        y = [2, 1, 3, 5, 6, 4, 3]
        self.ax.plot(x, y)


if __name__ == '__main__':
    try:
        app = QApplication(sys.argv)
        ui = platform(port)
        ui.show()
        sys.exit(app.exec_())
    except:
        if flag == LINUX:
            r.stop()
            r.__del__()
        sys.exit(-1)
# if __name__ == '__main__':
#     app = QApplication(sys.argv)
#     mainWnd = QMainWindow()
#     ui = DisplayUI.Ui_MainWindow()
#
#     # 可以理解成将创建的 ui 绑定到新建的 mainWnd 上
#     ui.setupUi(mainWnd)
#
#     display = Display(ui, mainWnd)
#
#     mainWnd.show()
#
#     sys.exit(app.exec_())


# ****调用C语言的动态链接库
''' 
CUR_PATH = os.path.dirname(__file__)
dllPath = os.path.join(CUR_PATH,"DynamicLinkLibarary/testDLL.dll")
pDll = ctypes.WinDLL(dllPath)
pDll.TransmitImage.argtypes = [npct.ndpointer(dtype = np.uint8, ndim = 1, flags="C_CONTIGUOUS"),
    c_int,c_int,c_int]
'''

# ****
'''
    申明函数onedemiarr, twodemiarr的传入参数类型；ndpointer为numpy.ctypeslib扩展库中提供的函数，
    可将numpy数组转为指针形式被C函数识别, 其中ndim表示数组维数，还有一个shape参数（可选）表示数组格式，
    flags = "C_CONTIGUOUS" 表示是和C语言相似的数组存放方式。
'''