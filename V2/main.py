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
from robotMotor.serialctrl import MotorControl
# 雷达数据
from DealRadarData import deal_radar_data, get_laser_data

# 在速度更新线程，增加条件判断，按空格键暂停程序


# 条件编译
WIN64 = 1
LINUX = 0
# 选择平台类型
flag = LINUX

# 雷达是否用文件数据来模拟
isSIMULATION = False
# 是否用模拟路径
USINGSIMULATION = False
# 电机是否是自动模式，若是则启动DWA算法
isAUTOMODE = True
# 端口设置
if flag == WIN64:
    LidarPort = "COM6"
    motorPort = "COM7"
else:
    # 激光雷达的端口
    LidarPort = "/dev/ttyUSB0"
    # 电机串口的端口编号
    motorPort = "/dev/ttyUSB1"
# 申请线程锁
lock = threading.Lock()
theta = []
thro = []
# 电机控制对象
myMotor = MotorControl(motorPort)


# 定义Lidar数据的长度，取决于DLL模块内部的定义，这里只是为了保持一致
LIDAR_DATA_LENGTH = 1800


# 定义getLidarData返回类型的结构体
class LidarData(Structure):
    _fields_ = [("angle", c_double*LIDAR_DATA_LENGTH), ("dist", c_double*LIDAR_DATA_LENGTH)]


# 定义DWA返回值为结构体是各部分的类型
class DWAResult(Structure):
    _fields_ = [("resV", c_double), ("resW", c_double)]



# 调用动态链接库
if flag == WIN64:
    # 打开图片处理模块
    CUR_PATH = os.path.dirname(__file__)
    dllPath = os.path.join(CUR_PATH, "DynamicLinkLibrary/testDLL.dll")
    pDll = ctypes.WinDLL(dllPath)
    pDll.TransmitImage.argtypes = [npct.ndpointer(dtype=np.uint8, ndim=1, flags="C_CONTIGUOUS"),
                                   c_int, c_int, c_int]


    # 打开激光雷达模块
    CUR_PATH = os.path.dirname(__file__)
    dllPath = os.path.join(CUR_PATH, "DynamicLinkLibrary/Lidar.dll")
    pLidar = ctypes.WinDLL(dllPath)
    pLidar.lidarInit.argtypes = [c_char_p,c_uint32]
    pLidar.getLidarData.restype = (LidarData)



    # 打开路径规划模块
    CUR_PATH = os.path.dirname(__file__)
    dllPath = os.path.join(CUR_PATH, "DynamicLinkLibrary/DWAport.dll")
    pDWA = ctypes.WinDLL(dllPath)
    pDWA.DWA_port.argtypes = [c_double, c_double, c_double, c_double, c_double, c_double, c_double,
                              npct.ndpointer(dtype=np.double, ndim=1, flags="C_CONTIGUOUS"),
                              npct.ndpointer(dtype=np.double, ndim=1, flags="C_CONTIGUOUS"),
                              c_bool]  # ,npct.ndpointer(dtype = np.float64, ndim = 1, flags="C_CONTIGUOUS"),npct.ndpointer(dtype = np.float64, ndim = 1, flags="C_CONTIGUOUS")]
    # 设置返回值为结构体的类型
    pDWA.DWA_port.restype = (DWAResult)




elif flag == LINUX:
    CUR_PATH = os.path.dirname(__file__)
    soPath = os.path.join(CUR_PATH, "DynamicLinkLibrary/libtest.so")
    pDll = cdll.LoadLibrary(soPath)
    pDll.TransmitImage.argtypes = [npct.ndpointer(dtype=np.uint8, ndim=1, flags="C_CONTIGUOUS"),
                                   c_int, c_int, c_int]

    CUR_PATH = os.path.dirname(__file__)
    soPath = os.path.join(CUR_PATH, "DynamicLinkLibrary/liblidar.so")
    pLidar = cdll.LoadLibrary(soPath)
    pLidar.lidarInit.argtypes = [c_char_p, c_uint32]
    pLidar.getLidarData.restype = (LidarData)


    CUR_PATH = os.path.dirname(__file__)
    soPath = os.path.join(CUR_PATH, "DynamicLinkLibrary/libDWA.so")
    pDWA = cdll.LoadLibrary(soPath)
    pDWA.DWA_port.argtypes = [c_double, c_double, c_double, c_double, c_double, c_double, c_double,
                              npct.ndpointer(dtype=np.double, ndim=1, flags="C_CONTIGUOUS"),
                              npct.ndpointer(dtype=np.double, ndim=1, flags="C_CONTIGUOUS"),
                              c_bool]
    pDWA.DWA_port.restype = (DWAResult)

'''
    申明函数onedemiarr, twodemiarr的传入参数类型；ndpointer为numpy.ctypeslib扩展库中提供的函数，
    可将numpy数组转为指针形式被C函数识别, 其中ndim表示数组维数，还有一个shape参数（可选）表示数组格式，
    flags = "C_CONTIGUOUS" 表示是和C语言相似的数组存放方式。
'''




class platform(QMainWindow, Ui_Platform):
    def __init__(self, LidarPort, parent=None):
        super(platform, self).__init__(parent)
        self.OPEN = True          # 控制所有线程打开与关闭
        self.PAUSE = False        # 是否暂停小车
        self.setupUi(self)
        self.PrepWidgets()        # 初始化控件
        self.MenuInit()           # 初始化菜单
        self.CallBackFunctions()  # 各种控件的回调函数
        self.openCamera()         # 默认打开摄像头
        self.openMapImg()         # 默认加载地图
        # self.openBaiduMap()     # 打开百度地图
        self.openRadar(LidarPort)      # 启动线程准备读雷达数据

    def PrepWidgets(self):
        self.PrepCamera()
        self.pushButton_closeCamera.setEnabled(False)
        self.pushButton_closeRadar.setEnabled(False)
        # 创建一个关闭事件并设为未触发
        self.stopEvent = threading.Event()
        self.stopEvent.clear()
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
        # openonlineMap_action = QAction('打开在线地图', self)
        # openonlineMap_action.setStatusTip("点击打开在线地图（百度地图）")
        # openonlineMap_action.triggered.connect(self.openBaiduMap)
        # file_menu.addAction(openonlineMap_action)

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

    def keyPressEvent(self, a0: QtGui.QKeyEvent) -> None:
        if(a0.key() == Qt.Key_Q):
            self.PAUSE = not self.PAUSE

    #  是否可更改成直接以地图作为背景，或者说有地图的接口可以显示地图
    def openMapImg(self):  # 选择本地图片显示
        self.label_mapImg.raise_()  # 把控件置到最顶层
        imgName = "image/njustmap.jpg"  # 这里为了方便别的地方引用图片路径，我们把它设置为全局变量
        map = QtGui.QPixmap(imgName).scaled(self.label_mapImg.width(),
                                            self.label_mapImg.height())  # 通过文件路径获取图片文件，并设置图片长宽为label控件的长宽
        self.label_mapImg.setPixmap(map)  # 在label控件上显示选择的图片

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
        # 整个程序结束，关闭雷达
        pLidar.closeLidar()
        myMotor.stop()  # 电机结束工作，停止运动（实际运用时，可以只在需要停下来时使用，其余时候根据指令改变转速即可）
        QCoreApplication.quit()



    def editSpeed(self):
        # 读取文本框的值
        global edit_lineSpeed
        global edit_angleSpeed
        global isAUTOMODE
        edit_angleSpeed = edit_lineSpeed = 0.0
        # 从文本框中获取输入的线速度和角速度
        edit_angleSpeed = self.lineEdit_angleSpeed.text()
        edit_lineSpeed = self.lineEdit_lineSpeed.text()
        if edit_angleSpeed == '':
            edit_angleSpeed = 0.0
        if edit_lineSpeed == '':
            edit_lineSpeed = 0.0

        isAUTOMODE = False
        # 设置电机线速度，角速度
        curV,curW = myMotor.modifySpeed(float(edit_lineSpeed), float(edit_angleSpeed))
        # 显示线速度，角速度
        self.label_lineSpeed.setText(str(curV) + 'mm/s')
        self.label_angleSpeed.setText(str(curW) + 'rad/s')


    # 多线程更新UI数据，一个是摄像头，一个是雷达，在雷达里调用路径规划，并传输线速度和角速度。
    def changeCameraState(self):
        if self.cap.isOpened():
            self.closeCamera()
        else:
            self.openCamera()

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

    def openRadar(self, LidarPort):
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
            # self.L = Lidar(LidarPort)
            # self.L.activate()
            pLidar.lidarInit(c_char_p(LidarPort.encode("ascii")),256000)

        # 启动雷达数据可视化线程
        self.threadDisplayRadar = threading.Thread(target=self.DisplayRadar)
        self.threadDisplayRadar.start()

        # 启动线速度，角速度更新线程，调用DWA模块
        self.threadRenewSpeed = threading.Thread(target=self.RenewSpeed)
        self.threadRenewSpeed.start()
    # 模拟路径
    def pathSimulation(self,count):
        # 前五秒前进
        if count <= 5:
            return 30,30
        # 五秒右转
        elif count > 5 and count <= 10:
            return 40,10
        # 五秒左转
        elif count > 10 and count <= 15:
            return 10, 40
        # 五秒后退
        elif count >15 and count <= 20:
            return -30,-30
        # 其他停机
        else:
            return 0,0

    # *********************************************************
    def RenewSpeed(self):
        global theta,thro
        count = 0
        while self.OPEN and isAUTOMODE:
            # 访问雷达数据
            if self.OPEN and isAUTOMODE:
                with lock:
                    # 只有OPEN的情况下，雷达才可以读；只有自动模式下，规划线程才需要自己去读数据。后面有时间单独写一个雷达的读取线程！！！
                    if self.OPEN and isAUTOMODE:
                        theta = []
                        thro = []
                        lidarRes = pLidar.getLidarData()
                        for i in range(0, LIDAR_DATA_LENGTH):
                            theta.append(lidarRes.angle[i])
                            thro.append(lidarRes.dist[i])
                # 计算当前坐标，再传入。
                self.curX = self.curX + self.resV * 0.2 * math.cos(self.curTheta)
                self.curY = self.curY + self.resW * 0.2 * math.sin(self.curTheta)
                self.curTheta = self.curTheta + self.curW * 0.2

                # 判断是否找到目标，可以再讨论****
                if math.fabs((self.curX - self.tarX)) < 300 and math.fabs((self.curY - self.tarY)) < 300:
                    self.findTarget = True

                # *********************************************************************************************
                if USINGSIMULATION:
                    V,W = self.pathSimulation(count)
                    # 传输线速度角速度,控制小车.
                    if not self.PAUSE:
                        self.curV,self.curW = myMotor.modifySpeedForPathSimulation(V ,W)
                        count += 1
                        time.sleep(1)
                    else :
                        self.curV, self.curW = myMotor.modifySpeedForPathSimulation(0,0)


                else:
                    DWAres = pDWA.DWA_port(self.curV, self.curW, self.curX, self.curY, self.curTheta, self.tarX,
                                           self.tarY,
                                           np.array(thro, dtype=c_double),
                                           np.array(theta, dtype=c_double),
                                           self.findTarget)
                    self.resV = DWAres.resV*6
                    self.resW = DWAres.resW
                    # 传输线速度角速度,控制小车.
                    if not self.PAUSE:
                        self.curV,self.curW = myMotor.modifySpeed(self.resV ,self.resW)
                    else :
                        self.curV, self.curW = myMotor.modifySpeed(0,0)
                # 显示线速度，角速度
                self.label_lineSpeed.setText(str(self.resV) + 'mm/s')
                self.label_angleSpeed.setText(str(self.resW) + 'rad/s')
            if not self.OPEN or not isAUTOMODE:
                break



    # 只要规划的速度和雷达显示的速度差不多，就不用了再分成两个线程了
    def DisplayRadar(self):
        global theta,thro
        while self.OPEN:
            # 手动模式下，显示线程才需要自己去读激光雷达
            if not isAUTOMODE:
                with lock:
                    theta = []
                    thro = []
                    if self.OPEN:
                        # 读雷达数据
                        lidarRes = pLidar.getLidarData()
                        # 转换成列表，方便处理。此处可进一步优化
                        for i in range(0, LIDAR_DATA_LENGTH):
                            theta.append(lidarRes.angle[i])
                            thro.append(lidarRes.dist[i])

            if theta != [] and thro != []:
                # 由极坐标系生成局部直角坐标系
                tmpx, tmpy, start_x, start_y = self.deal_radar_data(theta, thro)
                # 先清空画布再绘制散点图
                self.LineFigure.ax.clear()
                self.LineFigure.ax.scatter(tmpx, tmpy)
                self.line = Line2D([start_x, 0], [start_y, 0], color='red', linewidth=5)
                self.LineFigure.ax.add_line(self.line)
                self.LineFigure.ax.axis("off")
                self.LineFigure.draw()

    def deal_radar_data(self,angle, thro):
        factor = math.pi / 180
        start_pointx = 0
        start_pointy = 0
        x_pos = []
        y_pos = []
        for v in range(0, len(angle)):
            if (angle[v] > 0 and angle[v] < 0.6) and thro[v] > 0:
                start_pointx = thro[v] * math.cos(angle[v] * factor + math.pi / 2)
                start_pointy = (thro[v] * math.sin(angle[v] * factor + math.pi / 2))
                break
        for v in range(0, len(angle)):
            x_pos.append(thro[v] * math.cos(angle[v] * factor + math.pi / 2))
            y_pos.append(thro[v] * math.sin(angle[v] * factor + math.pi / 2))
            # 找正前方的点
        return x_pos, y_pos, start_pointx, start_pointy

    def Display(self):
        self.pushButton_closeCamera.setEnabled(True)
        self.pushButton_openCamera.setEnabled(False)

        while self.OPEN and self.cap.isOpened():

            # 次数改成调用链接库，不断读取数组。
            if self.OPEN and self.cap.isOpened():
                success, frame = self.cap.read()
                if (frame is None) or (len(frame) < 1):
                    break
                # RGB转BGR
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                # 模拟调用第四组的代码
                ary = np.array(frame, dtype=c_uint8)
                width, height, depth = ary.shape
                # 注意C++开的数组可能不够大，发生越界错，480*640*3
                pDll.TransmitImage(ary.flatten(), width, height, depth)
                # 转换成QImage
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
        # 设置画布背景
        # self.fig.figimage([0,1,2,3],cmap = plt.get_cmap('autumn'))
        super(Figure_Canvas, self).__init__(self.fig)
        self.ax = self.fig.add_subplot(111)

    def test(self):
        x = [1, 2, 3, 4, 5, 6, 7]
        y = [2, 1, 3, 5, 6, 4, 3]
        self.ax.plot(x, y)


if __name__ == '__main__':
    # try:
        app = QApplication(sys.argv)
        ui = platform(LidarPort)
        ui.show()
        sys.exit(app.exec_())
    # except:
    #     myMotor.stop()
    #     sys.exit(-1)

