
# 读取雷达的数据并显示,单位毫米
import math


def get_laser_data2():
    x_pos = []
    y_pos = []
    mid_pointx = []
    mid_pointy = []
    tmp = []
    thro = []
    theta = []
    with open("exp.txt",mode = 'r',encoding = 'utf-8' ) as f:
        lines = f.readlines()
    for i in range(0,36):
        thro.extend(list(map(float,lines[i].split())))
    del lines[0:36]
    for i in range(0,60):
        theta.extend(list(map(float,lines[i].split())))
    return thro,theta

    # for i in range(0,360):
    #     x_pos.append(thro[i] * math.cos(theta[i] + math.pi / 2))
    #     y_pos.append(thro[i] * math.sin(theta[i] + math.pi / 2))
    #     if ((theta[i] >= 0 and theta[i] < 0.314*3 ) or (theta[i] >= 5.6 and theta[i] < 6.3) and thro[i] > 0):
    #         mid_pointx.append(x_pos[len(x_pos) - 1])
    #         mid_pointy.append(y_pos[len(y_pos) - 1])
    # num = len(mid_pointx)
    # for i in range(1, num):
    #     mid_pointx[0] += mid_pointx[i]
    #     mid_pointy[0] += mid_pointy[i]
    # mid_pointx[0] /= num
    # mid_pointy[0] /= num
    # return x_pos,y_pos,mid_pointx[0],mid_pointy[0]

def get_laser_data():
    with open('lidardata.txt',mode = 'r',encoding = 'utf-8') as f:
        lines = f.readlines()
    del lines[0:3]
    flag = lines.pop(0)
    if flag == 'ok\n':
        flag = True
    else:
        flag = False

    vaild_data_num = int(lines.pop(0))
    tmp = []
    factor =  math.pi / 180
    count = 0
    mid_pointx = []
    mid_pointy = []
    angle = []
    thro = []

    for line in lines:
        tmp = line.split()
        float_data = list(map(float,tmp))
        angle.append(float_data[0])
        thro.append(float_data[1])
    x_pos,y_pos,mid_pointx,mid_pointy = deal_radar_data(angle,thro)
    return x_pos,y_pos,mid_pointx,mid_pointy

'''
    list assignment index out of range：列表超过限制
    一种情况是：list[index]index超出范围
    另一种情况是：list是一个空的，没有一个元素，进行list[0]就会出现错误！
'''

    #     x_pos.append(thro[1] * math.cos(thro[0] * factor + math.pi / 2))
    #     y_pos.append(thro[1] * math.sin(thro[0] * factor + math.pi / 2))
    #     count += 1
    #     if thro[0] >= 0 and thro[0] < 10 and thro[1] > 0:
    #         mid_pointx.append(x_pos[len(x_pos) - 1])
    #         mid_pointy.append(y_pos[len(y_pos) - 1])
    #     if count > limit_num:
    #         break
    #
    # num = len(mid_pointx)
    # for i in range(1,num):
    #     mid_pointx[0] += mid_pointx[i]
    #     mid_pointy[0] += mid_pointy[i]
    # mid_pointx[0] /= num
    # mid_pointy[0] /= num

    # ****


    # ****
    # return x_pos,y_pos,mid_pointx[0],mid_pointy[0]
    # p.plot([mid_pointx[0],0],[mid_pointy[0],0])
    # p.waitforbuttonpress()

def deal_radar_data2(theta,thro):
    count = 0
    limit_num = 1024
    factor = 180 / math.pi
    mid_pointx = [0]
    mid_pointy = [0]
    x_pos = []
    y_pos = []
    for v in range(0,len(theta)):
        x_pos.append(thro[v] * math.cos(theta[v] + math.pi / 2))
        y_pos.append(thro[v] * math.sin(theta[v] + math.pi / 2))
        count += 1
        # if ((theta[v] >= 0 and theta[v] < 0.314*3 ) or (theta[v] >= 5.6 and theta[v] < 6.3) and thro[v] > 0):
        if (((theta[v] >= 0 and theta[v] * factor < 10) or (theta[v] * factor >= 350 and theta[v] * factor < 360)) and
                thro[v] > 0):
            mid_pointx.append(x_pos[len(x_pos) - 1])
            mid_pointy.append(y_pos[len(y_pos) - 1])
        if count > limit_num:
            break
    # 求正前方的点
    num = len(mid_pointx)
    for i in range(1, num):
        mid_pointx[0] += mid_pointx[i]
        mid_pointy[0] += mid_pointy[i]
    mid_pointx[0] /= num
    mid_pointy[0] /= num
    return x_pos, y_pos, mid_pointx[0], mid_pointy[0]




def deal_radar_data(angle,thro):
    count = 0
    factor = math.pi / 180
    mid_pointx = 0
    mid_pointy = 0
    x_pos = []
    y_pos = []
    for v in range(0,len(angle)):
        if (angle[v]>0 and angle [v] < 0.6) and thro[v] > 0:
            mid_pointx = thro[v] * math.cos(angle[v] * factor + math.pi / 2)
            mid_pointy = (thro[v] * math.sin(angle[v] * factor + math.pi / 2))
            break
    for v in range(0,len(angle)):
        x_pos.append(thro[v] * math.cos(angle[v] * factor + math.pi / 2))
        y_pos.append(thro[v] * math.sin(angle[v] * factor + math.pi / 2))
        # 找正前方的点
    return x_pos, y_pos, mid_pointx, mid_pointy







# # 快捷键：跳到行尾：ALT+L 跳到行头 ：ALT+K  快速注释：ctrl + /
#
# class Human():
#     sumOfObject = 0
#     def __init__(self,name,age):
#         self.__name = name
#         self.__age = age
#         Human.addSum()
#
#     @classmethod
#     def addSum(cls):
#         cls.sumOfObject += 1
#         print(cls.sumOfObject)
#
# class Student(Human):
#     def __init__(this,name,age,school):
#         super(Student,this).__init__(name,age)
#         this.__school = school
#
#     def do_homework(self):
#         print("do homework")
#
#
# student1 = Student("石敢当",18,'njust')
# student1.do_homework()
# # print(student1.__dict__)
# # print(Student.__dict__)
# # print(Human.__dict__)
# print(type(student1))


# 枚举
#
# from enum import Enum
#
# from enum import unique
#
# @unique
# class VIP(Enum):
#     YELLOW = 1
#     RED = 2
#     WHITE = 3
#
#
# for v in VIP.__members__:
#     print(v)

# print(type(VIP(1)))

# print(VIP.YELLOW == VIP.YELLOW) # 只能在同一个类中通过枚举的类型进行等值比较

# 枚举的类型:VIP.YELLOW
# 枚举的名字：VIP.YELLOW.name
# 枚举的值：VIP.YELLOW.value

# 函数






