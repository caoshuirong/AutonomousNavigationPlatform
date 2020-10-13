import smbus


def getMotorSpeed(bus, addr=0x30):
    '''从smbus上读取左电机速度、右电机速度、电池电压并返回
    输入：bus -- smbus对象；addr -- 设备地址（默认0x30）
    输出：left_speed -- 左电机速度；right_speed -- 右电机速度；voltage -- 电池电压
    '''
    left_speed, right_speed, voltage = bus.read_byte(addr), bus.read_byte(addr), bus.read_byte(addr)
    return left_speed, right_speed, voltage


if __name__ == '__main__':
    # 初始化smbus对象和设备地址
    motor_bus = smbus.SMBus(1)
    motor_addr = 0x30
    
    # 读一次并打印
    left_speed, right_speed, voltage = getMotorSpeed(motor_bus, motor_addr)
    print("%d, %d, %d" % (left_speed, right_speed, voltage))