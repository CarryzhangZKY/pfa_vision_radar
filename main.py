import threading
import time
from collections import deque
import serial
from information_ui import draw_information_ui

    
import sys


import cv2
import numpy as np
from detect_function import YOLOv5Detector
from RM_serial_py.ser_api import  build_send_packet, receive_packet, Radar_decision, \
    build_data_decision, build_data_radar_all

state = 'B'  # R:红方/B:蓝方

if state == 'R':
    loaded_arrays = np.load('arrays_test_red.npy')  # 加载标定好的仿射变换矩阵
    map_image = cv2.imread("images/map_red.jpg")  # 加载红方视角地图
    mask_image = cv2.imread("images/map_mask.jpg")  # 加载红发落点判断掩码
else:
    loaded_arrays = np.load('arrays_test_blue.npy')  # 加载标定好的仿射变换矩阵
    map_image = cv2.imread("images/map_blue.jpg")  # 加载蓝方视角地图
    mask_image = cv2.imread("images/map_mask.jpg")  # 加载蓝方落点判断掩码

# 导入战场每个高度的不同仿射变化矩阵
M_height_r = loaded_arrays[1]  # R型高地
M_height_g = loaded_arrays[2]  # 环形高地
M_ground = loaded_arrays[0]  # 地面层、公路层

# 确定地图画面像素，保证不会溢出
height, width = mask_image.shape[:2]
height -= 1
width -= 1

# 初始化战场信息UI（标记进度、双倍易伤次数、双倍易伤触发状态）
information_ui = np.zeros((500, 420, 3), dtype=np.uint8) * 255
information_ui_show = information_ui.copy()
double_vulnerability_chance = -1  # 双倍易伤机会数
opponent_double_vulnerability = -1  # 是否正在触发双倍易伤
target = -1  # 飞镖当前瞄准目标（用于触发双倍易伤）
chances_flag = 1  # 双倍易伤触发标志位，需要从1递增，每小局比赛会重置，所以每局比赛要重启程序
progress_list = [-1, -1, -1, -1, -1, -1]  # 标记进度列表

# 加载战场地图
map_backup = cv2.imread("images/map.jpg")
map = map_backup.copy()

# 初始化盲区预测列表
guess_list = {
    "B1": True,
    "B2": True,
    "B3": True,
    "B4": True,
    "B5": True,
    "B6": True,
    "B7": True,
    "R1": True,
    "R2": True,
    "R3": True,
    "R4": True,
    "R5": True,
    "R6": True,
    "R7": True
}
# 上次盲区预测时的标记进度
guess_value = {
    "B1": 0,
    "B2": 0,
    "B7": 0,
    "R1": 0,
    "R2": 0,
    "R7": 0
}
# 当前标记进度（用于判断是否预测正确正确）
guess_value_now = {
    "B1": 0,
    "B2": 0,
    "B7": 0,
    "R1": 0,
    "R2": 0,
    "R7": 0
}

# 机器人名字对应ID
mapping_table = {
    "R1": 1,
    "R2": 2,
    "R3": 3,
    "R4": 4,
    "R5": 5,
    "R6": 6,
    "R7": 7,
    "B1": 101,
    "B2": 102,
    "B3": 103,
    "B4": 104,
    "B5": 105,
    "B6": 106,
    "B7": 107
}

# 盲区预测点位，如果没有定位模块，连接数服务器的非哨兵机器人坐标为（0,0）
guess_table = {
    "R1": [(1100, 1400), (900, 1400)],
    "R2": [(870, 1100), (1340, 680)],
    "R7": [(560, 630), (560, 870)],
    "B1": [(1700, 100), (1900, 100)],
    # "B1": [(0, 0), (19.0, 1.0)],
    "B2": [(1930, 400), (1460, 820)],
    "B7": [(2240, 870), (2240, 603)],
    # "B7": [(0, 0), (22.4, 6.3)]
}


# 机器人坐标滤波器（滑动窗口均值滤波）
class Filter:
    def __init__(self, window_size, max_inactive_time=2.0):
        self.window_size = window_size
        self.max_inactive_time = max_inactive_time
        self.data = {}  # 存储不同机器人的数据
        self.window = {}  # 存储滑动窗口内的数据
        self.last_update = {}  # 存储每个机器人的最后更新时间

    # 添加机器人坐标数据
    def add_data(self, name, x, y, threshold=100000.0):  # 阈值单位为mm，实测没啥用，不如直接给大点
        global guess_list
        if name not in self.data:
            # 如果实体名称不在数据字典中，初始化相应的deque。
            self.data[name] = deque(maxlen=self.window_size)
            self.window[name] = deque(maxlen=self.window_size)

        if len(self.window[name]) >= 2:
            # 计算当前坐标与前一个坐标的均方
            msd = sum((a - b) ** 2 for a, b in zip((x, y), self.window[name][-1])) / 2.0
            # print(name, msd)

            if msd > threshold:
                # 如果均方差超过阈值，可能是异常值，不将其添加到数据中
                return

        # 将坐标数据添加到数据字典和滑动窗口中。
        self.data[name].append((x, y))
        guess_list[name] = False

        self.window[name].append((x, y))
        self.last_update[name] = time.time()  # 更新最后更新时间

    # 过滤计算滑动窗口平均值
    def filter_data(self, name):
        if name not in self.data:
            return None

        if len(self.window[name]) < self.window_size:
            return None  # 不足以进行滤波

        # 计算滑动窗口内的坐标平均值
        x_avg = sum(coord[0] for coord in self.window[name]) / self.window_size
        y_avg = sum(coord[1] for coord in self.window[name]) / self.window_size

        return x_avg, y_avg

    # 获取所有机器人坐标
    def get_all_data(self):
        filtered_d = {}
        for name in self.data:
            # 超过max_inactive_time没识别到机器人将会清空缓冲区，并进行盲区预测
            if time.time() - self.last_update[name] > self.max_inactive_time:
                self.data[name].clear()
                self.window[name].clear()
                guess_list[name] = True
            # 识别到机器人，不进行盲区预测
            else:
                guess_list[name] = False
                filtered_d[name] = self.filter_data(name)
        # 返回所有当前识别到的机器人及其坐标的均值
        return filtered_d


# 海康相机图像获取线程
def hik_camera_get():
    # 获得设备信息
    global camera_image
    deviceList = MV_CC_DEVICE_INFO_LIST()
    tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE

    # ch:枚举设备 | en:Enum device
    # nTLayerType [IN] 枚举传输层 ，pstDevList [OUT] 设备列表
    while 1:
        ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
        if ret != 0:
            print("enum devices fail! ret[0x%x]" % ret)
            # sys.exit()

        if deviceList.nDeviceNum == 0:
            print("find no device!")
            # sys.exit()
        else:
            print("Find %d devices!" % deviceList.nDeviceNum)
            break

    for i in range(0, deviceList.nDeviceNum):
        mvcc_dev_info = cast(deviceList.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
        if mvcc_dev_info.nTLayerType == MV_GIGE_DEVICE:
            print("\ngige device: [%d]" % i)
            # 输出设备名字
            strModeName = ""
            for per in mvcc_dev_info.SpecialInfo.stGigEInfo.chModelName:
                strModeName = strModeName + chr(per)
            print("device model name: %s" % strModeName)
            # 输出设备ID
            nip1 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24)
            nip2 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16)
            nip3 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8)
            nip4 = (mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff)
            print("current ip: %d.%d.%d.%d\n" % (nip1, nip2, nip3, nip4))
        # 输出USB接口的信息
        elif mvcc_dev_info.nTLayerType == MV_USB_DEVICE:
            print("\nu3v device: [%d]" % i)
            strModeName = ""
            for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chModelName:
                if per == 0:
                    break
                strModeName = strModeName + chr(per)
            print("device model name: %s" % strModeName)

            strSerialNumber = ""
            for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chSerialNumber:
                if per == 0:
                    break
                strSerialNumber = strSerialNumber + chr(per)
            print("user serial number: %s" % strSerialNumber)
    # 手动选择设备
    # nConnectionNum = input("please input the number of the device to connect:")
    # 自动选择设备
    nConnectionNum = '0'
    if int(nConnectionNum) >= deviceList.nDeviceNum:
        print("intput error!")
        sys.exit()

    # ch:创建相机实例 | en:Creat Camera Object
    cam = MvCamera()

    # ch:选择设备并创建句柄 | en:Select device and create handle
    # cast(typ, val)，这个函数是为了检查val变量是typ类型的，但是这个cast函数不做检查，直接返回val
    stDeviceList = cast(deviceList.pDeviceInfo[int(nConnectionNum)], POINTER(MV_CC_DEVICE_INFO)).contents

    ret = cam.MV_CC_CreateHandle(stDeviceList)
    if ret != 0:
        print("create handle fail! ret[0x%x]" % ret)
        sys.exit()

    # ch:打开设备 | en:Open device
    ret = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
    if ret != 0:
        print("open device fail! ret[0x%x]" % ret)
        sys.exit()

    print(get_Value(cam, param_type="float_value", node_name="ExposureTime"),
          get_Value(cam, param_type="float_value", node_name="Gain"),
          get_Value(cam, param_type="enum_value", node_name="TriggerMode"),
          get_Value(cam, param_type="float_value", node_name="AcquisitionFrameRate"))

    # 设置设备的一些参数
    set_Value(cam, param_type="float_value", node_name="ExposureTime", node_value=16000)  # 曝光时间
    set_Value(cam, param_type="float_value", node_name="Gain", node_value=17.9)  # 增益值
    # 开启设备取流
    start_grab_and_get_data_size(cam)
    # 主动取流方式抓取图像
    stParam = MVCC_INTVALUE_EX()

    memset(byref(stParam), 0, sizeof(MVCC_INTVALUE_EX))
    ret = cam.MV_CC_GetIntValueEx("PayloadSize", stParam)
    if ret != 0:
        print("get payload size fail! ret[0x%x]" % ret)
        sys.exit()
    nDataSize = stParam.nCurValue
    pData = (c_ubyte * nDataSize)()
    stFrameInfo = MV_FRAME_OUT_INFO_EX()

    memset(byref(stFrameInfo), 0, sizeof(stFrameInfo))
    while True:
        ret = cam.MV_CC_GetOneFrameTimeout(pData, nDataSize, stFrameInfo, 1000)
        if ret == 0:
            image = np.asarray(pData)
            # 处理海康相机的图像格式为OPENCV处理的格式
            camera_image = image_control(data=image, stFrameInfo=stFrameInfo)
        else:
            print("no data[0x%x]" % ret)


def video_capture_get():
    global camera_image
    cam = cv2.VideoCapture(1)
    while True:
        ret, img = cam.read()
        if ret:
            camera_image = img
            time.sleep(0.016)  # 60fps


# 串口发送线程
def ser_send():
    seq = 0
    global chances_flag
    global guess_value
    # 单点预测时间
    guess_time = {
        'B1': 0,
        'B2': 0,
        'B7': 0,
        'R1': 0,
        'R2': 0,
        'R7': 0,
    }
    # 预测点索引
    guess_index = {
        'B1': 0,
        'B2': 0,
        'B7': 0,
        'R1': 0,
        'R2': 0,
        'R7': 0,
    }

    # 发送蓝方机器人坐标
    def send_point_B(send_name, all_filter_data):
        # front_time = time.time()
        # 转换为地图坐标系
        filtered_xyz = (2800 - all_filter_data[send_name][1], all_filter_data[send_name][0])
        # 转换为裁判系统单位M
        ser_x = int(filtered_xyz[0]) * 10 / 10
        ser_y = int(1500 - filtered_xyz[1]) * 10 / 10
        # 打包坐标数据包
        # data = build_data_radar(mapping_table.get(send_name), ser_x, ser_y)
        # packet, seq_s = build_send_packet(data, seq_s, [0x03, 0x05])
        # ser1.write(packet)
        # back_time = time.time()
        # # 计算发送延时，动态调整
        # waste_time = back_time - front_time
        # # print("发送：",send_name, seq_s)
        # time.sleep(0.1 - waste_time)
        return ser_x,ser_y
    # 发送红发机器人坐标
    def send_point_R(send_name, all_filter_data):
        # front_time = time.time()
        # 转换为地图坐标系
        filtered_xyz = (all_filter_data[send_name][1], 1500 - all_filter_data[send_name][0])
        # 转换为裁判系统单位M
        ser_x = int(filtered_xyz[0]) * 10 / 10
        ser_y = int(1500 - filtered_xyz[1]) * 10 / 10
        # 打包坐标数据包
        # data = build_data_radar(mapping_table.get(send_name), ser_x, ser_y)
        # packet, seq_s = build_send_packet(data, seq_s, [0x03, 0x05])
        # ser1.write(packet)
        # back_time = time.time()
        # # 计算发送延时，动态调整
        # waste_time = back_time - front_time
        # # print('发送：',send_name, seq_s)
        # time.sleep(0.1 - waste_time)
        return ser_x,ser_y

    # 发送盲区预测点坐标
    def send_point_guess(send_name, guess_time_limit):
        # front_time = time.time()
        # print(guess_value_now.get(send_name),guess_value.get(send_name) ,guess_index[send_name])
        # 进度未满 and 预测进度没有涨 and 超过单点预测时间上限，同时满足则切换另一个点预测
        if guess_value_now.get(send_name) < 120 and guess_value_now.get(send_name) - guess_value.get(
                send_name) <= 0 and time.time() - guess_time.get(send_name) >= guess_time_limit:
            guess_index[send_name] = 1 - guess_index[send_name]  # 每个ID不一样
            guess_time[send_name] = time.time()
        if guess_value_now.get(send_name) - guess_value.get(send_name) > 0:
            guess_time[send_name] = time.time()
        # 打包坐标数据包
        # data = build_data_radar(mapping_table.get(send_name), guess_table.get(send_name)[guess_index.get(send_name)][0],
        #                         guess_table.get(send_name)[guess_index.get(send_name)][1])
        # packet, seq_s = build_send_packet(data, seq_s, [0x03, 0x05])
        # ser1.write(packet)
        # back_time = time.time()
        # 计算发送延时，动态调整
        # waste_time = back_time - front_time
        # print('发送：',send_name, seq_s)
        # time.sleep(0.1 - waste_time)
        return guess_table.get(send_name)[guess_index.get(send_name)][0],guess_table.get(send_name)[guess_index.get(send_name)][1]

    time_s = time.time()
    target_last = 0  # 上一帧的飞镖目标
    update_time = 0  # 上次预测点更新时间
    send_count = 0  # 信道占用数，上限为4
    send_map = {
        "R1": (0, 0),
        "R2": (0, 0),
        "R3": (0, 0),
        "R4": (0, 0),
        "R5": (0, 0),
        "R6": (0, 0),
        "R7": (0, 0),
        "B1": (0, 0),
        "B2": (0, 0),
        "B3": (0, 0),
        "B4": (0, 0),
        "B5": (0, 0),
        "B6": (0, 0),
        "B7": (0, 0)
    }
    while True:

        guess_time_limit = 3 + 1.7  # 单位：秒，根据上一帧的信道占用数动态调整单点预测时间
        # print(guess_time_limit)
        send_count = 0  # 重置信道占用数
        try:
            all_filter_data = filter.get_all_data()
            if state == 'R':
                if not guess_list.get('B1'):
                    if all_filter_data.get('B1', False):
                        send_map['B1'] = send_point_B('B1', all_filter_data)
                else:
                    send_map['B1'] = (0, 0)

                if not guess_list.get('B2'):
                    if all_filter_data.get('B2', False):
                        send_map['B2'] = send_point_B('B2',  all_filter_data)
                else:
                    send_map['B2'] = (0, 0)

                # 步兵3号
                if not guess_list.get('B3'):
                    if all_filter_data.get('B3', False):
                        send_map['B3'] = send_point_B('B3',  all_filter_data)
                else:
                    send_map['B3'] = (0, 0)

                # 步兵4号
                if not guess_list.get('B4'):
                    if all_filter_data.get('B4', False):
                        send_map['B4'] = send_point_B('B4', all_filter_data)
                else:
                    send_map['B4'] = (0, 0)

                if not guess_list.get('B5'):
                    if all_filter_data.get('B5', False):
                        send_map['B5'] = send_point_B('B5',  all_filter_data)
                else:
                    send_map['B5'] = (0, 0)

                # 哨兵
                if guess_list.get('B7'):
                    send_map['B7'] = send_point_guess('B7', guess_time_limit)
                # 未识别到哨兵，进行盲区预测
                else:
                    if all_filter_data.get('B7', False):
                        send_map['B7'] = send_point_B('B7', all_filter_data)


            if state == 'B':
                if not guess_list.get('R1'):
                    if all_filter_data.get('R1', False):
                        send_map['R1'] = send_point_R('R1', all_filter_data)
                else:
                    send_map['R1'] = (0, 0)

                if not guess_list.get('R2'):
                    if all_filter_data.get('R2', False):
                        send_map['R2'] = send_point_R('R2', all_filter_data)
                else:
                    send_map['R2'] = (0, 0)

                # 步兵3号
                if not guess_list.get('R3'):
                    if all_filter_data.get('R3', False):
                        send_map['R3'] = send_point_R('R3', all_filter_data)
                else:
                    send_map['R3'] = (0, 0)

                # 步兵4号
                if not guess_list.get('R4'):
                    if all_filter_data.get('R4', False):
                        send_map['R4'] = send_point_R('R4', all_filter_data)
                else:
                    send_map['R4'] = (0, 0)

                if not guess_list.get('R5'):
                    if all_filter_data.get('R5', False):
                        send_map['R5'] = send_point_R('R5', all_filter_data)
                else:
                    send_map['R5'] = (0, 0)

                # 哨兵
                if guess_list.get('R7'):
                    send_map['R7'] = send_point_guess('R7', guess_time_limit)
                # 未识别到哨兵，进行盲区预测
                else:
                    if all_filter_data.get('R7', False):
                        send_map['R7'] = send_point_R('R7', all_filter_data)

            ser_data = build_data_radar_all(send_map,state)
            packet, seq = build_send_packet(ser_data, seq, [0x03, 0x05])
            ser1.write(packet)
            time.sleep(0.2)
            print(send_map,seq)
            # 超过单点预测时间上限，更新上次预测的进度
            if time.time() - update_time > guess_time_limit:
                update_time = time.time()
                if state == 'R':
                    guess_value['B1'] = guess_value_now.get('B1')
                    guess_value['B2'] = guess_value_now.get('B2')
                    guess_value['B7'] = guess_value_now.get('B7')
                else:
                    guess_value['R1'] = guess_value_now.get('R1')
                    guess_value['R2'] = guess_value_now.get('R2')
                    guess_value['R7'] = guess_value_now.get('R7')

            # 判断飞镖的目标是否切换，切换则尝试发动双倍易伤
            if target != target_last and target != 0:
                target_last = target
                # 有双倍易伤机会，并且当前没有在双倍易伤
                if double_vulnerability_chance > 0 and opponent_double_vulnerability == 0:
                    time_e = time.time()
                    # 发送时间间隔为10秒
                    if time_e - time_s > 10:
                        print("请求双倍触发")
                        data = build_data_decision(chances_flag, state)
                        packet, seq = build_send_packet(data, seq, [0x03, 0x01])
                        # print(packet.hex(),chances_flag,state)
                        ser1.write(packet)
                        print("请求成功", chances_flag)
                        # 更新标志位
                        chances_flag += 1
                        if chances_flag >= 3:
                            chances_flag = 1

                        time_s = time.time()
        except Exception as r:
            print('未知错误 %s' % (r))


# 裁判系统串口接收线程
def ser_receive():
    global progress_list  # 标记进度列表
    global double_vulnerability_chance  # 拥有双倍易伤次数
    global opponent_double_vulnerability  # 双倍易伤触发状态
    global target  # 飞镖当前目标
    progress_cmd_id = [0x02, 0x0C]  # 任意想要接收数据的命令码，这里是雷达标记进度的命令码0x020E
    vulnerability_cmd_id = [0x02, 0x0E]  # 双倍易伤次数和触发状态
    target_cmd_id = [0x01, 0x05]  # 飞镖目标
    buffer = b''  # 初始化缓冲区
    while True:
        # 从串口读取数据
        received_data = ser1.read_all()  # 读取一秒内收到的所有串口数据
        # 将读取到的数据添加到缓冲区中
        buffer += received_data

        # 查找帧头（SOF）的位置
        sof_index = buffer.find(b'\xA5')

        while sof_index != -1:
            # 如果找到帧头，尝试解析数据包
            if len(buffer) >= sof_index + 5:  # 至少需要5字节才能解析帧头
                # 从帧头开始解析数据包
                packet_data = buffer[sof_index:]

                # 查找下一个帧头的位置
                next_sof_index = packet_data.find(b'\xA5', 1)

                if next_sof_index != -1:
                    # 如果找到下一个帧头，说明当前帧头到下一个帧头之间是一个完整的数据包
                    packet_data = packet_data[:next_sof_index]
                    # print(packet_data)
                else:
                    # 如果没找到下一个帧头，说明当前帧头到末尾不是一个完整的数据包
                    break

                # 解析数据包
                progress_result = receive_packet(packet_data, progress_cmd_id,
                                                 info=False)  # 解析单个数据包，cmd_id为0x020E,不输出日志
                vulnerability_result = receive_packet(packet_data, vulnerability_cmd_id, info=False)
                target_result = receive_packet(packet_data, target_cmd_id, info=False)
                # 更新裁判系统数据，标记进度、易伤、飞镖目标
                if progress_result is not None:
                    received_cmd_id1, received_data1, received_seq1 = progress_result
                    progress_list = list(received_data1)
                    if state == 'R':
                        guess_value_now['B1'] = progress_list[0]
                        guess_value_now['B2'] = progress_list[1]
                        guess_value_now['B7'] = progress_list[5]
                    else:
                        guess_value_now['R1'] = progress_list[0]
                        guess_value_now['R2'] = progress_list[1]
                        guess_value_now['R7'] = progress_list[5]
                if vulnerability_result is not None:
                    received_cmd_id2, received_data2, received_seq2 = vulnerability_result
                    received_data2 = list(received_data2)[0]
                    double_vulnerability_chance, opponent_double_vulnerability = Radar_decision(received_data2)
                if target_result is not None:
                    received_cmd_id3, received_data3, received_seq3 = target_result
                    target = (list(received_data3)[1] & 0b1100000) >> 5

                # 从缓冲区中移除已解析的数据包
                buffer = buffer[sof_index + len(packet_data):]

                # 继续寻找下一个帧头的位置
                sof_index = buffer.find(b'\xA5')

            else:
                # 缓冲区中的数据不足以解析帧头，继续读取串口数据
                break
        time.sleep(0.5)


# 创建机器人坐标滤波器
filter = Filter(window_size=3, max_inactive_time=2)

# 加载模型，实例化机器人检测器和装甲板检测器
weights_path = 'models/car.onnx'  # 建议把模型转换成TRT的engine模型，推理速度提升10倍，转换方式看README
weights_path_next = 'models/armor.onnx'
# weights_path = 'models/car.engine'
# weights_path_next = 'models/armor.engine'
detector = YOLOv5Detector(weights_path, data='yaml/car.yaml', conf_thres=0.1, iou_thres=0.5, max_det=14, ui=True)
detector_next = YOLOv5Detector(weights_path_next, data='yaml/armor.yaml', conf_thres=0.50, iou_thres=0.2,
                               max_det=1,
                               ui=True)


# 图像测试模式（获取图像根据自己的设备，在）
camera_mode = 'test'  # 'test':测试模式,'hik':海康相机,'video':USB相机（videocapture）
ser1 = serial.Serial('COM19', 115200, timeout=1)  # 串口，替换 'COM1' 为你的串口号
# 串口接收线程
thread_receive = threading.Thread(target=ser_receive, daemon=True)
thread_receive.start()

# 串口发送线程
thread_list = threading.Thread(target=ser_send, daemon=True)
thread_list.start()

camera_image = None

if camera_mode == 'test':
    camera_image = cv2.imread('images/test_image.jpg')
elif camera_mode == 'hik':
    # 海康相机图像获取线程
    from hik_camera import call_back_get_image, start_grab_and_get_data_size, close_and_destroy_device, set_Value, get_Value,image_control
    if sys.platform.startswith("win"):
        from MvImport.MvCameraControl_class import *
    else:
        from MvImport_Linux.MvCameraControl_class import *
    thread_camera = threading.Thread(target=hik_camera_get, daemon=True)
    thread_camera.start()
elif camera_mode == 'video':
    # USB相机图像获取线程
    thread_camera = threading.Thread(target=video_capture_get, daemon=True)
    thread_camera.start()

while camera_image is None:
    print("等待图像。。。")
    time.sleep(0.5)

# 获取相机图像的画幅，限制点不超限
img0 = camera_image.copy()
img_y = img0.shape[0]
img_x = img0.shape[1]
print(img0.shape)

while True:
    # 刷新裁判系统信息UI图像
    information_ui_show = information_ui.copy()
    map = map_backup.copy()
    det_time = 0
    img0 = camera_image.copy()
    ts = time.time()
    # 第一层神经网络识别
    result0 = detector.predict(img0)
    det_time += 1
    for detection in result0:
        cls, xywh, conf = detection
        if cls == 'car':
            left, top, w, h = xywh
            left, top, w, h = int(left), int(top), int(w), int(h)
            # 存储第一次检测结果和区域
            # ROI出机器人区域
            cropped = camera_image[top:top + h, left:left + w]
            cropped_img = np.ascontiguousarray(cropped)
            # 第二层神经网络识别
            result_n = detector_next.predict(cropped_img)
            det_time += 1
            if result_n:
                # 叠加第二次检测结果到原图的对应位置
                img0[top:top + h, left:left + w] = cropped_img

                for detection1 in result_n:
                    cls, xywh, conf = detection1
                    if cls:  # 所有装甲板都处理，可选择屏蔽一些:
                        x, y, w, h = xywh
                        x = x + left
                        y = y + top

                        t1 = time.time()
                        # 原图中装甲板的中心下沿作为待仿射变化的点
                        camera_point = np.array([[[min(x + 0.5 * w, img_x), min(y + 1.5 * h, img_y)]]],
                                                dtype=np.float32)
                        # 低到高依次仿射变化
                        # 先套用地面层仿射变化矩阵
                        mapped_point = cv2.perspectiveTransform(camera_point.reshape(1, 1, 2), M_ground)
                        # 限制转换后的点在地图范围内
                        x_c = max(int(mapped_point[0][0][0]), 0)
                        y_c = max(int(mapped_point[0][0][1]), 0)
                        x_c = min(x_c, width)
                        y_c = min(y_c, height)
                        color = mask_image[y_c, x_c]  # 通过掩码图像，获取地面层的颜色：黑（0，0，0）
                        if color[0] == color[1] == color[2] == 0:
                            X_M = x_c
                            Y_M = y_c
                            # Z_M = 0
                            filter.add_data(cls, X_M, Y_M)
                        else:
                            # 不满足则继续套用R型高地层仿射变换矩阵
                            mapped_point = cv2.perspectiveTransform(camera_point.reshape(1, 1, 2), M_height_r)
                            # 限制转换后的点在地图范围内
                            x_c = max(int(mapped_point[0][0][0]), 0)
                            y_c = max(int(mapped_point[0][0][1]), 0)
                            x_c = min(x_c, width)
                            y_c = min(y_c, height)
                            color = mask_image[y_c, x_c]  # 通过掩码图像，获取R型高地层的颜色：绿（0，255，0）
                            if color[1] > color[2] and color[1] > color[0]:
                                X_M = x_c
                                Y_M = y_c
                                # Z_M = 400
                                filter.add_data(cls, X_M, Y_M)
                            else:
                                # 不满足则继续套用环形高地层仿射变换矩阵
                                mapped_point = cv2.perspectiveTransform(camera_point.reshape(1, 1, 2), M_height_g)
                                # 限制转换后的点在地图范围内
                                x_c = max(int(mapped_point[0][0][0]), 0)
                                y_c = max(int(mapped_point[0][0][1]), 0)
                                x_c = min(x_c, width)
                                y_c = min(y_c, height)
                                color = mask_image[y_c, x_c]  # 通过掩码图像，获取环型高地层的颜色：蓝（255，0，0）
                                if color[0] > color[2] and color[0] > color[1]:
                                    X_M = x_c
                                    Y_M = y_c
                                    # Z_M = 600
                                    filter.add_data(cls, X_M, Y_M)

    # 获取所有识别到的机器人坐标
    all_filter_data = filter.get_all_data()
    # print(all_filter_data_name)
    if all_filter_data != {}:
        for name, xyxy in all_filter_data.items():
            if xyxy is not None:
                if name[0] == "R":
                    color_m = (0, 0, 255)
                else:
                    color_m = (255, 0, 0)
                if state == 'R':
                    filtered_xyz = (2800 - xyxy[1], xyxy[0])  # 缩放坐标到地图图像
                else:
                    filtered_xyz = (xyxy[1], 1500 - xyxy[0])  # 缩放坐标到地图图像
                # 只绘制敌方阵营的机器人（这里不会绘制盲区预测的机器人）
                if name[0] != state:
                    cv2.circle(map, (int(filtered_xyz[0]), int(filtered_xyz[1])), 15, color_m, -1)  # 绘制圆
                    cv2.putText(map, str(name),
                                (int(filtered_xyz[0]) - 5, int(filtered_xyz[1]) + 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 2.5, (255, 255, 255), 5)
                    ser_x = int(filtered_xyz[0]) * 10 / 10
                    ser_y = int(1500 - filtered_xyz[1]) * 10 / 10
                    cv2.putText(map, "(" + str(ser_x) + "," + str(ser_y) + ")",
                                (int(filtered_xyz[0]) - 100, int(filtered_xyz[1]) + 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 4)

    te = time.time()
    t_p = te - ts
    print("fps:", 1 / t_p)  # 打印帧率
    # 绘制UI
    _ = draw_information_ui(progress_list, state, information_ui_show)
    cv2.putText(information_ui_show, "vulnerability_chances: " + str(double_vulnerability_chance),
                (10, 350),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(information_ui_show, "vulnerability_Triggering: " + str(opponent_double_vulnerability),
                (10, 400),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.imshow('information_ui', information_ui_show)
    map_show = cv2.resize(map, (600, 320))
    cv2.imshow('map', map_show)
    img0 = cv2.resize(img0, (1300, 900))
    cv2.imshow('img', img0)

    key = cv2.waitKey(1)
