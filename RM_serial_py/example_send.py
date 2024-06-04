import time

import serial
from ser_api import build_data_radar, build_send_packet, build_data_decision

# 发送示例
ser = serial.Serial('COM19', 115200, timeout=1)  # 根据实际情况修改串口号和波特率
seq = 0
while True:
    # 构建发送数据部分，需要根据不同命令码的内容，在ser_api中自行编写，这里是雷达发送坐标的示例
    data  = build_data_decision(1, 'R')  # 假设蓝方一号英雄坐标（20.11，10.22）
    # data = b'\x02\x00\x00\x00\x01\x00'
    packet, seq = build_send_packet(data, seq, [0x03, 0x01])  # 根据data构建完整待发送的数据包
    # data = build_data_decision(chances, state)
    # packet, seq = build_send_packet(data, seq, [0x03, 0x01])
    # print(packet.hex(),chances,state)
    ser.write(packet)
    print(packet)
    ser.write(packet)
    time.sleep(0.1)
