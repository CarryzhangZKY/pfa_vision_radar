import time
import serial
from ser_api import receive_packet

ser = serial.Serial('COM19', 115200, timeout=1)  # 裁判系统规定的频率
expected_cmd_id1 = [0x03, 0x03]  # 任意想要接收数据的命令码，这里是雷达标记进度的命令码0x020E
buffer = b''  # 初始化缓冲区

# 接收示例
while True:
    # 从串口读取数据
    received_data = ser.read_all()  # 读取一秒内收到的所有串口数据
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
            result1 = receive_packet(packet_data, expected_cmd_id1, info=True)  # 解析单个数据包，cmd_id为0x020E,不输出日志
            if result1 is not None:
                received_cmd_id1, received_data1, received_seq1 = result1

                # received_data1[0]=bytes(0x30)
                # received_data1 = b'\x10\x21\x00\x00\x00'
                # received_data1 = list(received_data1)

                # print("成功接收到命令ID为{}的数据包，数据为{}，序列号为{}".format(received_cmd_id1, received_data1, received_seq1))

                # result = (list(received_data1)[1] & 0b1100000) >> 5
                hex_representation = ' '.join(format(byte, '02X') for byte in received_data1)
                print(hex_representation)
                # for received_datas in received_data1:
                #     print(received_datas)
            # 从缓冲区中移除已解析的数据包
            buffer = buffer[sof_index + len(packet_data):]

            # 继续寻找下一个帧头的位置
            sof_index = buffer.find(b'\xA5')

        else:
            # 缓冲区中的数据不足以解析帧头，继续读取串口数据
            break

    time.sleep(1)  # 解析频率为1hz



