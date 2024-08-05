import struct

# CRC8校验表

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
CRC8_INIT = 0xff
CRC8_TAB = [
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc, 0x23,
    0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62, 0xbe, 0xe0,
    0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff, 0x46, 0x18, 0xfa,
    0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07, 0xdb, 0x85, 0x67,
    0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a, 0x65, 0x3b, 0xd9, 0x87,
    0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24, 0xf8, 0xa6, 0x44, 0x1a, 0x99,
    0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd, 0x11,
    0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50, 0xaf, 0xf1,
    0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee, 0x32, 0x6c, 0x8e,
    0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73, 0xca, 0x94, 0x76, 0x28,
    0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b, 0x57, 0x09, 0xeb, 0xb5,
    0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16, 0xe9, 0xb7, 0x55, 0x0b, 0x88,
    0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
]

# CRC16校验表
CRC_INIT = 0xffff
wCRC_Table = [
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
]


# CRC8校验
def Get_CRC8_Check_Sum(pchMessage, dwLength):
    ucCRC8 = CRC8_INIT
    for ch in pchMessage[:dwLength]:
        ucIndex = ucCRC8 ^ ch
        ucCRC8 = CRC8_TAB[ucIndex]
    return ucCRC8


# CRC16校验
def Get_CRC16_Check_Sum(pchMessage, dwLength):
    wCRC = CRC_INIT
    for ch in pchMessage[:dwLength]:
        wCRC = ((wCRC >> 8) & 0xFF) ^ wCRC_Table[(wCRC ^ ch) & 0xFF]
    return wCRC


# 雷达数据部分构建示例
def build_data_radar(target_robot_id, target_position_x, target_position_y):
    data = bytearray()
    data.extend(bytearray(struct.pack('H', target_robot_id)))  # 目标机器人ID (小端)
    data.extend(bytearray(struct.pack('f', target_position_x)))  # x坐标 (小端)
    data.extend(bytearray(struct.pack('f', target_position_y)))  # y坐标 (小端)
    return data


# 雷达数据部分构建示例
def build_data_radar_all(send_map,state):
    if state == 'R':
        data = bytearray()
        data.extend(bytearray(struct.pack('H', int(send_map['B1'][0]))))  # x坐标 (小端)
        data.extend(bytearray(struct.pack('H', int(send_map['B1'][1]))))  # y坐标 (小端)

        data.extend(bytearray(struct.pack('H', int(send_map['B2'][0]))))  # x坐标 (小端)
        data.extend(bytearray(struct.pack('H', int(send_map['B2'][1]))))  # y坐标 (小端)

        data.extend(bytearray(struct.pack('H', int(send_map['B3'][0]))))  # x坐标 (小端)
        data.extend(bytearray(struct.pack('H', int(send_map['B3'][1]))))  # y坐标 (小端)

        data.extend(bytearray(struct.pack('H', int(send_map['B4'][0]))))  # x坐标 (小端)
        data.extend(bytearray(struct.pack('H', int(send_map['B4'][1]))))  # y坐标 (小端)

        data.extend(bytearray(struct.pack('H', int(send_map['B5'][0]))))  # x坐标 (小端)
        data.extend(bytearray(struct.pack('H', int(send_map['B5'][1]))))  # y坐标 (小端)

        data.extend(bytearray(struct.pack('H', int(send_map['B7'][0]))))  # x坐标 (小端)
        data.extend(bytearray(struct.pack('H', int(send_map['B7'][1]))))  # y坐标 (小端)
    else:
        data = bytearray()
        data.extend(bytearray(struct.pack('H', int(send_map['R1'][0]))))  # x坐标 (小端)
        data.extend(bytearray(struct.pack('H', int(send_map['R1'][1]))))  # y坐标 (小端)

        data.extend(bytearray(struct.pack('H', int(send_map['R2'][0]))))  # x坐标 (小端)
        data.extend(bytearray(struct.pack('H', int(send_map['R2'][1]))))  # y坐标 (小端)

        data.extend(bytearray(struct.pack('H', int(send_map['R3'][0]))))  # x坐标 (小端)
        data.extend(bytearray(struct.pack('H', int(send_map['R3'][1]))))  # y坐标 (小端)

        data.extend(bytearray(struct.pack('H', int(send_map['R4'][0]))))  # x坐标 (小端)
        data.extend(bytearray(struct.pack('H', int(send_map['R4'][1]))))  # y坐标 (小端)

        data.extend(bytearray(struct.pack('H', int(send_map['R5'][0]))))  # x坐标 (小端)
        data.extend(bytearray(struct.pack('H', int(send_map['R5'][1]))))  # y坐标 (小端)

        data.extend(bytearray(struct.pack('H', int(send_map['R7'][0]))))  # x坐标 (小端)
        data.extend(bytearray(struct.pack('H', int(send_map['R7'][1]))))  # y坐标 (小端)

    return data



def build_data_decision(chances,state):
    data = bytearray()
    cmd_id = [0x01, 0x21]
    cmd_id = bytearray([cmd_id[1], cmd_id[0]])
    data.extend(cmd_id)
    if state == 'R':
        data.extend(bytearray(struct.pack('H', 9)))
    else:
        data.extend(bytearray(struct.pack('H', 109)))
    data.extend(bytearray([0x80,0x80]))
    data.extend(bytearray(struct.pack('B', chances)))
    return data



# 完整数据包构建
def build_send_packet(data, seq, cmd_id):
    data_length = len(data)  # 数据部分长度
    frame_header = bytearray([0xA5])  # SOF
    cmd_id = bytearray([cmd_id[1], cmd_id[0]])
    frame_header.extend(struct.pack('H', data_length))  # 编码数据长度 (小端)
    frame_header.append(seq)  # seq
    crc8 = Get_CRC8_Check_Sum(frame_header, 4)
    frame_header.append(crc8)  # CRC8校验码
    frame_tail = bytearray()
    # 帧尾CRC16校验
    frame_tail.extend(struct.pack('H', Get_CRC16_Check_Sum(frame_header + cmd_id + data,
                                                           len(frame_header + cmd_id + data) + 1)))
    packet = frame_header + cmd_id + data + frame_tail
    return packet, (seq + 1) % 256


# 单个数据包的特定命令码解析
def receive_packet(serial_data, expected_cmd_id, info):
    # 定义常量

    FRAME_HEADER_LEN = 5  # 帧头长度（SOF + 数据长度 + 序列号 + CRC8校验码）
    FRAME_TAIL_LEN = 2  # 帧尾长度（CRC16校验码）

    # 检查串口数据是否足够包含一个完整的数据包

    # 查找帧头（SOF）
    sof_index = serial_data.find(b'\xA5')
    if sof_index == -1:
        if info:
            print('未找到SOF')
        return None  # 未找到SOF

    # 提取帧头
    header = serial_data[sof_index:sof_index + FRAME_HEADER_LEN]

    # 提取数据长度和序列号
    data_length_bytes = header[1:3]
    data_length = int.from_bytes(data_length_bytes, byteorder='little')
    if len(serial_data) < FRAME_HEADER_LEN + data_length + FRAME_TAIL_LEN:
        if info:
            print("数据不足")
        return None  # 数据不足

    seq = header[3]

    # 计算CRC8并与接收到的CRC8进行比较
    crc8 = Get_CRC8_Check_Sum(header[:-1], FRAME_HEADER_LEN - 1)
    if crc8 != header[-1]:
        if info:
            print('CRC8校验失败')
        return None  # CRC8校验失败

    # 提取命令ID
    cmd_id_bytes = serial_data[sof_index + FRAME_HEADER_LEN:sof_index + FRAME_HEADER_LEN + 2]
    expected_cmd_id = bytes([expected_cmd_id[1], expected_cmd_id[0]])
    # 检查接收到的命令ID是否与期望的命令ID匹配
    if cmd_id_bytes != expected_cmd_id:
        if info:
            print('命令ID不匹配')
        return None  # 命令ID不匹配

    # 提取数据部分
    data_start_index = sof_index + FRAME_HEADER_LEN + 2  # 数据起始位置
    data_end_index = data_start_index + data_length
    data = serial_data[data_start_index:data_end_index]

    # 提取帧尾（CRC16）
    frame_tail_start = data_end_index
    frame_tail_end = frame_tail_start + FRAME_TAIL_LEN
    frame_tail_bytes = serial_data[frame_tail_start:frame_tail_end]

    # 计算CRC16并与接收到的CRC16进行比较
    calculated_crc16 = Get_CRC16_Check_Sum(header + cmd_id_bytes + data,
                                           FRAME_HEADER_LEN + 2 + data_length + FRAME_TAIL_LEN)
    received_crc16 = int.from_bytes(frame_tail_bytes, byteorder='little')
    if calculated_crc16 != received_crc16:
        if info:
            print('CRC16校验失败')
        return None  # CRC16校验失败

    return cmd_id_bytes, data, seq


def Radar_decision(byte_data):
    # 确保输入是一个字节
    if not isinstance(byte_data, int) or byte_data < 0 or byte_data > 255:
        raise ValueError("输入必须是一个字节（0-255）")

    # 提取第 0-1 位的双倍易伤机会
    double_vulnerability_chance = byte_data & 0b00000011

    # 提取第 2 位的对方双倍易伤状态
    opponent_double_vulnerability = (byte_data & 0b00000100) >> 2

    # 提取第 3-7 位（保留位，不使用）
    reserved_bits = (byte_data & 0b11111000) >> 3

    # 打印结果

    # print(f"双倍易伤机会: {double_vulnerability_chance}")
    # print(f"对方正在被触发双倍易伤: {opponent_double_vulnerability}")
    # print(f"保留位: {reserved_bits}")
    return double_vulnerability_chance, opponent_double_vulnerability