import time

import cv2
import numpy as np

index_table = {
    1: "R1",
    2: "R2",
    3: "R3",
    4: "R4",
    5: "R5",
    6: "R7",
    101: "B1",
    102: "B2",
    103: "B3",
    104: "B4",
    105: "B5",
    106: "B7"
}


# 绘制裁判系统数据的UI
def draw_information_ui(bar_list, camp, image):
    cv2.line(image, (300, 0), (300, 300), (0, 150, 0), 2)
    height_light = [0, 0, 0, 0, 0, 0]

    # 计算每条线段的长度
    num_lines = len(bar_list)
    max_value = 120
    threshold = 100  # 临界值
    max_length = 300  # 最大长度
    segment_height = int(300 / num_lines)

    # 绘制线段和索引
    for i, value in enumerate(bar_list):

        # 计算线段长度
        if value > threshold:
            line_length = int((value / max_value) * max_length)
            line_height = 8
            height_light[i] = 1
            if camp == 'R':
                color = (255, 0, 0)  # 超过临界值的线段高光处理为绿色
            else:
                color = (0, 0, 255)
        else:
            line_height = 3
            if camp == 'R':
                color = (200, 0, 0)  # 超过临界值的线段高光处理为绿色
            else:
                color = (0, 0, 200)
            line_length = int((value / max_value) * max_length)

        # 绘制线段
        start_point = (50, i * segment_height + segment_height // 2)
        end_point = (50 + line_length, i * segment_height + segment_height // 2)
        cv2.line(image, start_point, end_point, color, line_height, lineType=cv2.LINE_AA)

        # 绘制索引
        if camp == 'R':
            index = i + 101
        else:
            index = i + 1
        cv2.putText(image, str(index_table.get(index)), (10, start_point[1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(image, str(value), (370, start_point[1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2,
                    cv2.LINE_AA)

    # 在值为100的位置处画一条垂直线

    return height_light

# 测试代码
# ts = time.time()
# bar_list = [80, 100, 120, 90, 110, 1]  # 示例数据
# height_l = draw_lines(bar_list, 'B')
# te = time.time()
# print(height_l,te-ts)
# # 显示图像
# cv2.imshow('Lines', image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
