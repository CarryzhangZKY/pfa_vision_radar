import threading
import time

import cv2
import numpy as np
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPixmap, QImage, QTextCursor
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QTextEdit, QGridLayout

from hik_camera import call_back_get_image, start_grab_and_get_data_size, close_and_destroy_device, set_Value, \
    get_Value, image_control

from MvImport.MvCameraControl_class import *


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


color = [(255, 255, 255), (0, 255, 0), (0, 0, 255)]


class MyUI(QWidget):
    def __init__(self):
        super().__init__()
        self.capturing = True
        self.initUI()

    def initUI(self):
        # 左上角部分
        self.state = state
        self.left_top_label = QLabel(self)
        self.left_top_label.setFixedSize(1350, 1000)
        self.left_top_label.setStyleSheet("border: 2px solid black;")
        self.left_top_label.mousePressEvent = self.left_top_clicked
        self.image_points = [[(0, 0), (0, 0), (0, 0), (0, 0)], [(0, 0), (0, 0), (0, 0), (0, 0)],
                             [(0, 0), (0, 0), (0, 0), (0, 0)], [(0, 0), (0, 0), (0, 0), (0, 0)]]
        self.map_points = [[(0, 0), (0, 0), (0, 0), (0, 0)], [(0, 0), (0, 0), (0, 0), (0, 0)],
                           [(0, 0), (0, 0), (0, 0), (0, 0)], [(0, 0), (0, 0), (0, 0), (0, 0)]]
        self.image_count = 0
        self.map_count = 0
        # 右上角部分
        self.right_top_label = QLabel(self)
        self.right_top_label.setFixedSize(550, 900)
        self.right_top_label.setStyleSheet("border: 2px solid black;")
        self.right_top_label.mousePressEvent = self.right_top_clicked

        # 左下角部分
        self.left_bottom_text = QTextEdit(self)
        self.left_bottom_text.setFixedSize(300, 60)

        # 右下角部分
        self.button1 = QPushButton('开始标定', self)
        self.button1.setFixedSize(100, 30)
        self.button1.clicked.connect(self.button1_clicked)

        self.button2 = QPushButton('切换高度', self)
        self.button2.setFixedSize(100, 30)
        self.button2.clicked.connect(self.button2_clicked)

        self.button3 = QPushButton('加载坐标', self)
        self.button3.setFixedSize(100, 30)
        self.button3.clicked.connect(self.button3_clicked)

        self.button4 = QPushButton('保存计算', self)
        self.button4.setFixedSize(100, 30)
        self.button4.clicked.connect(self.button4_clicked)
        self.height = 0
        # 加载预先选择好的图片并缩放到940x600
        if self.state == 'R':
            self.save_path = 'arrays_test_red.npy'
            right_image_path = "images/map_red.jpg"  # 替换为右边图片的路径
        else:
            self.save_path = 'arrays_test_blue.npy'
            right_image_path = "images/map_blue.jpg"  # 替换为右边图片的路径

        # _,left_image = self.camera_capture.read()
        left_image = camera_image
        right_image = cv2.imread(right_image_path)

        # 记录缩放比例
        self.left_scale_x = left_image.shape[1] / 1350.0
        self.left_scale_y = left_image.shape[0] / 1000.0

        self.right_scale_x = right_image.shape[1] / 550.0
        self.right_scale_y = right_image.shape[0] / 900.0
        left_image = cv2.cvtColor(left_image, cv2.COLOR_BGR2RGB)
        self.left_image = cv2.resize(left_image, (1350, 1000))
        right_image = cv2.cvtColor(right_image, cv2.COLOR_BGR2RGB)
        self.right_image = cv2.resize(right_image, (550, 900))
        # 缩放图像
        self.update_images()

        self.camera_timer = QTimer(self)
        self.camera_timer.timeout.connect(self.update_camera)
        self.camera_timer.start(50)  # 50毫秒更新一次相机
        # 设置按钮样式
        self.set_button_style(self.button1)
        self.set_button_style(self.button2)
        self.set_button_style(self.button3)
        self.set_button_style(self.button4)

        # Set up the grid layout for buttons
        grid_layout = QGridLayout()
        grid_layout.addWidget(self.button1, 0, 0)
        grid_layout.addWidget(self.button2, 0, 1)
        grid_layout.addWidget(self.button3, 1, 0)
        grid_layout.addWidget(self.button4, 1, 1)

        # Create a new widget to contain buttons and text
        buttons_and_text_widget = QWidget()

        # Set up horizontal layout for the new widget
        hbox_buttons_and_text = QHBoxLayout(buttons_and_text_widget)
        hbox_buttons_and_text.addLayout(grid_layout)
        hbox_buttons_and_text.addWidget(self.left_bottom_text)

        # Set up vertical layouts for left and right sides
        vbox_left = QVBoxLayout()
        vbox_left.addWidget(self.left_top_label)

        vbox_right = QVBoxLayout()
        vbox_right.addWidget(self.right_top_label)
        vbox_right.addWidget(buttons_and_text_widget)  # Add the new widget to the right side

        # Set up horizontal layout for the whole window
        hbox = QHBoxLayout()
        # hbox.setSpacing(1)
        hbox.addLayout(vbox_left)
        hbox.addLayout(vbox_right)

        # Set the main layout of the window and other properties
        self.setLayout(hbox)
        self.setGeometry(0, 0, 1900, 1000)
        self.setWindowTitle('PyQt UI 示例')
        self.setWindowFlags(Qt.FramelessWindowHint)
        self.show()

    def keyPressEvent(self, event):
        # 按下键盘事件
        if event.key() == Qt.Key_Escape:
            self.close()

    def update_images(self):
        # Load and update the left image

        left_pixmap = self.convert_cvimage_to_pixmap(self.left_image)
        self.left_top_label.setPixmap(left_pixmap)

        # Load and update the right image

        right_pixmap = self.convert_cvimage_to_pixmap(self.right_image)
        self.right_top_label.setPixmap(right_pixmap)

    def update_camera(self):
        if self.capturing:
            img0 = camera_image
            left_image = cv2.cvtColor(img0, cv2.COLOR_BGR2RGB)
            self.left_image = cv2.resize(left_image, (1350, 1000))
            self.update_images()

    def left_top_clicked(self, event):
        # 图像点击事件
        if not self.capturing:
            x = int(event.pos().x() * self.left_scale_x)
            y = int(event.pos().y() * self.left_scale_y)

            self.image_points[self.height][self.image_count % 4] = (x, y)

            cv2.circle(self.left_image, (int(x / self.left_scale_x), int(y / self.left_scale_y)), 4, color[self.height],
                       -1)
            cv2.putText(self.left_image, str(self.image_count % 4),
                        (int(x / self.left_scale_x), int(y / self.left_scale_y)), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        color[self.height], 3)
            self.image_count += 1
            self.update_images()
            self.append_text(f'图像真实点击坐标：({x}, {y})')

    def right_top_clicked(self, event):
        # 地图点击事件
        if not self.capturing:
            x = int(event.pos().x() * self.right_scale_x)
            y = int(event.pos().y() * self.right_scale_y)
            self.map_points[self.height][self.map_count % 4] = (x, y)

            cv2.circle(self.right_image, (int(x / self.right_scale_x), int(y / self.right_scale_y)), 4,
                       color[self.height],
                       -1)
            cv2.putText(self.right_image, str(self.map_count % 4),
                        (int(x / self.right_scale_x), int(y / self.right_scale_y)), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        color[self.height], 2)
            self.map_count += 1
            self.update_images()
            self.append_text(f'地图真实点击坐标：({x}, {y})')

    def button1_clicked(self):
        # 按钮1点击事件
        self.append_text('开始标定')
        self.capturing = False

        print('开始标定')

    def button2_clicked(self):
        # 按钮2点击事件
        self.append_text('切换高度')
        self.image_count = 0
        self.map_count = 0
        self.height = (self.height + 1) % 3
        print('切换高度')

    def button3_clicked(self):
        # 按钮3点击事件
        self.append_text('加载坐标')  # 该功能还未制作

        print('加载坐标')

    def button4_clicked(self):
        # 按钮4点击事件
        print(self.image_points)
        print(self.map_points)
        for i in range(0, 3):
            image_point = np.array(self.image_points[i], dtype=np.float32)
            map_point = np.array(self.map_points[i], dtype=np.float32)
            self.T.append(cv2.getPerspectiveTransform(image_point, map_point))

        np.save(self.save_path, self.T)

        self.append_text('保存计算')
        print('保存计算', self.save_path)
        time.sleep(1)
        sys.exit()

    def convert_cvimage_to_pixmap(self, cvimage):
        height, width, channel = cvimage.shape
        bytes_per_line = 3 * width
        qimage = QImage(cvimage.data, width, height, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimage)
        return pixmap

    def set_button_style(self, button):
        button.setStyleSheet("QPushButton { font-size: 18px; }")

    def append_text(self, text):
        # 在文本组件中追加文本
        current_text = self.left_bottom_text.toPlainText()
        self.left_bottom_text.setPlainText(current_text + '\n' + text)
        # 自动向下滚动文本组件
        cursor = self.left_bottom_text.textCursor()
        cursor.movePosition(QTextCursor.End)
        self.left_bottom_text.setTextCursor(cursor)


if __name__ == '__main__':
    camera_mode = 'test'  # 'test':测试模式,'hik':海康相机,'video':USB相机（videocapture）
    camera_image = None
    state = 'B'  # R:红方/B:蓝方

    if camera_mode == 'test':
        camera_image = cv2.imread('images/test_image.jpg')
    elif camera_mode == 'hik':
        # 海康相机图像获取线程
        thread_camera = threading.Thread(target=hik_camera_get, daemon=True)
        thread_camera.start()
    elif camera_mode == 'video':
        # USB相机图像获取线程
        thread_camera = threading.Thread(target=video_capture_get, daemon=True)
        thread_camera.start()

    while camera_image is None:
        print("等待图像。。。")
        time.sleep(0.5)
    app = QApplication(sys.argv)
    myui = MyUI()
    sys.exit(app.exec_())
