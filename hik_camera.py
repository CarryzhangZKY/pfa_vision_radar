import ctypes
import os
import sys
import numpy as np
from os import getcwd
import cv2
# import msvcrt
from ctypes import *


if sys.platform.startswith("win"):    
    sys.path.append("./MvImport")
    from MvImport.MvCameraControl_class import *
else:
    sys.path.append("./MvImport_Linux")
    from MvImport_Linux.MvCameraControl_class import *

global img_test


# 枚举设备
def image_control(data, stFrameInfo):
    image = None
    if stFrameInfo.enPixelType == 17301505:
        image = data.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth))
        # image_show(image=image, name=stFrameInfo.nHeight)
    elif stFrameInfo.enPixelType == 17301513:
        data = data.reshape(stFrameInfo.nHeight, stFrameInfo.nWidth, -1)
        image = cv2.cvtColor(data, cv2.COLOR_BAYER_RG2RGB)
        # image_show(image=image, name=stFrameInfo.nHeight)
    elif stFrameInfo.enPixelType == 35127316:
        data = data.reshape(stFrameInfo.nHeight, stFrameInfo.nWidth, -1)
        image = cv2.cvtColor(data, cv2.COLOR_RGB2BGR)
        # image_show(image=image, name=stFrameInfo.nHeight)
    elif stFrameInfo.enPixelType == 34603039:
        data = data.reshape(stFrameInfo.nHeight, stFrameInfo.nWidth, -1)
        image = cv2.cvtColor(data, cv2.COLOR_YUV2BGR_Y422)
        # image_show(image=image, name=stFrameInfo.nHeight)
    return image


def enum_devices(device=0, device_way=False):
    """
    device = 0  枚举网口、USB口、未知设备、cameralink 设备
    device = 1 枚举GenTL设备
    """
    if device_way == False:
        if device == 0:
            tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE | MV_UNKNOW_DEVICE | MV_1394_DEVICE | MV_CAMERALINK_DEVICE
            deviceList = MV_CC_DEVICE_INFO_LIST()
            # 枚举设备
            ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
            if ret != 0:
                print("enum devices fail! ret[0x%x]" % ret)
                sys.exit()
            if deviceList.nDeviceNum == 0:
                print("find no device!")
                sys.exit()
            print("Find %d devices!" % deviceList.nDeviceNum)
            return deviceList
        else:
            pass
    elif device_way == True:
        pass


# 判断不同类型设备
def identify_different_devices(deviceList):
    # 判断不同类型设备，并输出相关信息
    for i in range(0, deviceList.nDeviceNum):
        mvcc_dev_info = cast(deviceList.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
        # 判断是否为网口相机
        if mvcc_dev_info.nTLayerType == MV_GIGE_DEVICE:
            print("\n网口设备序号: [%d]" % i)
            # 获取设备名
            strModeName = ""
            for per in mvcc_dev_info.SpecialInfo.stGigEInfo.chModelName:
                strModeName = strModeName + chr(per)
            print("当前设备型号名: %s" % strModeName)
            # 获取当前设备 IP 地址
            nip1_1 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24)
            nip1_2 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16)
            nip1_3 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8)
            nip1_4 = (mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff)
            print("当前 ip 地址: %d.%d.%d.%d" % (nip1_1, nip1_2, nip1_3, nip1_4))
            # 获取当前子网掩码
            nip2_1 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentSubNetMask & 0xff000000) >> 24)
            nip2_2 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentSubNetMask & 0x00ff0000) >> 16)
            nip2_3 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentSubNetMask & 0x0000ff00) >> 8)
            nip2_4 = (mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentSubNetMask & 0x000000ff)
            print("当前子网掩码 : %d.%d.%d.%d" % (nip2_1, nip2_2, nip2_3, nip2_4))
            # 获取当前网关
            nip3_1 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nDefultGateWay & 0xff000000) >> 24)
            nip3_2 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nDefultGateWay & 0x00ff0000) >> 16)
            nip3_3 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nDefultGateWay & 0x0000ff00) >> 8)
            nip3_4 = (mvcc_dev_info.SpecialInfo.stGigEInfo.nDefultGateWay & 0x000000ff)
            print("当前网关 : %d.%d.%d.%d" % (nip3_1, nip3_2, nip3_3, nip3_4))
            # 获取网口 IP 地址
            nip4_1 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nNetExport & 0xff000000) >> 24)
            nip4_2 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nNetExport & 0x00ff0000) >> 16)
            nip4_3 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nNetExport & 0x0000ff00) >> 8)
            nip4_4 = (mvcc_dev_info.SpecialInfo.stGigEInfo.nNetExport & 0x000000ff)
            print("当前连接的网口 IP 地址 : %d.%d.%d.%d" % (nip4_1, nip4_2, nip4_3, nip4_4))
            # 获取制造商名称
            strmanufacturerName = ""
            for per in mvcc_dev_info.SpecialInfo.stGigEInfo.chManufacturerName:
                strmanufacturerName = strmanufacturerName + chr(per)
            print("制造商名称 : %s" % strmanufacturerName)
            # 获取设备版本
            stdeviceversion = ""
            for per in mvcc_dev_info.SpecialInfo.stGigEInfo.chDeviceVersion:
                stdeviceversion = stdeviceversion + chr(per)
            print("设备当前使用固件版本 : %s" % stdeviceversion)
            # 获取制造商的具体信息
            stManufacturerSpecificInfo = ""
            for per in mvcc_dev_info.SpecialInfo.stGigEInfo.chManufacturerSpecificInfo:
                stManufacturerSpecificInfo = stManufacturerSpecificInfo + chr(per)
            print("设备制造商的具体信息 : %s" % stManufacturerSpecificInfo)
            # 获取设备序列号
            stSerialNumber = ""
            for per in mvcc_dev_info.SpecialInfo.stGigEInfo.chSerialNumber:
                stSerialNumber = stSerialNumber + chr(per)
            print("设备序列号 : %s" % stSerialNumber)
            # 获取用户自定义名称
            stUserDefinedName = ""
            for per in mvcc_dev_info.SpecialInfo.stGigEInfo.chUserDefinedName:
                stUserDefinedName = stUserDefinedName + chr(per)
            print("用户自定义名称 : %s" % stUserDefinedName)

        # 判断是否为 USB 接口相机
        elif mvcc_dev_info.nTLayerType == MV_USB_DEVICE:
            print("\nU3V 设备序号e: [%d]" % i)
            strModeName = ""
            for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chModelName:
                if per == 0:
                    break
                strModeName = strModeName + chr(per)
            print("当前设备型号名 : %s" % strModeName)
            strSerialNumber = ""
            for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chSerialNumber:
                if per == 0:
                    break
                strSerialNumber = strSerialNumber + chr(per)
            print("当前设备序列号 : %s" % strSerialNumber)
            # 获取制造商名称
            strmanufacturerName = ""
            for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chVendorName:
                strmanufacturerName = strmanufacturerName + chr(per)
            print("制造商名称 : %s" % strmanufacturerName)
            # 获取设备版本
            stdeviceversion = ""
            for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chDeviceVersion:
                stdeviceversion = stdeviceversion + chr(per)
            print("设备当前使用固件版本 : %s" % stdeviceversion)
            # 获取设备序列号
            stSerialNumber = ""
            for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chSerialNumber:
                stSerialNumber = stSerialNumber + chr(per)
            print("设备序列号 : %s" % stSerialNumber)
            # 获取用户自定义名称
            stUserDefinedName = ""
            for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chUserDefinedName:
                stUserDefinedName = stUserDefinedName + chr(per)
            print("用户自定义名称 : %s" % stUserDefinedName)
            # 获取设备 GUID
            stDeviceGUID = ""
            for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chDeviceGUID:
                stDeviceGUID = stDeviceGUID + chr(per)
            print("设备GUID号 : %s" % stDeviceGUID)
            # 获取设备的家族名称
            stFamilyName = ""
            for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chFamilyName:
                stFamilyName = stFamilyName + chr(per)
            print("设备的家族名称 : %s" % stFamilyName)

        # 判断是否为 1394-a/b 设备
        elif mvcc_dev_info.nTLayerType == MV_1394_DEVICE:
            print("\n1394-a/b device: [%d]" % i)

        # 判断是否为 cameralink 设备
        elif mvcc_dev_info.nTLayerType == MV_CAMERALINK_DEVICE:
            print("\ncameralink device: [%d]" % i)
            # 获取当前设备名
            strModeName = ""
            for per in mvcc_dev_info.SpecialInfo.stCamLInfo.chModelName:
                if per == 0:
                    break
                strModeName = strModeName + chr(per)
            print("当前设备型号名 : %s" % strModeName)
            # 获取当前设备序列号
            strSerialNumber = ""
            for per in mvcc_dev_info.SpecialInfo.stCamLInfo.chSerialNumber:
                if per == 0:
                    break
                strSerialNumber = strSerialNumber + chr(per)
            print("当前设备序列号 : %s" % strSerialNumber)
            # 获取制造商名称
            strmanufacturerName = ""
            for per in mvcc_dev_info.SpecialInfo.stCamLInfo.chVendorName:
                strmanufacturerName = strmanufacturerName + chr(per)
            print("制造商名称 : %s" % strmanufacturerName)
            # 获取设备版本
            stdeviceversion = ""
            for per in mvcc_dev_info.SpecialInfo.stCamLInfo.chDeviceVersion:
                stdeviceversion = stdeviceversion + chr(per)
            print("设备当前使用固件版本 : %s" % stdeviceversion)


# 输入需要连接的相机的序号
def input_num_camera(deviceList):
    nConnectionNum = input("please input the number of the device to connect:")
    if int(nConnectionNum) >= deviceList.nDeviceNum:
        print("intput error!")
        sys.exit()
    return nConnectionNum


# 创建相机实例并创建句柄,(设置日志路径)
def creat_camera(deviceList, nConnectionNum, log=True, log_path=getcwd()):
    """
    :param deviceList:        设备列表
    :param nConnectionNum:    需要连接的设备序号
    :param log:               是否创建日志
    :param log_path:          日志保存路径
    :return:                  相机实例和设备列表
    """
    # 创建相机实例
    cam = MvCamera()
    # 选择设备并创建句柄
    stDeviceList = cast(deviceList.pDeviceInfo[int(nConnectionNum)], POINTER(MV_CC_DEVICE_INFO)).contents
    if log == True:
        ret = cam.MV_CC_SetSDKLogPath(log_path)
        print(log_path)
        if ret != 0:
            print("set Log path  fail! ret[0x%x]" % ret)
            sys.exit()
        # 创建句柄,生成日志
        ret = cam.MV_CC_CreateHandle(stDeviceList)
        if ret != 0:
            print("create handle fail! ret[0x%x]" % ret)
            sys.exit()
    elif log == False:
        # 创建句柄,不生成日志
        ret = cam.MV_CC_CreateHandleWithoutLog(stDeviceList)
        print(1111)
        if ret != 0:
            print("create handle fail! ret[0x%x]" % ret)
            sys.exit()
    return cam, stDeviceList


# 打开设备
def open_device(cam):
    # ch:打开设备 | en:Open device
    ret = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
    if ret != 0:
        print("open device fail! ret[0x%x]" % ret)
        sys.exit()


# 获取各种类型节点参数
def get_Value(cam, param_type="int_value", node_name="PayloadSize"):
    """
    :param cam:            相机实例
    :param_type:           获取节点值得类型
    :param node_name:      节点名 可选 int 、float 、enum 、bool 、string 型节点
    :return:               节点值
    """
    if param_type == "int_value":
        stParam = MVCC_INTVALUE_EX()
        memset(byref(stParam), 0, sizeof(MVCC_INTVALUE_EX))
        ret = cam.MV_CC_GetIntValueEx(node_name, stParam)
        if ret != 0:
            print("获取 int 型数据 %s 失败 ! 报错码 ret[0x%x]" % (node_name, ret))
            sys.exit()
        int_value = stParam.nCurValue
        return int_value

    elif param_type == "float_value":
        stFloatValue = MVCC_FLOATVALUE()
        memset(byref(stFloatValue), 0, sizeof(MVCC_FLOATVALUE))
        ret = cam.MV_CC_GetFloatValue(node_name, stFloatValue)
        if ret != 0:
            print("获取 float 型数据 %s 失败 ! 报错码 ret[0x%x]" % (node_name, ret))
            sys.exit()
        float_value = stFloatValue.fCurValue
        return float_value

    elif param_type == "enum_value":
        stEnumValue = MVCC_ENUMVALUE()
        memset(byref(stEnumValue), 0, sizeof(MVCC_ENUMVALUE))
        ret = cam.MV_CC_GetEnumValue(node_name, stEnumValue)
        if ret != 0:
            print("获取 enum 型数据 %s 失败 ! 报错码 ret[0x%x]" % (node_name, ret))
            sys.exit()
        enum_value = stEnumValue.nCurValue
        return enum_value

    elif param_type == "bool_value":
        stBool = c_bool(False)
        ret = cam.MV_CC_GetBoolValue(node_name, stBool)
        if ret != 0:
            print("获取 bool 型数据 %s 失败 ! 报错码 ret[0x%x]" % (node_name, ret))
            sys.exit()
        return stBool.value

    elif param_type == "string_value":
        stStringValue = MVCC_STRINGVALUE()
        memset(byref(stStringValue), 0, sizeof(MVCC_STRINGVALUE))
        ret = cam.MV_CC_GetStringValue(node_name, stStringValue)
        if ret != 0:
            print("获取 string 型数据 %s 失败 ! 报错码 ret[0x%x]" % (node_name, ret))
            sys.exit()
        string_value = stStringValue.chCurValue
        return string_value


# 设置各种类型节点参数
def set_Value(cam, param_type="int_value", node_name="PayloadSize", node_value=None):
    """
    :param cam:               相机实例
    :param param_type:        需要设置的节点值得类型
        int:
        float:
        enum:     参考于客户端中该选项的 Enum Entry Value 值即可
        bool:     对应 0 为关，1 为开
        string:   输入值为数字或者英文字符，不能为汉字
    :param node_name:         需要设置的节点名
    :param node_value:        设置给节点的值
    :return:
    """

    if param_type == "int_value":
        stParam = int(node_value)
        ret = cam.MV_CC_SetIntValueEx(node_name, stParam)
        if ret != 0:
            print("设置 int 型数据节点 %s 失败 ! 报错码 ret[0x%x]" % (node_name, ret))
            sys.exit()
        print("设置 int 型数据节点 %s 成功 ！设置值为 %s !" % (node_name, node_value))

    elif param_type == "float_value":
        stFloatValue = float(node_value)
        ret = cam.MV_CC_SetFloatValue(node_name, stFloatValue)
        if ret != 0:
            print("设置 float 型数据节点 %s 失败 ! 报错码 ret[0x%x]" % (node_name, ret))
            sys.exit()
        print("设置 float 型数据节点 %s 成功 ！设置值为 %s !" % (node_name, node_value))

    elif param_type == "enum_value":
        stEnumValue = node_value
        ret = cam.MV_CC_SetEnumValue(node_name, stEnumValue)
        if ret != 0:
            print("设置 enum 型数据节点 %s 失败 ! 报错码 ret[0x%x]" % (node_name, ret))
            sys.exit()
        print("设置 enum 型数据节点 %s 成功 ！设置值为 %s !" % (node_name, node_value))

    elif param_type == "bool_value":
        ret = cam.MV_CC_SetBoolValue(node_name, node_value)
        if ret != 0:
            print("设置 bool 型数据节点 %s 失败 ！ 报错码 ret[0x%x]" % (node_name, ret))
            sys.exit()
        print("设置 bool 型数据节点 %s 成功 ！设置值为 %s !" % (node_name, node_value))

    elif param_type == "string_value":
        stStringValue = str(node_value)
        ret = cam.MV_CC_SetStringValue(node_name, stStringValue)
        if ret != 0:
            print("设置 string 型数据节点 %s 失败 ! 报错码 ret[0x%x]" % (node_name, ret))
            sys.exit()
        print("设置 string 型数据节点 %s 成功 ！设置值为 %s !" % (node_name, node_value))


# 寄存器读写
def read_or_write_memory(cam, way="read"):
    if way == "read":
        pass
        cam.MV_CC_ReadMemory()
    elif way == "write":
        pass
        cam.MV_CC_WriteMemory()


# 判断相机是否处于连接状态(返回值如何获取)=================================
def decide_divice_on_line(cam):
    value = cam.MV_CC_IsDeviceConnected()
    if value == True:
        print("该设备在线 ！")
    else:
        print("该设备已掉线 ！", value)


# 设置 SDK 内部图像缓存节点个数
def set_image_Node_num(cam, Num=1):
    ret = cam.MV_CC_SetImageNodeNum(nNum=Num)
    if ret != 0:
        print("设置 SDK 内部图像缓存节点个数失败 ,报错码 ret[0x%x]" % ret)
    else:
        print("设置 SDK 内部图像缓存节点个数为 %d  ，设置成功!" % Num)


# 设置取流策略
def set_grab_strategy(cam, grabstrategy=0, outputqueuesize=1):
    """
    • OneByOne: 从旧到新一帧一帧的从输出缓存列表中获取图像，打开设备后默认为该策略
    • LatestImagesOnly: 仅从输出缓存列表中获取最新的一帧图像，同时清空输出缓存列表
    • LatestImages: 从输出缓存列表中获取最新的OutputQueueSize帧图像，其中OutputQueueSize范围为1 - ImageNodeNum，可用MV_CC_SetOutputQueueSize()接口设置，ImageNodeNum默认为1，可用MV_CC_SetImageNodeNum()接口设置OutputQueueSize设置成1等同于LatestImagesOnly策略，OutputQueueSize设置成ImageNodeNum等同于OneByOne策略
    • UpcomingImage: 在调用取流接口时忽略输出缓存列表中所有图像，并等待设备即将生成的一帧图像。该策略只支持GigE设备，不支持U3V设备
    """
    if grabstrategy != 2:
        ret = cam.MV_CC_SetGrabStrategy(enGrabStrategy=grabstrategy)
        if ret != 0:
            print("设置取流策略失败 ,报错码 ret[0x%x]" % ret)
        else:
            print("设置 取流策略为 %d  ，设置成功!" % grabstrategy)
    else:
        ret = cam.MV_CC_SetGrabStrategy(enGrabStrategy=grabstrategy)
        if ret != 0:
            print("设置取流策略失败 ,报错码 ret[0x%x]" % ret)
        else:
            print("设置 取流策略为 %d  ，设置成功!" % grabstrategy)

        ret = cam.MV_CC_SetOutputQueueSize(nOutputQueueSize=outputqueuesize)
        if ret != 0:
            print("设置使出缓存个数失败 ,报错码 ret[0x%x]" % ret)
        else:
            print("设置 输出缓存个数为 %d  ，设置成功!" % outputqueuesize)


# 显示图像
def image_show(image, name):
    global img_test

    image = cv2.resize(image, (1200, 800), interpolation=cv2.INTER_AREA)
    img_test = image.copy()
    name = str(name)
    cv2.imshow(name, image)
    cv2.imwrite("data/images/name.bmp", image)
    k = cv2.waitKey(1) & 0xff





# 主动图像采集
def access_get_image(cam, active_way="getImagebuffer"):
    """
    :param cam:     相机实例
    :active_way:主动取流方式的不同方法 分别是（getImagebuffer）（getoneframetimeout）
    :return:
    """
    if active_way == "getImagebuffer":
        stOutFrame = MV_FRAME_OUT()
        memset(byref(stOutFrame), 0, sizeof(stOutFrame))
        while True:
            ret = cam.MV_CC_GetImageBuffer(stOutFrame, 1000)
            if None != stOutFrame.pBufAddr and 0 == ret and stOutFrame.stFrameInfo.enPixelType == 17301505:
                print("get one frame: Width[%d], Height[%d], nFrameNum[%d]" % (
                    stOutFrame.stFrameInfo.nWidth, stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nFrameNum))
                pData = (c_ubyte * stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight)()
                cdll.msvcrt.memcpy(byref(pData), stOutFrame.pBufAddr,
                                   stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight)
                data = np.frombuffer(pData, count=int(stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight),
                                     dtype=np.uint8)
                image_control(data=data, stFrameInfo=stOutFrame.stFrameInfo)
            elif None != stOutFrame.pBufAddr and 0 == ret and stOutFrame.stFrameInfo.enPixelType == 17301514:
                print("get one frame: Width[%d], Height[%d], nFrameNum[%d]" % (
                    stOutFrame.stFrameInfo.nWidth, stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nFrameNum))
                pData = (c_ubyte * stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight)()
                cdll.msvcrt.memcpy(byref(pData), stOutFrame.pBufAddr,
                                   stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight)
                data = np.frombuffer(pData, count=int(stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight),
                                     dtype=np.uint8)
                image_control(data=data, stFrameInfo=stOutFrame.stFrameInfo)
            elif None != stOutFrame.pBufAddr and 0 == ret and stOutFrame.stFrameInfo.enPixelType == 35127316:
                print("get one frame: Width[%d], Height[%d], nFrameNum[%d]" % (
                    stOutFrame.stFrameInfo.nWidth, stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nFrameNum))
                pData = (c_ubyte * stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 3)()
                cdll.msvcrt.memcpy(byref(pData), stOutFrame.pBufAddr,
                                   stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 3)
                data = np.frombuffer(pData,
                                     count=int(stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 3),
                                     dtype=np.uint8)
                image_control(data=data, stFrameInfo=stOutFrame.stFrameInfo)
            elif None != stOutFrame.pBufAddr and 0 == ret and stOutFrame.stFrameInfo.enPixelType == 34603039:
                print("get one frame: Width[%d], Height[%d], nFrameNum[%d]" % (
                    stOutFrame.stFrameInfo.nWidth, stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nFrameNum))
                pData = (c_ubyte * stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 2)()
                cdll.msvcrt.memcpy(byref(pData), stOutFrame.pBufAddr,
                                   stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 2)
                data = np.frombuffer(pData,
                                     count=int(stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 2),
                                     dtype=np.uint8)
                image_control(data=data, stFrameInfo=stOutFrame.stFrameInfo)
            else:
                print("no data[0x%x]" % ret)
            nRet = cam.MV_CC_FreeImageBuffer(stOutFrame)

    elif active_way == "getoneframetimeout":
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
                print("get one frame: Width[%d], Height[%d], nFrameNum[%d] " % (
                    stFrameInfo.nWidth, stFrameInfo.nHeight, stFrameInfo.nFrameNum))
                image = np.asarray(pData)
                image_control(data=image, stFrameInfo=stFrameInfo)
            else:
                print("no data[0x%x]" % ret)


# 回调取图采集
if sys.platform.startswith("win"):
    winfun_ctype = WINFUNCTYPE
else:
    winfun_ctype = CFUNCTYPE
stFrameInfo = POINTER(MV_FRAME_OUT_INFO_EX)
pData = POINTER(c_ubyte)
FrameInfoCallBack = winfun_ctype(None, pData, stFrameInfo, c_void_p)


def image_callback(pData, pFrameInfo, pUser):
    global img_buff
    img_buff = None
    stFrameInfo = cast(pFrameInfo, POINTER(MV_FRAME_OUT_INFO_EX)).contents
    if stFrameInfo:
        # print("get one frame: Width[%d], Height[%d], nFrameNum[%d]" % (
        # stFrameInfo.nWidth, stFrameInfo.nHeight, stFrameInfo.nFrameNum))
        print(stFrameInfo.enPixelType)

    if img_buff is None and stFrameInfo.enPixelType == 17301505:
        img_buff = (c_ubyte * stFrameInfo.nWidth * stFrameInfo.nHeight)()
        cdll.msvcrt.memcpy(byref(img_buff), pData, stFrameInfo.nWidth * stFrameInfo.nHeight)
        data = np.frombuffer(img_buff, count=int(stFrameInfo.nWidth * stFrameInfo.nHeight), dtype=np.uint8)
        image_control(data=data, stFrameInfo=stFrameInfo)
        del img_buff
    elif img_buff is None and stFrameInfo.enPixelType == 17301513:
        print(123)
        img_buff = (c_ubyte * stFrameInfo.nWidth * stFrameInfo.nHeight)()
        cdll.msvcrt.memcpy(byref(img_buff), pData, stFrameInfo.nWidth * stFrameInfo.nHeight)
        data = np.frombuffer(img_buff, count=int(stFrameInfo.nWidth * stFrameInfo.nHeight), dtype=np.uint8)
        image_control(data=data, stFrameInfo=stFrameInfo)
        del img_buff
    elif img_buff is None and stFrameInfo.enPixelType == 35127316:
        img_buff = (c_ubyte * stFrameInfo.nWidth * stFrameInfo.nHeight * 3)()
        cdll.msvcrt.memcpy(byref(img_buff), pData, stFrameInfo.nWidth * stFrameInfo.nHeight * 3)
        data = np.frombuffer(img_buff, count=int(stFrameInfo.nWidth * stFrameInfo.nHeight * 3), dtype=np.uint8)
        image_control(data=data, stFrameInfo=stFrameInfo)
        del img_buff
    elif img_buff is None and stFrameInfo.enPixelType == 34603039:
        img_buff = (c_ubyte * stFrameInfo.nWidth * stFrameInfo.nHeight * 2)()
        cdll.msvcrt.memcpy(byref(img_buff), pData, stFrameInfo.nWidth * stFrameInfo.nHeight * 2)
        data = np.frombuffer(img_buff, count=int(stFrameInfo.nWidth * stFrameInfo.nHeight * 2), dtype=np.uint8)
        image_control(data=data, stFrameInfo=stFrameInfo)
        del img_buff


CALL_BACK_FUN = FrameInfoCallBack(image_callback)

# 事件回调
stEventInfo = POINTER(MV_EVENT_OUT_INFO)
pData = POINTER(c_ubyte)
EventInfoCallBack = winfun_ctype(None, stEventInfo, c_void_p)


def event_callback(pEventInfo, pUser):
    stPEventInfo = cast(pEventInfo, POINTER(MV_EVENT_OUT_INFO)).contents
    nBlockId = stPEventInfo.nBlockIdHigh
    nBlockId = (nBlockId << 32) + stPEventInfo.nBlockIdLow
    nTimestamp = stPEventInfo.nTimestampHigh
    nTimestamp = (nTimestamp << 32) + stPEventInfo.nTimestampLow
    if stPEventInfo:
        print("EventName[%s], EventId[%u], BlockId[%d], Timestamp[%d]" % (
            stPEventInfo.EventName, stPEventInfo.nEventID, nBlockId, nTimestamp))


CALL_BACK_FUN_2 = EventInfoCallBack(event_callback)


# 注册回调取图
def call_back_get_image(cam):
    # ch:注册抓图回调 | en:Register image callback
    ret = cam.MV_CC_RegisterImageCallBackEx(CALL_BACK_FUN, None)
    if ret != 0:
        print("register image callback fail! ret[0x%x]" % ret)
        sys.exit()


# 关闭设备与销毁句柄
def close_and_destroy_device(cam, data_buf=None):
    # 停止取流
    ret = cam.MV_CC_StopGrabbing()
    if ret != 0:
        print("stop grabbing fail! ret[0x%x]" % ret)
        sys.exit()
    # 关闭设备
    ret = cam.MV_CC_CloseDevice()
    if ret != 0:
        print("close deivce fail! ret[0x%x]" % ret)
        del data_buf
        sys.exit()
    # 销毁句柄
    ret = cam.MV_CC_DestroyHandle()
    if ret != 0:
        print("destroy handle fail! ret[0x%x]" % ret)
        del data_buf
        sys.exit()
    del data_buf


# 开启取流并获取数据包大小
def start_grab_and_get_data_size(cam):
    ret = cam.MV_CC_StartGrabbing()
    if ret != 0:
        print("开始取流失败! ret[0x%x]" % ret)
        sys.exit()


def main():
    # 获得设备信息
    global img_test
    deviceList = MV_CC_DEVICE_INFO_LIST()
    tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE

    # ch:枚举设备 | en:Enum device
    # nTLayerType [IN] 枚举传输层 ，pstDevList [OUT] 设备列表
    ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
    if ret != 0:
        print("enum devices fail! ret[0x%x]" % ret)
        sys.exit()

    if deviceList.nDeviceNum == 0:
        print("find no device!")
        sys.exit()

    print("Find %d devices!" % deviceList.nDeviceNum)

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
    # 选择设备
    nConnectionNum = input("please input the number of the device to connect:")

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

    # # 设置缓存节点个数
    # set_image_Node_num(cam, Num=10)
    # # 设置取流策略
    # set_grab_strategy(cam, grabstrategy=2, outputqueuesize=10)
    # 设置设备的一些参数
    set_Value(cam, param_type="float_value", node_name="ExposureTime", node_value=15000)
    set_Value(cam, param_type="float_value", node_name="Gain", node_value=12)
    set_Value(cam, param_type="enum_value", node_name="PixelFormat", node_value=0x02180014)
    # 获取设备的一些参数
    # get_value = get_Value(cam , param_type = "int_value" , node_name = "PayloadSize")

    stdcall = input("回调方式取流显示请输入 0    主动取流方式显示请输入 1:")
    if int(stdcall) == 0:
        # 回调方式抓取图像
        call_back_get_image(cam)
        # 开启设备取流
        start_grab_and_get_data_size(cam)
        # 当使用 回调取流时，需要在此处添加
        print("press a key to stop grabbing.")
        cdll.msvcrt.getch()
        # 关闭设备与销毁句柄
        close_and_destroy_device(cam)
    elif int(stdcall) == 1:
        # 开启设备取流
        start_grab_and_get_data_size(cam)
        # 主动取流方式抓取图像
        access_get_image(cam, active_way="getImagebuffer")
        # 关闭设备与销毁句柄
        close_and_destroy_device(cam)


if __name__ == "__main__":
    main()
