# 导入需要的库
import os
import sys
import time
from pathlib import Path

# 初始化目录
import cv2

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # 定义YOLOv5的根目录
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # 将YOLOv5的根目录添加到环境变量中（程序结束后删除）
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from utils.general import (LOGGER, check_img_size, scale_boxes)

# 导入letterbox

import random
import torch
import numpy as np
from utils.general import non_max_suppression, xyxy2xywh
from utils.torch_utils import select_device
from utils.plots import Annotator
from models.common import DetectMultiBackend
from utils.augmentations import letterbox


class YOLOv5Detector:
    def __init__(self, weights_path, img_size=(640, 640), conf_thres=0.70, iou_thres=0.2, max_det=10,
                 device='', classes=None, agnostic_nms=False, augment=False, visualize=False, half=True, dnn=False,
                 data='data/coco128.yaml', ui=False):
        # 设置设备
        self.ui = ui
        self.device = select_device(device)

        # 加载模型
        self.model = DetectMultiBackend(weights_path, device=self.device, dnn=dnn, fp16=half, data=data)

        stride, self.names, pt, jit, onnx, engine = self.model.stride, self.model.names, self.model.pt, self.model.jit, self.model.onnx, self.model.engine
        self.img_size = check_img_size(img_size, s=stride)
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in self.names]
        # 使用半精度 Float16 推理
        self.half = half and (pt or jit or onnx or engine) and self.device.type != 'cpu'
        if pt or jit:
            self.model.model.half() if self.half else self.model.model.float()
        self.save_time = 0
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.max_det = max_det
        self.classes = classes
        self.agnostic_nms = agnostic_nms
        self.augment = augment
        self.visualize = visualize
        bs = 1  # batch_size
        # 开始预测
        self.model.warmup(imgsz=(1 if pt or self.model.triton else bs, 3, *self.img_size))  # warmup

    def predict(self, img):
        # 对图片进行处理

        im0 = img.copy()
        im = letterbox(im0, self.img_size, self.model.stride, auto=self.model.pt)[0]
        im = im.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        im = np.ascontiguousarray(im)

        im = torch.from_numpy(im).to(self.device)
        im = im.half() if self.half else im.float()
        im /= 255
        if len(im.shape) == 3:
            im = im[None]  # expand for batch dim

        # 预测
        pred = self.model(im, augment=self.augment, visualize=self.visualize)

        # NMS
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms,
                                   max_det=self.max_det)

        # 用于存放结果
        detections = []

        # 处理预测结果
        for i, det in enumerate(pred):
            # gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            if len(det):

                det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()
                # print(det)
                for *xyxy, conf, cls in reversed(det):
                    xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4))).view(-1).tolist()
                    xywh = [round(x) for x in xywh]
                    xywh = [xywh[0] - xywh[2] // 2, xywh[1] - xywh[3] // 2, xywh[2], xywh[3]]
                    if self.ui:
                        annotator = Annotator(np.ascontiguousarray(img), line_width=3, example=str(self.names))
                        # print(int(cls))
                        label = f'{self.names[int(cls)]} {conf:.2f}'
                        annotator.box_label(xyxy, label, color=self.colors[int(cls)])

                    cls = self.names[int(cls)]
                    conf = float(conf)
                    line = (cls, xywh, conf)
                    detections.append(line)
        # if self.save_data:
        #     if time.time() - self.save_time >= 1:
        #         self.save_time = time.time()
        #         save_path = "save_data/img/" + str(round(time.time())) + ".jpg"
        #         cv2.imwrite(save_path, img)
        #         # save_path = "save_data/img_ui/img_ui_" + str(round(time.time())) + ".jpg"
        #         # cv2.imwrite(save_path, im0)
                # print(1)

        # LOGGER.info(f'({t3 - t2:.3f}s)')

        return detections
