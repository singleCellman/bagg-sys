# _*_ coding:utf-8 _*_
"""
@File       : classification.py
@Project    : baggage-system-pyqt-rebuild
@Author     : DaveHao
@Contact_1  : dave@fcsoft.cc
@Software   : PyCharm
@License    : MulanPSL2
@Description: 图像分类脚本
"""
import os.path

import onnxruntime as ort
import cv2
import time
import numpy as np
import global_value as gv
import torch
import torchvision.transforms as transforms
from PyQt5.QtCore import QThread, pyqtSignal


class Classification(QThread):
    baggage_classify_done = pyqtSignal(str, str, bool)  # 行李分类成功的信号

    def __init__(self, parent=None):
        super(Classification, self).__init__(parent)
        self.img = None
        # 加载模型
        self.modelPath = gv.getParam('classifyModelPath')
        self.net = torch.load(self.modelPath)

    def run(self):
        while True:  # 默认进入死循环
            if gv.getValue('baggageClassifyFlag'):  # 如果需要分类
                if gv.getValue('stackCaptureFlag'):  # 如果是调试模式
                    if os.path.exists('./data/baggage_latest.jpg'):  # 检测图像是否存在
                        self.img = cv2.imread('./data/baggage_latest.jpg')  # 如果存在，则从本地文件加载图像
                    else:
                        self.baggage_classify_done.emit('行李分类结束', '错误：没有检测到行李图像，请确保行李图像位于./data/baggage_latest.jpg',
                                                        True)  # 做出对应提醒
                else:  # 如果不是调试模式
                    self.img = gv.getValue('baggageImageCaptured')  # 从公共变量加载图像
                try:
                    # 载入图像处理参数
                    size = gv.getParam('imgInputSize')
                    baggageImgCropXmin = gv.getParam('baggageImgCropXmin') / 100.0
                    baggageImgCropXmax = gv.getParam('baggageImgCropXmax') / 100.0
                    baggageImgCropYmin = gv.getParam('baggageImgCropYmin') / 100.0
                    baggageImgCropYmax = gv.getParam('baggageImgCropYmax') / 100.0
                    height, width = self.img.shape[:2]  # 获取图片长和宽
                    # 制作裁剪框
                    start_row, start_col = int(width * baggageImgCropXmin), int(height * baggageImgCropYmin)
                    end_row, end_col = int(width * baggageImgCropXmax), int(height * baggageImgCropYmax)
                    cropped = self.img[start_row:end_row, start_col:end_col]  # 执行裁剪
                    cropped = cv2.cvtColor(cropped, cv2.COLOR_BGR2RGB)  # 转为RGB
                    transform = transforms.Compose([transforms.Resize((size, size)),
                                                    transforms.ToTensor(), ])  # 调整尺寸并转为张量
                    img_tensor = transform(cropped).unsqueeze(0)  # 展平
                    if torch.cuda.is_available():
                        img_tensor = img_tensor.cuda()  # 如果显卡可用则使用显卡
                    net_output = self.net(img_tensor)  # 喂给模型
                    _, predicted = torch.max(net_output.data, 1)  # 得到预测结果
                    result = predicted[0].item()
                    self.baggage_classify_done.emit('行李分类完成', str(result),
                                                    gv.getValue('stackCaptureFlag'))  # 发出信号通知stackEventHandle方法
                except Exception as e:
                    self.baggage_classify_done.emit('行李分类完成', f'错误：str(e)',
                                                    gv.getValue('stackCaptureFlag'))
                gv.setValue('baggageClassifyFlag', False)  # 不论结果如何都复位分类标志
                time.sleep(1 / 20)  # 扫描频率20Hz


if __name__ == '__main__':
    pass
