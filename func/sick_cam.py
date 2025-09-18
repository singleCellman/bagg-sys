# _*_ coding:utf-8 _*_
"""
@File       : sick_cam.py
@Project    : baggage-system-pyqt-rebuild
@Author     : DaveHao
@Contact_1  : dave@fcsoft.cc
@Software   : PyCharm
@License    : MulanPSL2
@Description:
"""
import datetime
import sys

from PyQt5.QtCore import QThread, pyqtSignal

import global_value as gv

from func.sick_lib import Device
from func.sick_lib import Data
import logging
import argparse
import numpy as np
import open3d as o3d
import cv2
import time

class SickThread(QThread):
    sick_connected = pyqtSignal(str, bool)  # 相机连接成功与否的信号
    sick_capture_done = pyqtSignal(str, str, bool)  # 相机拍摄完成信号

    def __init__(self, parent=None):
        super(SickThread, self).__init__(parent)
        self.sick_ip = gv.getParam('stackCameraIP')
        self.port = gv.getParam('stackCameraPort')
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.WARNING)


    def run(self):
        _count = 0
        _run_flag = False
        while True:
            if _count == 5:
                self.sick_connected.emit('sick|垛型相机连接失败，请检查后重启系统！', False)  # 发出错误信号
                _run_flag = False
                break
            try:
                self.device_control = Device.Control(self.sick_ip)
                self.device_control.open()
                self.device_control.login(Device.USERLEVEL_SERVICE, 'CUST_SERV')
                self.device_control.enableDepthMapDataTransfer()
                self.device_control.logout()
                self.device_control.close()
                self.device_streaming = Device.Streaming(self.sick_ip, self.port)
                self.device_streaming.openStream()
                self.device_streaming.sendBlobRequest()
                self.cam_data = Data.Data()
                _run_flag = True
                self.sick_connected.emit('sick', True)  # 发出连接成功信号
                break
            except Exception as e:
                self.sick_connected.emit(f'sick|未检测到垛型相机（{_count + 1}/5）：{str(e)}', False)  # 发出错误信号
                _count += 1
                time.sleep(1)
        while _run_flag:
            try:
                self.device_streaming.getFrame()  # 获取下一帧
                whole_frame = self.device_streaming.frame
                self.cam_data.read(whole_frame)  # 解构帧数据
                self.cam_params = self.cam_data.cameraParams  # 相机内参
                self.num_cols = self.cam_data.cameraParams.width
                self.num_rows = self.cam_data.cameraParams.height
                self.f2rc = self.cam_params.f2rc / 1000.0
                self.pixel_size_z = 1.0
                if self.cam_data.hasDepthMap:  # 如果深度数据存在
                    distance_data_raw = np.array(self.cam_data.depthmap.distance, dtype=float)  # 读取深度数据并转为ndarray
                    distace_data_show = np.zeros(self.cam_params.width * self.cam_params.height)  # 创建一个用于显示的数组
                    # 剔除离群数据
                    count = 0
                    for x in distance_data_raw:
                        if x < 1000:
                            x = 1000
                        if x > 4300:
                            x = 4300
                        distace_data_show[count] = x
                        count += 1
                    _tmp_data = np.asarray(distace_data_show.reshape((self.num_rows, self.num_cols)))  # 重塑形状
                    stack_img = (_tmp_data - np.min(_tmp_data)) / (
                            np.max(_tmp_data) - np.min(_tmp_data)) * 255 // 1  # 归一化
                    stack_img_show = np.rot90(stack_img, 2)
                    gv.setValue('stackImg', stack_img_show)  # 存入公共变量
                    if gv.getValue('stackCaptureFlag'):  # 如果需要采集点云
                        pcd_array = np.zeros([self.num_rows * self.num_cols, 3])  # 建立一个点云空数组用于保存
                        # cv2.imwrite(f'./data/stack_color.jpg', self.cam_data.polarData2D)
                        # 深度图转为点云
                        count = 0
                        for row in range(self.num_rows):
                            y_p = (self.cam_params.cy - row) / self.cam_params.fy
                            y_p_2 = y_p * y_p
                            for col in range(self.num_cols):
                                if distance_data_raw[count] > 1000 and distance_data_raw[count] < 4300:
                                    x_p = (self.cam_params.cx - col) / self.cam_params.fx
                                    r_2 = x_p * x_p + y_p_2
                                    r_4 = r_2 * r_2
                                    k = 1 + self.cam_params.k1 * r_2 + self.cam_params.k2 * r_4
                                    x = x_p * k
                                    y = y_p * k
                                    z = 1.0
                                    s_0 = (x ** 2 + y ** 2 + z ** 2) ** 0.5 * 1000
                                    x /= s_0
                                    y /= s_0
                                    z /= s_0
                                    x = (x * distance_data_raw[count] * self.pixel_size_z) * 1000.
                                    y = (y * distance_data_raw[count] * self.pixel_size_z) * 1000.
                                    z = (z * distance_data_raw[count] * self.pixel_size_z - self.f2rc) * 1000.
                                    pcd_array[count, :] = [x, y, z]
                                count += 1
                        pcd = o3d.geometry.PointCloud()
                        pcd.points = o3d.utility.Vector3dVector(pcd_array)
                        gv.setValue('baggagePointCloud', pcd)  # 点云数据写入公共变量
                        gv.setValue('stackCaptureFlag', False)
                        self.sick_capture_done.emit('垛型拍照成功', '', gv.getValue('debugFlag'))
                        if gv.getValue('saveStackFlag'):  # 如果需要保存点云
                            dt = datetime.datetime.now().strftime('%Y%m%d%H%M%S')
                            o3d.io.write_point_cloud(f'./data/stack_{dt}.ply', pcd)
                            o3d.io.write_point_cloud('./data/stack_latest.ply', pcd)
                            gv.setValue('saveStackFlag', False)
                            self.sick_capture_done.emit('垛型保存成功', f'./data/stack_{dt}.ply', True)
            except:
                pass
            finally:
                pass