# _*_ coding:utf-8 _*_
"""
@File       : kinect_cam.py
@Project    : baggage-system-pyqt-rebuild
@Author     : DaveHao
@Contact_1  : dave@fcsoft.cc
@Software   : PyCharm
@License    : MulanPSL2
@Description: Kinect DK 相机接口
"""
import time

import numpy as np
from PyQt5.QtCore import QThread, pyqtSignal

import global_value as gv
from func import kinect_lib as pykinect
import open3d as o3d
import cv2
import datetime



def kinectConnectionCheck() -> str:
    """
    静态方法，检查kinect dk相机连接情况
        :return: 已连接的相机序列号，如未连接则为0
    """
    pykinect.initialize_libraries(module_k4a_path='../func/kinect_lib/k4a.dll')  # 加载dll文件
    try:
        res, device = pykinect.start_device()  # 尝试连接
        if res:
            return str(device.get_serialnum())  # 如果连接成功，返回序列号 get_serialnum()
        else:
            return '|USB设备未连接'  # 否则返回0
    except Exception as e:
        return '|' + str(e)  # 其他情况返回错误信息


class KinectDKThread(QThread):
    kinect_connected = pyqtSignal(str, bool)  # 相机连接成功与否的信号
    kinect_capture_done = pyqtSignal(str, str, bool)  # 相机拍摄完成信号

    def __init__(self, parent=None):
        super(KinectDKThread, self).__init__(parent)
        pykinect.initialize_libraries(module_k4a_path='func/kinect_lib/k4a.dll')  # 加载dll文件

    def run(self):
        _count = 0  # 一个简陋的计时器
        _run_flag = False  # 线程是否继续进行的标志
        while True:  # 如果相机连接异常
            if _count == 5:
                self.kinect_connected.emit('kinect|行李相机连接失败，请检查后重启系统！', False)  # 发出信号通知kinect连接失败
                break
            _res = kinectConnectionCheck()
            if not '|' in _res:  # 如果连接成功
                _run_flag = True  # 继续运行置位
                break
            else:
                _msg = _res.split('|')[-1]
                self.kinect_connected.emit(f'kinect|未检测到行李相机（{_count + 1}/5）：{_msg}', False)  # 发出信号通知kinect连接失败
                _count += 1
            time.sleep(1)
        if _run_flag:  # 只有相机连接成功才有必要进行下面的操作
            # 相机参数设置
            device_config = pykinect.default_configuration  # 拉取一个默认配置模板用于修改
            device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_720P# 彩色流分辨率为720p  1536P
            device_config.depth_mode = pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED   # 深度模式设置为去畸变窄视场  c K4A_DEPTH_MODE_WFOV_2X2BINNED
            res, device = pykinect.start_device(config=device_config)  # 启动相机
            if res:
                gv.setValue('kinectSerial', str(device.get_serialnum()))  # 如果连接成功，返回相机序列号
                print(str(device.get_serialnum()))
                self.kinect_connected.emit('kinect', True)  # 发出信号通知kinect连接成功
            # ----开始无限循环，直至线程被kill
            while True:
                try:
                    capture = device.update()  # 捕获一帧数据
                    flag_depth = True  # 深度采集标志，默认为真，避免卡在标志检测
                    flag_color, color_image = capture.get_color_image()  # 分离彩色帧
                    flag_capture = gv.getValue('baggageCaptureFlag')
                    flag_ir = True
                    flag_capture_2nd = gv.getValue('第二个行李相机拍照标志')
                    if flag_capture:  # 如果需要采集深度图
                        flag_depth, depth_image = capture.get_depth_image()  # 分离深度帧 capture.get_colored_depth_image()  capture.get_transformed_colored_depth_image()
                        dt = datetime.datetime.now().strftime('%Y%m%d%H%M%S')
                        cv2.imwrite(f'./data/baggage_depth{dt}.png', depth_image)
                    # if flag_capture:  # 如果需要采集深度图
                    #     flag_depth, depth_image1 = capture.get_transformed_depth_image() # 分离深度帧 capture.get_colored_depth_image()  capture.get_transformed_colored_depth_image()
                    #     dt = datetime.datetime.now().strftime('%Y%m%d%H%M%S')
                    #     cv2.imwrite(f'./data/baggage_tr{dt}.png', depth_image1)
                    # if flag_capture:  # 如果需要采集点云
                    #     flag_ir , ir_image = capture.get_transformed_colored_depth_image() # 分离深度帧 capture.get_colored_depth_image()  capture.get_transformed_colored_depth_image()
                    #     dt = datetime.datetime.now().strftime('%Y%m%d%H%M%S')
                    #     cv2.imwrite(f'./data/baggage_tr_co{dt}.png', ir_image)
                    if not (flag_color and flag_depth):  # 如没有准备好则重复该循环
                        continue
                    if flag_capture:  # 如果需要采集点云
                        ret, points = capture.get_pointcloud() # 获取点信息
                        pcd = o3d.geometry.PointCloud()  # 创建空点云用于保存点信息
                        pcd.points = o3d.utility.Vector3dVector(points)  # ndarray转换为open3D的点云
                        gv.setValue('baggagePointCloud', pcd)  # 将点云保存在公共变量中
                        gv.setValue('baggageImageCaptured', color_image)  # 顺便保存图像
                        self.kinect_capture_done.emit('行李拍照成功', '', gv.getValue('debugFlag'))
                        if gv.getValue('saveBaggageFlag'):  # 如果需要保存
                            print('保存了第一件行李信息')
                            dt = datetime.datetime.now().strftime('%Y%m%d%H%M%S')
                            pcd.paint_uniform_color((50/255, 205/255, 50/255))  # 史莱姆绿
                            o3d.io.write_point_cloud(f'./data/baggage_{dt}.ply', pcd)
                            o3d.io.write_point_cloud(f'./data/baggage_latest.ply', pcd)
                            cv2.imwrite(f'./data/baggage_{dt}.jpg', color_image)  # 保存到记录文件
                            cv2.imwrite(f'./data/baggage_latest.jpg', color_image)  # 保存到文件
                            gv.setValue('saveBaggageFlag', False)  # 重置保存标志
                            self.kinect_capture_done.emit('行李保存成功', f'./data/baggage_{dt}.ply',
                                                          True)  # 发送信号通知stackEventHandle方法
                            gv.setValue('saveOK', True)
                            print(gv.getValue('saveOK'))
                        gv.setValue('baggageCaptureFlag', False)  # 重置采集标志

                    # img = cv2.resize(color_image, dsize=(int(color_image.shape[1] / 2), int(color_image.shape[0] / 2)),
                    #                  interpolation=cv2.INTER_CUBIC)  # 减小一半的分辨率
#####################################################################################################################################

                    # if flag_capture_2nd:  # 如果第二件行李需要采集点云
                    #     flag_depth, depth_image = capture.get_colored_depth_image()  # 分离深度帧
                    # if not (flag_color and flag_depth):  # 如没有准备好则重复该循环
                    #     continue
                    # if flag_capture_2nd:  # 如果需要采集点云
                    #     ret, points = capture.get_pointcloud()  # 获取点信息
                    #     pcd = o3d.geometry.PointCloud()  # 创建空点云用于保存点信息
                    #     pcd.points = o3d.utility.Vector3dVector(points)  # ndarray转换为open3D的点云
                    #     gv.setValue('baggagePointCloud_2nd', pcd)  # 将点云保存在公共变量中
                    #     gv.setValue('baggageImageCaptured_2nd', color_image)  # 顺便保存图像
                    #     self.kinect_capture_done.emit('第二件行李拍照成功', '', gv.getValue('debugFlag'))
                    #     if gv.getValue('saveBaggageFlag_2nd'):  # 如果需要保存
                    #         print('保存了第二件行李信息')
                    #         dt = datetime.datetime.now().strftime('%Y%m%d%H%M%S')
                    #         pcd.paint_uniform_color((50/255, 205/255, 50/255))  # 史莱姆绿
                    #         o3d.io.write_point_cloud(f'./data/2nd_baggage_{dt}.ply', pcd)
                    #         o3d.io.write_point_cloud(f'./data/2nd_baggage_latest.ply', pcd)
                    #         cv2.imwrite(f'./data/2nd_baggage_{dt}.jpg', color_image)  # 保存到记录文件
                    #         cv2.imwrite(f'./data/2nd_baggage_latest.jpg', color_image)  # 保存到文件
                    #         gv.setValue('saveBaggageFlag_2nd', False)  # 重置保存标志
                    #         self.kinect_capture_done.emit('第二件行李保存成功', f'./data/2nd_baggage_{dt}.ply',
                    #                                       True)  # 发送信号通知stackEventHandle方法
                    #     gv.setValue('第二个行李相机拍照标志', False)  # 重置采集标志
########################################################################################################################################


                    img = cv2.resize(color_image, dsize=(int(color_image.shape[1]), int(color_image.shape[0])),
                                     interpolation=cv2.INTER_CUBIC)  # 不减小分辨率
                    # img = color_image[:, abs(color_image.shape[1] - color_image.shape[0]) // 2:max(
                    #     color_image.shape[1], color_image.shape[0]) - abs(
                    #     color_image.shape[1] - color_image.shape[0]) // 2, :]  # 把图像切割成正方形
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # 转换色彩空间
                    # 保存到对应位置的公共变量
                    gv.setValue('baggageImg', img)



                except Exception as e:
                    print(e)
                    pass
                finally:
                    pass
        else:
            pass


if __name__ == '__main__':
    pass
