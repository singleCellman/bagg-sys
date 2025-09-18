# _*_ coding:utf-8 _*_
"""
@File       : main_window_overload.py
@Project    : baggage-system-pyqt-rebuild
@Author     : DaveHao
@Contact_1  : dave@fcsoft.cc
@Software   : PyCharm
@License    : MulanPSL2
@Description: 窗体模型，继承ui文件窗体类，并添加自定义信号等操作
"""
import os
import time
import datetime

import cv2
import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl

from PyQt5 import QtGui
from PyQt5.QtGui import QIcon, QBrush, QColor
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QRegExp, QTimer, Qt

import global_value as gv
from func import KinectDKThread, SickThread, get_host_ip, SocketHelper, BaggageMeasure, StackDeconstruction, \
    stack_planning, BaggageMeasure_2nd
from func.opengl_helper import createVoxelMesh
from stack.evaluation import stackPlanning

# PyQtGraph 的显示配置，该设置应在窗体实例化之前进行
pg.setConfigOption('background', 'w')  # 设置显示背景色为白色
pg.setConfigOption('foreground', 'k')  # 设置显示前景色为黑色
# 设置图像显示以行为主，默认以列为主，不进行此设置会导致图像转置
pg.setConfigOption('imageAxisOrder', 'row-major')
uiClass, baseClass = pg.Qt.loadUiType("./UI/main_window.ui")  # 动态加载ui文件


class MainWindowFrame(uiClass, baseClass, SocketHelper):
    """
    继承于UI基类的自定义窗体框架
    """

    def __init__(self):
        super(MainWindowFrame, self).__init__()
        self.setupUi(self)
        ###################################################
        # lineEdit所使用的内容格式验证器
        ###################################################
        regex_percent_2 = QRegExp("^[1-9]?[0-9](\.[0-9]{1,2})?$")  # 正则，匹配最多两位小数的百分数
        regex_angle_90_1 = QRegExp("^-?(90|90.0|-90.0|[1-8]?\d(\.\d)?)$")  # 正则，匹配-90~90间的角度
        regex_angle_180_1 = QRegExp("^-?(180|180.0|-180.0|[1]?[1-7]?[0-9](\.[0-9]{1,3})?)$")  # 正则，匹配-180~180间的角度，三位小数
        regex_float_1 = QRegExp("^-?[0-9]+(\.[0-9]{1})?$")  # 正则，匹配最多一位小数的浮点数
        regex_float_2 = QRegExp("^-?[0-9]+(\.[0-9]{2})?$")  # 正则，匹配最多两位小数的浮点数
        regex_float_3 = QRegExp("^-?[0-9]+(\.[0-9]{3})?$")  # 正则，匹配最多三位小数的浮点数
        regex_ip_addr = QRegExp("^(\.((2(5[0-5]|[0-4]\d))|[0-1]?\d{1,2})){3}$")  # 正则，匹配IP地址
        self.validator_percent_2 = QtGui.QRegExpValidator(regex_percent_2)
        self.validator_angle_90_1 = QtGui.QRegExpValidator(regex_angle_90_1)
        self.validator_angle_180_1 = QtGui.QRegExpValidator(regex_angle_180_1)
        self.validator_float_1 = QtGui.QRegExpValidator(regex_float_1)
        self.validator_float_2 = QtGui.QRegExpValidator(regex_float_2)
        self.validator_float_3 = QtGui.QRegExpValidator(regex_float_3)
        self.validator_ip_addr = QtGui.QRegExpValidator(regex_ip_addr)
        ###################################################
        # UI设置
        ###################################################
        # 加载三种提示图标
        self.info_icon = QIcon('./UI/info.ico')
        self.warning_icon = QIcon('./UI/warning.ico')
        self.error_icon = QIcon('./UI/error.ico')
        self.lbl_stackModelName.setStyleSheet('color: blue')  # 设置码垛规划模型路径显示的颜色
        self.lbl_classifyModelName.setStyleSheet('color: blue')  # 设置分类模型路径显示的颜色
        # 隐藏ImageView控件中的直方图，菜单按钮，ROI等选项
        self.wg_baggageCameraShow.ui.histogram.hide()
        self.wg_baggageCameraShow.ui.menuBtn.hide()
        self.wg_baggageCameraShow.ui.roiBtn.hide()
        self.wg_stackCameraShow.ui.histogram.hide()
        self.wg_stackCameraShow.ui.menuBtn.hide()
        self.wg_stackCameraShow.ui.roiBtn.hide()
        # 设置OpenGLWidget缺省属性
        self.gl_stackDiagram.setBackgroundColor('whitesmoke')  # 设置背景色
        self.gl_stackDiagram.setCameraPosition(
            pos=pg.Vector(int(gv.getParam('gridX')) / 2, int(gv.getParam('gridY') / 2), 0), distance=400,
            azimuth=-90)  # 设置视角
        # 设置码垛结果面板的表头样式
        self.tw_stackOutput.setRowCount(0)
        self.tw_stackOutput.setColumnCount(5)
        self.tw_stackOutput.setHorizontalHeaderLabels(['尺寸', '接方式', '接位置', '放方式', '位置'])
        font = self.tw_stackOutput.horizontalHeader().font()
        font.setBold(True)
        self.tw_stackOutput.horizontalHeader().setFont(font)
        self.tw_stackOutput.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tw_stackOutput.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.tw_stackOutput.horizontalHeader().setSectionResizeMode(4, QHeaderView.ResizeToContents)
        self.tw_stackOutput.setEditTriggers(QAbstractItemView.NoEditTriggers)  # 禁止编辑
        self.tw_stackOutput.setSelectionBehavior(QAbstractItemView.SelectRows)  # 整行选择
        # 行列尺寸自适应
        self.tw_stackOutput.resizeColumnsToContents()
        self.tw_stackOutput.resizeRowsToContents()
        # 表格表头的显示与隐藏
        # self.tw_stackOutput.verticalHeader().setVisible(False)
        self.tw_stackOutput.horizontalHeader().setVisible(True)
        ###################################################
        # 标志量设置
        ###################################################
        self.socket_status = False
        ###################################################
        # 后台循环线程
        ###################################################
        self.sick_thread = SickThread()  # 实例化sick相机的线程
        self.kinect_thread = KinectDKThread()  # 实例化kinect相机的线程
        self.timer = QTimer()  # 设置一个定时器用来刷新相机画面，注意，实时画面刷新不要用信号，会导致严重卡顿
        ###################################################
        # 后台非循环线程
        ###################################################
        self.stack_deconstruction_thread = StackDeconstruction()  # 实例化垛型解构线程
        # self.stack_plan_thread = stackPlanning()  # 实例化码垛规划线程

        # 信号绑定
        ###################################################
        # 主窗体
        self.tw_interfacePanel.currentChanged['int'].connect(self.changePanel)
        # --控制面板
        # self.pb_startStackCycling.clicked.connect(lambda: self.toggleRunningStatus("","",True,"","")) #### 不正确 不能从开始就传入，否则会卡崩
        self.pb_startStackCycling.clicked.connect(self.toggleRunningStatus)
        ########## 修改了
        # self.pb_startStackCycling.clicked.connect(lambda: self.toggleRunningStatus("", ""))#### 不正确 不能从开始就传入，否则会卡崩
        # self.sick_thread.sick_connected.connect(self.toggleRunningStatus)  # 绑定sick相机线程的信号
        # self.sick_thread.sick_capture_done.connect(self.toggleRunningStatus)  # 绑定sick相机拍摄完成信号
        # self.kinect_thread.kinect_connected.connect(self.toggleRunningStatus)  # 绑定kinect相机线程的信号
        # self.kinect_thread.kinect_capture_done.connect(self.toggleRunningStatus)  # 绑定kinect相机拍摄完成信号
        # self.stack_deconstruction_thread.stack_deconstruction_done.connect(self.toggleRunningStatus)  # 垛型解构信号绑定
        # self.stack_plan_thread.stack_plan_done.connect(self.toggleRunningStatus)  # 码垛规划信号绑定

        ###### 没用
        # self.baggage_measure_thread = BaggageMeasure()  # 实例化行李测量线程
        # self.baggage_measure_thread.baggage_measure_done.connect(self.stackEventHandle)  # 绑定测量成功信号到本地
        ###################################################

        #######################################
        self.socketMsg.connect(self.displayLog)  # 绑定收到的socket数据到日志显示方法中
        # self.socketMsg.connect(self.toggleRunningStatus)
        # self.socketMsg.connect(self.onSocketDataReceived)
        # self.onSocketDataReceived(msg="ready")
        #######################################

        self.pb_emergencyStop.clicked.connect(self.emergencyStop)
        self.pb_saveLog.clicked.connect(lambda: self.saveLog(True))
        self.pb_clearLog.clicked.connect(self.clearLog)
        # --调试面板
        # ----码垛控制
        self.pb_baggageCapture.clicked.connect(self.baggagePointcloudCapture)
        self.pb_baggageClassify.clicked.connect(self.baggageClassify)
        self.pb_baggageMeasure.clicked.connect(self.baggageMeasure)
        self.pb_openFolder.clicked.connect(openFolder)
        self.pb_stackCapture.clicked.connect(self.stackPointcloudCapture)
        self.pb_stackPlan.clicked.connect(self.stackPlan)
        self.pb_stackDeconstruction.clicked.connect(self.stackDeconstruction)
        # ----码垛数据
        self.pb_clearStackData.clicked.connect(self.clearStackData)
        self.pb_sendStackData.clicked.connect(self.sendRandomStackData)
        # ----Socket调试
        self.pb_toggleListen.clicked.connect(self.toggleSocketServer)
        self.pb_clearSendData.clicked.connect(lambda: self.clearData('send'))
        self.pb_clearReceivedData.clicked.connect(lambda: self.clearData('rec'))
        self.pb_sendCustomData.clicked.connect(lambda: self.sendCustomData(self.te_sendData.toPlainText()))
        # --码垛设置
        self.pb_unlockStackSettings.clicked.connect(self.settingsLogic)
        self.pb_loadStackDefault.clicked.connect(self.loadDefault)
        self.pb_saveStackAsDefault.clicked.connect(self.saveAsDefault)
        # ----设备信息
        self.pb_refreshIPList.clicked.connect(self.refreshIPList)
        # --算法设置
        self.pb_unlockAlgorithmSettings.clicked.connect(self.settingsLogic)
        self.pb_loadAlgorithmDefault.clicked.connect(self.loadDefault)
        self.pb_saveAlgorithmAsDefault.clicked.connect(self.saveAsDefault)
        # ----码垛算法设置
        self.pb_loadStackModel.clicked.connect(self.loadStackModel)
        # ----分类算法设置
        self.pb_loadClassifyModel.clicked.connect(self.loadClassifyModel)
        # 其他信号
        self.timer.timeout.connect(self.updateLogic)  # 把定时器产生的信号绑定到对应的槽
        self.sick_thread.sick_connected.connect(self.cameraHandle)  # 绑定sick相机线程的信号
        self.sick_thread.sick_capture_done.connect(self.stackEventHandle)  # 绑定sick相机拍摄完成信号
        self.kinect_thread.kinect_connected.connect(self.cameraHandle)  # 绑定kinect相机线程的信号
        self.kinect_thread.kinect_capture_done.connect(self.stackEventHandle)  # 绑定kinect相机拍摄完成信号
        self.stack_deconstruction_thread.stack_deconstruction_done.connect(self.stackEventHandle)  # 垛型解构信号绑定
        # self.stack_plan_thread.stack_plan_done.connect(self.stackEventHandle)  # 码垛规划信号绑定

        ###################################################
        # 启动循环线程
        ###################################################
        self.timer.start(1 / 20)  # 启动定时器并设定刷新率为20fps
        self.sick_thread.start()  # 启动sick相机线程
        time.sleep(0.02)  # 延迟20ms避免出现CPU尖峰
        self.kinect_thread.start()  # 启动kinect相机线程
        ###################################################
        # 附加的动态显示内容
        ###################################################
        # 显示模型文件路径
        _stack_model_name = os.path.basename(gv.getParam('stackModelPath'))
        self.lbl_stackModelName.setText(_stack_model_name)
        _classify_model_name = os.path.basename(gv.getParam('classifyModelPath'))
        self.lbl_classifyModelName.setText(_classify_model_name)
        # 将本机IP列表装入comboBox中
        self.cb_hostIP.addItems(get_host_ip())
        self.loadSettings()  # 载入设置参数
        self.toggleWidgets()  # 锁定设置界面
        self.validatorInit()  # 初始化各个输入框的内容格式验证器
        # 显示启动欢迎语
        self.displayLog('欢迎使用机场行李码垛系统，等待相机连接。。。')
        # 绘制虚拟码垛空间
        self.drawVirtualStack(mode='重置')

    def drawVirtualStack(self, mode='重置'):
        if mode == '重置':  # 重置模式，仅绘制车体
            # 从设置参数中获取车体尺寸
            _space_length = int(gv.getParam('gridX'))
            _space_width = int(gv.getParam('gridY'))
            _space_height = int(gv.getParam('gridZ'))
            # 车体四个面的绘制起点
            p_array = np.array([[0, 0, -1], [-1, 0, -1], [_space_length, 0, -1], [-1, _space_width, -1]])
            # 车体四个面的尺寸
            size_array = np.array([[_space_length, _space_width, 1], [1, _space_width + 1, _space_height + 21],
                                   [1, _space_width + 1, _space_height + 21],
                                   [_space_length + 2, 1, _space_height + 21]])
            # 生成OpenGL用的mesh数据
            _mesh = createVoxelMesh(pos=p_array, size=size_array, facecolor=(0, 191 / 255, 1, 1))
            # 设置
            self.gl_stackDiagram.setCameraPosition(pos=pg.Vector(int(_space_length) / 2, int(_space_width / 2), 0),
                                                   distance=400,
                                                   azimuth=-90, elevation=60)  # 设置视角
            self.gl_stackDiagram.clear()
            self.gl_stackDiagram.addItem(_mesh)
            # 绘制网格
            _grid_length = (_space_length // 10 + 1) * 10
            _grid_width = (_space_width // 10 + 1) * 10
            _grid_height = (_space_height // 10 + 2) * 10
            grid_bottom = gl.GLGridItem(glOptions='opaque')  # 实例化一个网格对象
            grid_bottom.setSize(_grid_length, _grid_width, 10)  # 设置好长度和宽度，高度随便填
            grid_bottom.setSpacing(10, 10, 10)  # 网格间距选择为10
            grid_bottom.setColor('gold')  # 颜色为金色
            grid_bottom.translate(_grid_length // 2, _grid_width // 2, 0)  # 调整网格位置
            self.gl_stackDiagram.addItem(grid_bottom)  # 添加到控件中显示
            grid_left = gl.GLGridItem(glOptions='opaque')
            grid_left.rotate(90, 0, 1, 0)  # 沿y轴旋转90度
            grid_left.setSize(_grid_height, _grid_width, 10)
            grid_left.setSpacing(10, 10, 10)
            grid_left.setColor('gold')
            grid_left.translate(0, _grid_width // 2, _grid_height // 2)
            self.gl_stackDiagram.addItem(grid_left)
            grid_right = gl.GLGridItem(glOptions='opaque')
            grid_right.rotate(90, 0, 1, 0)  # 沿y轴旋转90度
            grid_right.setSize(_grid_height, _grid_width, 10)
            grid_right.setSpacing(10, 10, 10)
            grid_right.setColor('gold')
            grid_right.translate(_space_length, _grid_width // 2, _grid_height // 2)  # 注意右侧网格的x平移距离为码垛空间长，否则贴不到空间侧壁
            self.gl_stackDiagram.addItem(grid_right)
            grid_back = gl.GLGridItem(glOptions='opaque')
            grid_back.rotate(90, 1, 0, 0)  # 沿x轴旋转90度
            grid_back.setSize(_grid_length, _grid_height, 10)
            grid_back.setSpacing(10, 10, 10)
            grid_back.setColor('gold')
            grid_back.translate(_grid_length // 2, _space_width, _grid_height // 2)
            self.gl_stackDiagram.addItem(grid_back)
        elif mode == '更新':
            p_array = gv.getValue('drawStackPositionArray')
            size_array = gv.getValue('drawStackSizeArray')
            if p_array is not None:
                _mesh = createVoxelMesh(pos=p_array, size=size_array, edge=True,
                                        facecolor=(50 / 255, 205 / 255, 50 / 255, 1))
                self.gl_stackDiagram.addItem(_mesh)
        elif mode == '添加':
            p_array = gv.getValue('drawPlanPositionArray')
            size_array = gv.getValue('drawPlanSizeArray')
            _mesh = createVoxelMesh(pos=p_array, size=size_array, edge=True, facecolor=(255, 255, 0, 1))
            self.gl_stackDiagram.addItem(_mesh)

    def closeEvent(self, event) -> None:
        """
        重写closeEvent方法，实现MainWindow窗体关闭时执行一些代码
            :param event: close()触发的事件
        """
        try:
            self.tcp_close()  # 尝试关闭TCP连接（针对没有启动自动码垛时关闭系统可能引发的异常）
        except Exception:
            pass
        finally:
            self.saveLog(False)  # 将日志写入文件
            self.timer.stop()

    def cameraHandle(self, camera_type: str, is_connected: bool):
        """
        相机连接事件处理方法
            :param camera_type: 相机类型即事件内容
            :param is_connected: 连接成功标识，True-成功；False-失败
            :return: 无
        """
        if 'kinect' in camera_type:  # kinect处理流程
            if is_connected:  # 如果连接成功
                self.lbl_baggageCamStatus.setStyleSheet(
                    'QLabel{color: green; font: normal normal 12pt \'Microsoft YaHei\';}')  # 设置为绿色
                self.lbl_baggageCamStatus.setText(gv.getValue('kinectSerial'))  # 显示序列号
                self.pt_logOutput.appendHtml(hypertextAssembly('行李相机连接成功！', 'green'))  # 显示连接成功信息
                gv.setValue('baggageCameraReady', True)  # 标记行李相机就绪
            else:  # 连接失败
                self.lbl_baggageCamStatus.setStyleSheet(
                    'QLabel{color: red; font: normal normal 12pt \'Microsoft YaHei\';}')  # 设置为红色
                self.lbl_baggageCamStatus.setText('未连接')  # 显示未连接标识
                _err_msg = camera_type.split('|')[1]  # 分离出报错信息
                if '连接失败，请检查后重启系统！' in _err_msg:  # 分色显示不同内容
                    self.pt_logOutput.appendHtml(hypertextAssembly(f'{_err_msg}', 'red'))  # 显示未连接标识
                else:
                    self.pt_logOutput.appendHtml(hypertextAssembly(f'{_err_msg}', 'orange'))  # 显示未连接标识
                gv.setValue('baggageCameraReady', False)  # 标记行李相机未就绪
                img_interrupt = cv2.imread('./UI/camera_interrupt.jpg')
                img_interrupt = cv2.cvtColor(img_interrupt, cv2.COLOR_BGR2RGB)
                gv.setValue('baggageImg', img_interrupt)

        if 'sick' in camera_type:  # sick相机处理流程
            if is_connected:  # 如果连接成功
                _ip = gv.getParam('stackCameraIP')
                _port = gv.getParam('stackCameraPort')  # 获取ip和端口
                self.lbl_stackCamStatus.setStyleSheet(
                    'QLabel{color: green; font: normal normal 12pt \'Microsoft YaHei\';}')  # 设置为绿色
                self.lbl_stackCamStatus.setText(f'{_ip}:{_port}')  # 显示IP和端口
                self.pt_logOutput.appendHtml(hypertextAssembly('垛型相机连接成功！', 'green'))  # 显示连接成功信息
                gv.setValue('stackCameraReady', True)  # 标记垛型相机就绪
            else:  # 连接失败
                self.lbl_stackCamStatus.setStyleSheet(
                    'QLabel{color: red; font: normal normal 12pt \'Microsoft YaHei\';}')  # 设置为红色
                self.lbl_stackCamStatus.setText('未连接')  # 显示未连接标识
                _err_msg = camera_type.split('|')[1]  # 分离出报错信息
                if '连接失败，请检查后重启系统！' in _err_msg:  # 分色显示不同内容
                    self.pt_logOutput.appendHtml(hypertextAssembly(f'{_err_msg}', 'red'))  # 显示未连接标识
                else:
                    self.pt_logOutput.appendHtml(hypertextAssembly(f'{_err_msg}', 'orange'))  # 显示未连接标识
                gv.setValue('stackCameraReady', False)  # 标记垛型相机未就绪
                img_interrupt = cv2.imread('./UI/camera_interrupt.jpg')
                img_interrupt = cv2.cvtColor(img_interrupt, cv2.COLOR_BGR2RGB)
                gv.setValue('stackImg', img_interrupt)

    def stackEventHandle(self, event_name, event_data, debug_flag):
        """
        码垛事件控制方法
            :param event_name: 事件名称
            :param event_data: 事件内容
            :param debug_flag: 调试模式标志
            :return: 无
        """
        if event_name == '行李到达':  # 行李到达事件，该事件在调试模式下不会触发
            #######
            # 重置拍摄完成信号、重置码垛规划完成信号

            #######
            gv.setValue('baggageStackingFlag', True)  # 声明行李正在处理中，阻止下一件行李到达影响码垛作业
            gv.setValue('stackCorrectFlag', False)  # 声明后续垛型修正的来源
            # 日志显示
            self.displayLog('新行李到达，开始采集')
            gv.setValue('baggageCaptureFlag', True)  # 修改公共变量通知kinect线程采集行李数据 相机拍照
            gv.setValue('saveBaggageFlag', True)  # 修改公共变量保存采集行李数据
            # gv.setValue('stackCaptureFlag', True)  # 修改公共变量通知sick相机线程采集垛型数据
            gv.setValue('baggageArrived', False)  # 复位行李到达信号
            # self.baggageClassify()
            # self.baggageMeasure()
        elif event_name == '行李拍照成功':  ### 忽略
            # gv.setValue('baggageCaptureFlag', False) # 复位KINECT线程信号

            if debug_flag:
                self.displayLog('【调试】行李数据采集成功')
            else:
                self.displayLog('行李数据采集成功，开始分类')
                # gv.setValue('baggageClassifyFlag', True)  # 修改公共变量通知分类线程开始行李分类
            # self.baggageMeasure()
        elif event_name == '行李保存成功':  # 保存行李数据
            self.displayLog(f'第一件已保存至：{event_data}')
            self.baggageMeasure()
            # self.stackEventHandle('行李到达', '', False)
            ######
            # 得到了信息，创造一个新的函数，得到码放信息，加一个标志
            # self.two_baggagesize.append(gv.getValue('currentBaggageLength'))
            # self.two_baggagesize.append(gv.getValue('currentBaggageWidth'))
            # self.two_baggagesize.append(gv.getValue('currentBaggageAngle'))

            ###############################################################################################

            # self.list.append(stack_planning.get_item(gv.getValue('currentBaggageWidth')))
            # print(self.list)
            # 第二件

            ###############################################################################################
            # self.list.append(stack_planning.get_item(gv.getValue('currentBaggageWidth')))
            # print(self.list)
        ######################################################################################################

        ######
        elif event_name == '第二件行李保存成功':  # 保存行李数据
            self.displayLog(f'第二件已保存至：{event_data}')
            self.baggageMeasure_2nd()
            self.two_baggagesize.append(gv.getValue('currentBagLength'))
            self.two_baggagesize.append(gv.getValue('currentBagWidth'))
            self.two_baggagesize.append(gv.getValue('currentBagAngle'))

            # self.list.append(stack_planning.get_item(gv.getValue('currentBagWidth')))
            # print(self.list)


        # elif event_name == '行李分类结束':
        #     if '错误' in event_data:  # 如果分类出现错误
        #         if debug_flag:  # 如果是调试模式
        #             self.displayLog(f'【调试】行李分类失败，{event_data}', '错误')
        #         else:  # 非调试模式
        #             self.displayLog(f'行李分类失败，已跳过分类步骤。{event_data}', '错误')
        #             # 不管结果如何都进行下一步
        #             # baggage_measure_thread = BaggageMeasure()  # 实例化行李测量线程
        #             # baggage_measure_thread.baggage_measure_done.connect(self.stackEventHandle)  # 绑定测量成功信号到本地
        #             # baggage_measure_thread.start()  # 开始测量
        #             # self.baggageMeasure()
        #     else:  # 分类正常
        #         if debug_flag:  # 如果是调试模式
        #             self.displayLog(f'【调试】行李分类成功，分类结果为：{event_data}')
        #         else:  # 非调试模式
        #             self.displayLog(f'行李分类成功，分类结果为：{event_data}')
        #             # 不管结果如何都进行下一步
        #             # baggage_measure_thread = BaggageMeasure()  # 实例化行李测量线程
        #             # baggage_measure_thread.baggage_measure_done.connect(self.stackEventHandle)  # 绑定测量成功信号到本地
        #             # baggage_measure_thread.start()  # 开始测量
        #             self.baggageMeasure()
        # ###################
        # ###################
        # elif event_name == '行李分类完成':
        #     if '错误' in event_data:  # 如果分类出现错误
        #         if debug_flag:  # 如果是调试模式
        #             self.displayLog(f'【调试】行李分类失败，{event_data}', '错误')
        #         else:  # 非调试模式
        #             self.displayLog(f'行李分类失败，已跳过分类步骤。{event_data}', '错误')
        #             # 不管结果如何都进行下一步
        #             # baggage_measure_thread = BaggageMeasure()  # 实例化行李测量线程
        #             # baggage_measure_thread.baggage_measure_done.connect(self.stackEventHandle)  # 绑定测量成功信号到本地
        #             # baggage_measure_thread.start()  # 开始测量
        #             # self.baggageMeasure()
        #     else:  # 分类正常
        #         if debug_flag:  # 如果是调试模式
        #             self.displayLog(f'【调试】行李分类成功，分类结果为：{event_data}')
        #         else:  # 非调试模式
        #             self.displayLog(f'行李分类成功，分类结果为：{event_data}')
        #             # 不管结果如何都进行下一步
        #             # self.baggage_measure_thread = BaggageMeasure()  # 实例化行李测量线程
        #             # self.baggage_measure_thread.baggage_measure_done.connect(self.stackEventHandle)  # 绑定测量成功信号到本地
        #             # self.baggage_measure_thread.start()  # 开始测量
        #             self.baggageMeasure()
        elif event_name == '行李一次测量结束':
            pass

        #########################
        ######################### 第一次测量
        elif event_name == '行李测量结束':
            if '错误' in event_data:  # 如果测量出现错误
                if debug_flag:  # 如果是调试模式
                    self.displayLog(f'【调试】行李测量失败，{event_data}', '错误')
                else:
                    self.displayLog(f'行李测量失败，{event_data}', '错误')
                    self.displayLog(f'自动码垛中止，请检查后重新启动码垛作业', '错误')
                    gv.setValue('baggageCaptureFlag', True)  # 修改公共变量通知kinect线程采集行李数据 相机拍照
                    gv.setValue('saveBaggageFlag', True)  # 修改公共变量保存采集行李数据
            else:  # 测量成功
                # 获取行李数据，直接转换为整数
                ###########
                ###########
                print("尺寸测量成功了，准备接取")

        elif event_name == '发送第一个箱子信息':

            self.displayLog("收到机器人指令，开始尺寸测量")
            # todo:__________________________________2024-7-4_______________________________
            width_list = gv.getValue('widthstack')
            _width = int(width_list.pop(0))
            print(_width)
            _length = int(gv.getValue('lengthstack').pop(0))
            _thick = int(gv.getValue('thickstack').pop(0))
            _angle = round(gv.getValue('anglestack').pop(0), 1)
            _pos_x = int(gv.getValue("centerXstack").pop(0) + gv.getParam('pickIncrementX'))
            _pos_y = int(gv.getValue("centerYstack").pop(0))
            _size1 = int(gv.getValue('size').pop(0))
            print(_size1)

            sum_baggage = int(gv.getValue('num_baggage') + 1)
            if sum_baggage > 10:
                sum_baggage = int(1)
                gv.setValue('num_baggage', sum_baggage)
                floor_baggage = int(gv.getValue('floor_baggage') + 1)
                gv.setValue('floor_baggage', floor_baggage)
            else:
                gv.setValue('num_baggage', sum_baggage)
                floor_baggage = int(gv.getValue('floor_baggage'))
            # todo:______________________________________________________________________________

            if debug_flag:  # 如果是调试模式
                # 显示在码垛数据区域
                self.le_baggageLength.setText(str(_length))
                self.le_baggageWidth.setText(str(_width))
                self.le_baggageThick.setText(str(_thick))
                self.le_baggageAngle.setText(str(_angle))
                self.le_pickX.setText(str(_pos_x))
                # 清空码垛数据区域中的位置数据
                self.le_positionX.setText('')
                self.le_positionY.setText('')
                self.le_positionZ.setText('')
                # 清空码垛数据区域中的接方式选项
                # self.rb_pickV.setChecked(False)
                # self.rb_pickH.setChecked(False)
                # 输出日志
                self.displayLog(f'【调试】行李测量成功')
                self.displayLog(f'长:{_length}；宽:{_width}，厚:{_thick}；转角:{_angle}；接位置:{_pos_x}')
            else:  # 如果不是调试模式
                # 输出日志
                self.le_baggageLength.setText(str(_length))
                self.le_baggageWidth.setText(str(_width))
                self.le_baggageThick.setText(str(_thick))
                self.le_baggageAngle.setText(str(_angle))
                self.le_pickX.setText(str(_pos_x))
                self.displayLog(f'行李测量成功:')
                self.displayLog(f'长:{_length}；宽:{_width}，厚:{_thick}；转角:{_angle}；接位置:{_pos_x}')
                # self.displayLog(f'开始垛型规划')
                # self.stack_plan_thread.start()
                # self.sendCustomData()
                # time.sleep(3)
                _stackcmd = f'START|0|{_pos_x}|{_pos_y}|0|{_length}|{_width}|{_thick}|{_angle}|{_size1}|{0}|{0}|{floor_baggage}|{sum_baggage}|END'
                self.sendSocketData(_stackcmd)
                # todo： ————————————————————————2024.6.28————————————————————————————
                gv.setValue('baggageStackingFlag', False)
                # # 接一个的时候，将运行标志调至否
                # if _jieyige == 1:
                #     gv.setValue('baggageStackingFlag', False)
                # else:
                #     pass
        # todo：——————————————————————————————————————————————————————————————————————————
        # _reply = QMessageBox.question(self, '指令发送二次确认', f'即将发送机器人动作指令：\n'
        #                                                 f'{_stackcmd}\n'
        #                                                 f'指令内容如下\n'
        #                                                 f'行李尺寸：{_length} * {_width} * {_thick}；行李角度{_angle}\n'
        #                                                 f'接位置：0；接方式：{0}；托盘接取姿态：竖 \n'
        #                                                 f'放位置：({_pos_x} , {_pos_y} , 0)\n'
        #                                                 f'调试模式下无超限检测功能，机器人将按指令进行动作，请确认机器人运行在手动非全速状态下。\n'
        #                                                 f'确定发送指令吗？', QMessageBox.Yes, QMessageBox.No)
        # if _reply == QMessageBox.Yes:
        #     self.sendSocketData(_stackcmd)
        # else:
        #     # _stackcmd = ''
        #     pass

        # print("向机器人发送指令")# 启动码垛规划线程
        # self.displayLog("向机器人发送指令")
        # gv.setValue('baggageCaptureFlag', False)  # 修改公共变量通知kinect线程采集行李数据
        # gv.setValue('saveBaggageFlag', False)  # 修改公共变量保存采集行李数据
        # gv.setValue('stackCaptureFlag', True)  # 修改公共变量通知sick相机线程采集垛型数据
        # gv.setValue('baggageArrived', True)  # 复位行李到达信号
        # gv.setValue('baggageStackingFlag', False)
        elif event_name == '行李第二次测量结束':
            if '错误' in event_data:  # 如果测量出现错误
                if debug_flag:  # 如果是调试模式
                    self.displayLog(f'【调试】行李测量失败，{event_data}', '错误')
                else:
                    self.displayLog(f'行李测量失败，{event_data}', '错误')
                    self.displayLog(f'自动码垛中止，请检查后重新启动码垛作业', '错误')
                    gv.setValue('第二个行李相机拍照标志', True)  # 修改公共变量通知kinect线程采集行李数据 相机拍照
                    gv.setValue('saveBaggageFlag_2nd', True)  # 修改公共变量保存采集行李数据
            else:  # 测量成功
                # 获取行李数据，直接转换为整数
                #########

                print("尺寸第二次测量成功了，测量结束")
                # todo:__________________________________6.21_______________________________
                width_list = gv.getValue('widthstack')
                # # print(width_list)
                _width = int(width_list.pop(0))
                # print(_width)
                #
                # # length_list = gv.getValue('lengthstack')
                # # print(width_list)
                _length = int(gv.getValue('lengthstack').pop(0))
                _thick = int(gv.getValue('thickstack').pop(0))
                _angle = round(gv.getValue('anglestack').pop(0), 1)
                _pos_x = int(gv.getValue("centerXstack").pop(0) + gv.getParam('pickIncrementX'))
                _pos_y = int(gv.getValue("centerYstack").pop(0))
                # # print(_width)
                _size1 = int(gv.getValue('size1'))
                _size2 = int(gv.getValue('size').pop(0))
                currentBagWidth = _width
                # todo:______________________________________________________________________________
                # currentBagWidth = int(gv.getValue('currentBagWidth'))
                if currentBagWidth > 300:
                    # gv.setValue('size_2', int(1))
                    # size_2 = gv.getValue('size_2')
                    print(f'第二个箱子的大小为 {_size2}')
                else:
                    print(f'第二个箱子的大小为 {_size1}')
                    # gv.setValue('size_2', int(0))
                self.displayLog("收到机器人指令，开始第二次尺寸测量")
                # _length = int(gv.getValue('currentBagLength'))
                print(f"{_length}")
                # _width = int(gv.getValue('currentBagWidth'))
                print(f"{_length}")
                # _thick = int(gv.getValue('currentBagThick'))
                # _angle = round(gv.getValue('currentBagAngle'), 1)
                # _pos_x = int(gv.getValue('currentBagCenterX') + gv.getParam('pickIncrementX'))
                # _pos_y = int(gv.getValue('currentBagCenterY'))

                # _size1 = int(gv.getValue("size_1"))
                # _size2 = int(gv.getValue('size_2'))
                _floor = int(gv.getValue('floor'))
                _box = int(gv.getValue('box'))

                if debug_flag:  # 如果是调试模式
                    # 显示在码垛数据区域
                    self.le_baggageLength.setText(str(_length))
                    self.le_baggageWidth.setText(str(_width))
                    self.le_baggageThick.setText(str(_thick))
                    self.le_baggageAngle.setText(str(_angle))
                    self.le_pickX.setText(str(_pos_x))
                    # 清空码垛数据区域中的位置数据
                    self.le_positionX.setText('')
                    self.le_positionY.setText('')
                    self.le_positionZ.setText('')
                    # 清空码垛数据区域中的接方式选项
                    # self.rb_pickV.setChecked(False)
                    # self.rb_pickH.setChecked(False)
                    # 输出日志
                    self.displayLog(f'【调试】行李测量成功')
                    self.displayLog(f'长:{_length}；宽:{_width}，厚:{_thick}；转角:{_angle}；接位置:{_pos_x}')
                else:  # 如果不是调试模式
                    # 输出日志
                    # _stackcmd = f'START|0|{_pos_x}|{_pos_y}|0|{_length}|{_width}|{_thick}|{_angle}|{_size_1st}|{_floor_1st}|{_boxNum_1st}|{_size_2nd}|{_floor_2nd}|{_boxNum_2nd}|END'
                    # self.displayLog(f'START|0|{_pos_x}|{_pos_y}|0|{_length}|{_width}|{_thick}|{_angle}|{_size_1st}|{_floor_1st}|{_boxNum_1st}|{_size_2nd}|{_floor_2nd}|{_boxNum_2nd}|END')
                    self.displayLog(f'行李测量成功:')
                    self.displayLog(f'长:{_length}；宽:{_width}，厚:{_thick}；转角:{_angle}；接位置:{_pos_x}')

                    _stackcmd = f'START|0|{_pos_x}|{_pos_y}|0|{_length}|{_width}|{_thick}|{_angle}|{_size1}|{_floor}|{_box}|{_size2}|0|0|END'
                    self.sendSocketData(_stackcmd)
                    # _reply = QMessageBox.question(self, '指令发送二次确认', f'即将发送机器人动作指令：\n'
                    #                                                 f'{_stackcmd}\n'
                    #                                                 f'指令内容如下\n'
                    #                                                 f'行李尺寸：{_length} * {_width} * {_thick}；行李角度{_angle}\n'
                    #                                                 f'接位置：0；接方式：0\n'
                    #                                                 f'放位置：({_pos_x} , {_pos_y} , 0)\n'
                    #                                                 f'调试模式下无超限检测功能，机器人将按指令进行动作，请确认机器人运行在手动非全速状态下。\n'
                    #                                                 f'确定发送指令吗？', QMessageBox.Yes, QMessageBox.No)
                    # if _reply == QMessageBox.Yes:
                    #     self.sendSocketData(_stackcmd)
                    # else:
                    #     # _stackcmd = ''
                    #     pass
                    # self.sendRandomStackData()

                    # print("向机器人发送指令")# 启动码垛规划线程
                    # self.displayLog("向机器人发送指令")
                    # gv.setValue('baggageCaptureFlag', False)  # 修改公共变量通知kinect线程采集行李数据
                    # gv.setValue('saveBaggageFlag', False)  # 修改公共变量保存采集行李数据
                    # gv.setValue('stackCaptureFlag', True)  # 修改公共变量通知sick相机线程采集垛型数据
                    # gv.setValue('baggageArrived', True)  # 复位行李到达信号
                    gv.setValue('baggageStackingFlag', False)

        elif event_name == '垛型拍照成功':
            self.stack_capture_success = True  # 垛型采集成功标志
            if debug_flag:
                self.displayLog(f'【调试】垛型数据采集成功')
            else:
                self.displayLog(f'垛型数据采集成功，开始解构')
                # 启动垛型解构线程
                print('垛型数据采集成功，开始解构')
                # self.stack_deconstruction_thread.start()
        elif event_name == '垛型保存成功':
            self.displayLog(f'【调试】已保存至：{event_data}')
        elif event_name == '垛型解构结束':
            if '错误' in event_data:
                if debug_flag:  # 如果是调试模式
                    self.displayLog(f'【调试】垛型解构测量失败，{event_data}', '错误')
                else:
                    self.displayLog(f'垛型解构测量失败，{event_data}', '错误')
                    self.displayLog(f'自动码垛中止，请检查后重新启动码垛作业', '错误')
            else:
                self.drawVirtualStack('重置')  # 重置虚拟垛型
                self.drawVirtualStack('更新')  # 更新虚拟垛型
                if gv.getValue('stackCorrectFlag'):  # 如果垛型解构由修正垛型流程申请，注意，仅在非调试模式中才会出现此情况
                    gv.setValue('stackCorrectFlag', False)  # 复位标志位
                    self.displayLog(f'垛型修正结束，等待新行李到达。。')
                    gv.setValue('baggageStackingFlag', False)  # 复位行李处理标志位
                else:  # 如果由行李到达流程申请
                    if debug_flag:
                        self.displayLog(f'【调试】垛型解构成功，虚拟垛型已更新')
                    else:
                        self.displayLog(f'垛型解构成功，开始规划码垛位置')
                        print('垛型解构成功')
                        # self.stack_plan_thread.start()  # 启动码垛规划线程
        elif event_name == '码垛规划结束':
            if event_data == '空间已满':  # 如果空间满
                if debug_flag:
                    self.displayLog('【调试】码垛规划结束：码垛空间已满，请清理后重试。')
                else:
                    self.displayLog('码垛规划结束：码垛空间已满，本轮码垛结束。', '成功')
            elif '错误' in event_data:  # 如果码垛出错
                print('aaa')
                if debug_flag:
                    self.displayLog(f'【调试】{event_data}')
                else:
                    self.displayLog(event_data)
            else:  # 正常结束码垛规划
                self.drawVirtualStack('重置')  # 复位虚拟垛型
                self.drawVirtualStack('更新')  # 重新加载垛型
                self.drawVirtualStack('添加')  # 在虚拟垛型上更新规划位置
                # 获取行李参数

                _pick_x = int(gv.getValue('currentBaggageCenterX') + gv.getParam('pickIncrementX'))
                _length = int(gv.getValue('currentBaggageLength'))
                _width = int(gv.getValue('currentBaggageWidth'))
                _thick = int(gv.getValue('currentBaggageThick'))
                _angle = round(gv.getValue('currentBaggageAngle'), 1)
                # 获取码垛参数
                _stack_mode = gv.getValue('stackMode')
                _pick_mode = gv.getValue('pickMode')
                # 换算Z
                _pos_z = gv.getValue('positionZ') * gv.getParam('scaleZ') + gv.getParam('putIncrementZ')
                # 换算XY，注意，坐标换算回来以后应减去间距的一半，以保证行李能居中放置
                if _pick_mode == '正常':
                    _pos_x = gv.getValue('positionY') * gv.getParam('scaleY') + _length + gv.getParam(
                        'baggageLengthIncrement') / 2 + gv.getParam('putIncrementX')
                    _pos_y = (gv.getParam('gridX') - gv.getValue('positionX')) * gv.getParam(
                        'scaleX') - _width - gv.getParam('baggageWidthIncrement') / 2 + gv.getParam('putIncrementY')
                else:
                    _pos_x = gv.getValue('positionY') * gv.getParam('scaleY') + _width + gv.getParam(
                        'baggageWidthIncrement') / 2 + gv.getParam('putIncrementX')
                    _pos_y = (gv.getParam('gridX') - gv.getValue('positionX')) * gv.getParam(
                        'scaleX') - _length - gv.getParam('baggageLengthIncrement') / 2 + gv.getParam('putIncrementY')
                # 取整
                _pos_x = int(_pos_x)
                _pos_y = int(_pos_y)
                _pos_z = int(_pos_z)
                _pick_mode_cmd = 1 if _pick_mode == '正常' else 2
                # self.rb_pickV.setChecked(True if _pick_mode == '正常' else False)
                # self.rb_pickH.setChecked(False if _pick_mode == '正常' else True)
                # 组装码垛字符串
                _stack_cmd = f'START|{_pick_x}|{_pos_x}|{_pos_y}|{_pos_z}|{_length}|{_width}|{_thick}|{_angle}|0|0|0|0|{_pick_mode_cmd}|END'
                if debug_flag:  # 如果是调试模式
                    # 显示在码垛数据区域
                    self.le_positionX.setText(str(round(_pos_x, 1)))
                    self.le_positionY.setText(str(round(_pos_y, 1)))
                    self.le_positionZ.setText(str(round(_pos_z, 1)))
                    # 显示在接收区
                    self.displayLog('【调试】规划成功，目标角点位置：')
                    self.displayLog(f'【调试】({_pos_x},{_pos_y},{_pos_z})，接方式:{_pick_mode}，放方式:{_stack_mode}')
                    # 显示要发送的指令
                    # self.te_sendData.setText(_stack_cmd)
                else:  # 如果不是调试模式
                    # 显示日志
                    # print("规划成功，规划结束了，等待下一条指令")
                    # self.displayLog(f'规划成功，位置:({_pos_x},{_pos_y},{_pos_z})，接方式:{_pick_mode}，放方式:{_stack_mode}')
                    self.displayLog(f'规划成功，位置:({_pos_x},{_pos_y},{_pos_z})')

                    # self.displayLog('下发码垛指令：')
                    # self.displayLog(f'{_stack_cmd}')

                    ###########
                    print("向机器人发送指令")  # 启动码垛规划线程
                    self.displayLog("向机器人发送指令")
                    # self.sendSocketData(_stack_cmd)

                    ##############
                    #### 规划完成 后 像机器人发送码放指令
                    # self.sendRandomStackData()
                    ###########
                    # self.sendCustomData()
                    print("错误发送")
                    # self.sendSocketData(_stack_cmd)  # 发送
                    self.displayLog("码垛成功，当前行李处理结束，等待下一件行李")
                    print("码垛成功，当前行李处理结束，等待下一件行李")

                _items = [f'{_length}*{_width}*{_thick}', f'{_pick_mode}', f'{_pick_x}', f'{_stack_mode}',
                          f'{_pos_x}*{_pos_y}*{_pos_z}']
                row = self.tw_stackOutput.rowCount()
                self.tw_stackOutput.insertRow(row)
                for i in range(len(_items)):
                    item = QTableWidgetItem(str(_items[i]))
                    if i == len(_items) - 1:
                        item.setForeground(QBrush(QColor(0, 0, 255)))
                    item.setTextAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
                    self.tw_stackOutput.setItem(row, i, item)
                print("规划成功，规划结束了，等待下一条指令")
                gv.setValue('baggageStackingFlag', False)

        elif event_name == '码垛成功':
            self.displayLog(f'码垛结束，开始修正垛型')
            gv.setValue('stackCorrectFlag', True)  # 修正垛型标志置位
            gv.setValue('stackCaptureFlag', True)  # 垛型采集标志置位
        else:
            pass

    def changePanel(self, idx):
        """
        切换选项卡的附加逻辑
            :param idx: 目标选项卡的序号
            :return: 无
        """
        if idx == 0:
            gv.setValue('debugFlag', True)  # 退出debug模式
        elif idx == 1:
            self.pt_receivedData.clear()  # 切换到调试面板时清空输出区域
            gv.setValue('debugFlag', True)  # 进入debug模式

    ##### 不正确 ，不能从这里传入信号
    # def toggleRunningStatus(self, event_name:str, event_data:str, debug_flag:bool , msg:str , msg_type:str):

    def toggleRunningStatus(self):
        """
        开始自动码垛按钮触发的逻辑
        ####  这句 debug_flag: 是否是调试模式 不要了
            :return: 无
        """
        gv.setValue('baggageStackingFlag', False)  # 复位行李处理标志
        gv.setValue('debugFlag', False)  # 只要点击就退出debug模式
        if gv.getValue('systemStartFlag'):  # 如果系统已经启动
            # self.tw_interfacePanel.setTabEnabled(1, True)
            # self.tw_interfacePanel.setTabEnabled(2, True)
            # self.tw_interfacePanel.setTabEnabled(3, True)
            self.pb_startStackCycling.setText('开始自动码垛')  # 修改按钮标签
            gv.setValue('systemStartFlag', False)  # 停止运行
            self.tcp_close()
            self.pt_logOutput.appendHtml(hypertextAssembly('码垛结束', 'blue'))
            # print(gv.getValue('debugFlag'))
        else:
            # self.tw_interfacePanel.setTabEnabled(1, False)
            # self.tw_interfacePanel.setTabEnabled(2, False)
            # self.tw_interfacePanel.setTabEnabled(3, False)
            self.pb_startStackCycling.setText('停止自动码垛')  # 修改按钮标签
            gv.setValue('systemStartFlag', True)  # 开始运行
            self.tcp_server_start(int(gv.getParam('hostPort')))  ###### 已经开始侦听了
            self.pt_logOutput.appendHtml(hypertextAssembly('开始码垛', 'blue'))

            # print(gv.getValue('debugFlag'))
            ####################################
            ####################################
            ####################################
            # self.onSocketDataReceived() ##### 这句不正确，不能定义方法信号
            self.socketMsg.connect(self.onSocketDataReceived)

            ##### 这句是正确的，点击自动码垛按钮后，开始侦听，收到消息后，开始执行self.onSocketDataReceived，同时又激活了
            ##### 测量完成 信号，到达 码垛控制事件，激活了 垛型规划。但是只能执行一次，可能需要重置某个信号

            # if msg == "ready":
            #     print(f"Message from client: {msg}")
            # self.baggageMeasure()
            # self.pt_logOutput.appendHtml(hypertextAssembly('111', 'blue'))
            # self.sendCustomData(data)

    def onSocketDataReceived(self, msg, msg_type):
        # data = self.socketMsg  # Convert received data to lowercase and remove any leading/trailing spaces
        # if msg_type == '错误':
        #     if msg[-1] == "ready":  # If the received data is "ready"msg[-1] == 'ARRIVED
        #         # self.stackEventHandle('行李到达', '', False)  # Trigger the baggage arrival logic
        if msg_type == '数据':
            # if msg == "ARRIVED":
            if msg[-1] == "A":

                print(f"Message from client: {msg}")
                # self.baggageMeasure()
                # self.sendCustomData(data)
                ############# 应当把第一个行李到达信号设置一下，开始循环
                # baggage_measure_thread = BaggageMeasure()  # 实例化行李测量线程
                # baggage_measure_thread.baggage_measure_done.connect(self.stackEventHandle)  # 绑定测量成功信号到本地
                # baggage_measure_thread.start()  # 开始测量
                #   self.baggageMeasure() ##### 收到就测量
                #   self.baggageClassify() ##### 收到就分类
                print("收到消息，开始准备测量了")
                self.displayLog("机器人已就位，开始自动码垛")
                gv.setValue('baggageArrived', True)
                # self.stackEventHandle('行李到达', '', True)
                if not gv.getValue('baggageStackingFlag'):  # 如果上一件行李处理结束
                    self.stackEventHandle('行李到达', '', True)
        # todo:__________________________________6.27_______________________________
        if msg_type == '数据':
            if msg[-1] == "B":
                print(f"Message from client: {msg}")
                self.stackEventHandle('发送第一个箱子信息', '', gv.getValue('debugFlag'))
                # self.baggageMeasure()

        if msg_type == '数据':
            if msg[-1] == "C":  ###第一件接到了小的，重新接取
                print(f"Message from client: {msg}")
                gv.setValue('baggageCaptureFlag', True)  # 修改公共变量通知kinect线程采集行李数据 相机拍照
                gv.setValue('saveBaggageFlag', True)  # 修改公共变量保存采集行李数据
                # self.baggageMeasure()

                # def  onSocketDataReceived_2nd (self, msg, msg_type ):
        if msg_type == '数据':
            if msg[-1] == "D":  ####接第二件箱子
                print(f"Message from client: {msg}")
                gv.setValue('第二个行李相机拍照标志', True)
                gv.setValue('saveBaggageFlag_2nd', True)
                # self.baggageMeasure()

    # todo:__________________________________------_______________________________
    # elif msg_type == '数据':  # 数据类型
    #     if msg[-1] == 'ARRIVED':  # 行李到达指定位置
    #         if not gv.getValue('baggageArrived'):  # 如果没有行李到达
    #             gv.setValue('baggageArrived', True)  # 将行李到达的消息装入变量
    #     elif msg[-1] == 'DONE':  # 行李码垛作业完成
    #         self.stackEventHandle('码垛成功', '', False)  # 触发码垛成功逻辑
    #     else:
    #         self.pt_logOutput.appendHtml(
    #             hypertextAssembly(f'来自{msg[0]}:{msg[1]}的未知类型消息：{msg[-1]}', 'orange'))  # 显示在运行日志，橙色
    #     self.pt_receivedData.appendHtml(
    #         hypertextAssembly(f'来自{msg[0]}:{msg[1]}的消息：{msg[-1]}', 'green'))  # 显示在调试日志，绿色

    def displayLog(self, msg: str, msg_type='信息'):
        """
        将消息显示在运行日志
            :param msg: 要显示的消息内容
            :param msg_type: 消息类型，见SocketHelper中的常量定义
            :return: 无
        """
        if msg_type == '信息':  # 信息类型
            self.pt_logOutput.appendHtml(hypertextAssembly(f'{msg}', 'blue'))  # 显示在运行日志，蓝色
            self.pt_receivedData.appendHtml(hypertextAssembly(f'{msg}', 'blue'))  # 显示在调试日志，蓝色
        elif msg_type == '成功':  # 发送成功类型
            self.pt_logOutput.appendHtml(hypertextAssembly(f'{msg}', 'green'))  # 显示在运行日志，绿色
            self.pt_receivedData.appendHtml(hypertextAssembly(f'{msg}', 'green'))  # 显示在调试日志，绿色
        elif msg_type == '错误':  # 错误类型
            self.pt_logOutput.appendHtml(hypertextAssembly(f'{msg}', 'red'))  # 显示在运行日志，红色
            self.pt_receivedData.appendHtml(hypertextAssembly(f'{msg}', 'red'))  # 显示在调试日志，红色
        elif msg_type == '数据':  # 数据类型
            if msg == 'ARRIVED':  # 行李到达指定位置
                if not gv.getValue('baggageArrived'):  # 如果没有行李到达
                    gv.setValue('baggageArrived', True)  # 将行李到达的消息装入变量
            elif msg[-1] == 'DONE':  # 行李码垛作业完成
                self.stackEventHandle('码垛成功', '', False)  # 触发码垛成功逻辑
            elif msg[-1] == str("Y"):
                if not gv.getValue('baggageArrived'):  # 如果没有行李到达
                    gv.setValue('baggageArrived', True)  # 将行李到达的消息装入变量
                # print(f"Message from client: {msg}")
                ##########
                self.displayLog('机器人准备就绪')
            else:
                self.pt_logOutput.appendHtml(
                    hypertextAssembly(f'来自{msg[0]}:{msg[1]}的未知类型消息：{msg[-1]}', 'orange'))  # 显示在运行日志，橙色
            self.pt_receivedData.appendHtml(
                hypertextAssembly(f'来自{msg[0]}:{msg[1]}的消息：{msg[-1]}', 'green'))  # 显示在调试日志，绿色

    def updateLogic(self):
        """
        后台刷新逻辑
            :return: 无
        """
        # 相机画面更新
        self.wg_stackCameraShow.setImage(gv.getValue('stackImg'))
        self.wg_baggageCameraShow.setImage(gv.getValue('baggageImg'))
        if gv.getValue('baggageArrived'):  # 如果有新行李到达
            if not gv.getValue('baggageStackingFlag'):  # 如果上一件行李处理结束   None 的 not 等于True
                self.stackEventHandle('行李到达', '', False)  # 触发行李到达逻辑
                # 注意，此处不复位行李到达信号，由stackEventHandle方法对行李到达信号进行复位

    def emergencyStop(self):

        """
        紧急停止
            :return:无
        """
        self.tw_interfacePanel.setTabEnabled(1, True)
        self.tw_interfacePanel.setTabEnabled(2, True)  # 重新使能各个面板
        self.pb_startStackCycling.setText('开始自动码垛')  # 修改按钮标签
        self.pt_logOutput.appendHtml(
            hypertextAssembly(f'紧急停止！', 'red'))  # 发出警告
        # TODO: 停止机器人和码垛规划线程的指令
        self.tcp_close()

    def saveLog(self, show_hint=True):
        """
        保存日志
            :return: 无
        """
        _log = self.pt_logOutput.toPlainText()  # 获取日志内容
        current_date = str(datetime.date.today())  # 获取当日日期
        if not os.path.exists('./log'):  # 检查log文件夹是否存在
            os.mkdir('./log')  # 如果不存在则创建
        with open(f'./log/{current_date}.log', 'a') as file:  # ”a"代表追加，如不存在则创建
            file.write(_log + '\n')  # 写入文件
        _log_list = []  # 定义一个空列表用于处理日志文件内容
        for i in open(f'./log/{current_date}.log', 'r'):  # 遍历每一行
            if i in _log_list:
                continue
            _log_list.append(i)  # 清除其中的重复内容
        with open(f'./log/{current_date}.log', 'w') as handle:
            handle.writelines(_log_list)  # 将去重后的内容写回日志文件
        if show_hint:  # 如果需要弹出提示
            msgBox = QMessageBox(QMessageBox.Information, '提示', f'日志已追加至./log/{current_date}.log')
            msgBox.setWindowIcon(self.info_icon)
            msgBox.exec_()

    def clearLog(self):
        """
        清空日志区域
            :return: 无
        """
        self.pt_logOutput.clear()

    def stackPointcloudCapture(self):
        """
        保存垛型点云
            :return: 无
        """
        gv.setValue('stackCaptureFlag', True)
        gv.setValue('saveStackFlag', True)

    def stackPlan(self):
        """
        垛型规划逻辑
            :return: 无
        """
        self.stack_plan_thread.start()  # 启动码垛规划线程

    def stackDeconstruction(self):
        """
        垛型点云解构
            :return: 无
        """
        # 启动垛型解构线程
        self.stack_deconstruction_thread.start()

    def stackPlanReset(self):
        """
        垛型规划
            :return:无
        """

    def baggagePointcloudCapture(self):
        """
        保存行李点云
            :return: 无
        """
        gv.setValue('baggageCaptureFlag', True)
        gv.setValue('saveBaggageFlag', True)

    def baggageClassify(self):
        """
        行李分类
            :return: 无
        """
        # self.baggage_classify_thread = Classification()  # 实例化行李测量线程
        # self.baggage_classify_thread.baggage_classify_done.connect(self.stackEventHandle)  # 绑定测量成功信号到本地
        # self.baggage_classify_thread.start()  # 开始测量
        # self.baggageMeasure()
        # time.sleep(0.5)
        # self.sendRandomStackData()
        pass

    def baggageMeasure(self):
        """
        行李测量逻辑
            :return: 无
        """
        self.baggage_measure_thread = BaggageMeasure()  # 实例化行李测量线程
        self.baggage_measure_thread.baggage_measure_done.connect(self.stackEventHandle)  # 绑定测量成功信号到本地
        self.baggage_measure_thread.start()  # 开始测量

    def baggageMeasure_2nd(self):
        self.baggage_measure_thread_2nd = BaggageMeasure_2nd()  # 实例化行李测量线程
        self.baggage_measure_thread_2nd.baggage_measure_done.connect(self.stackEventHandle)  # 绑定测量成功信号到本地
        self.baggage_measure_thread_2nd.start()  # 开始测量

    def clearStackData(self):
        self.le_baggageLength.setText('')
        self.le_baggageWidth.setText('')
        self.le_baggageThick.setText('')
        self.le_baggageAngle.setText('')
        self.le_pickX.setText('')
        self.le_positionX.setText('')
        self.le_positionY.setText('')
        self.le_positionZ.setText('')

    def sendRandomStackData(self):
        """
        发送随机生成的码垛数据到已连接的客户端 非空检测
            :return: 无
        """
        # _stackcmd = ''
        # 非空检测
        if self.le_baggageLength.text() == '':
            self.le_baggageLength.setText('0.0')
        if self.le_baggageWidth.text() == '':
            self.le_baggageWidth.setText('0.0')
        if self.le_baggageThick.text() == '':
            self.le_baggageThick.setText('0.0')
        if self.le_baggageAngle.text() == '':
            self.le_baggageAngle.setText('0.0')
        if self.le_pickX.text() == '':
            self.le_pickX.setText('0.0')
        if self.le_positionX.text() == '':
            self.le_positionX.setText('0.0')
        if self.le_positionY.text() == '':
            self.le_positionY.setText('0.0')
        if self.le_positionZ.text() == '':
            self.le_positionZ.setText('0.0')
        # 从lineEdit中取数据
        _pick_x = round(float(self.le_pickX.text()), 2)
        _pos_x = round(float(self.le_positionX.text()), 2)
        _pos_y = round(float(self.le_positionY.text()), 2)
        _pos_z = round(float(self.le_positionZ.text()), 2)
        _length = round(float(self.le_baggageLength.text()), 2)
        _width = round(float(self.le_baggageWidth.text()), 2)
        _thick = round(float(self.le_baggageThick.text()), 2)
        _angle = round(float(self.le_baggageAngle.text()), 2)
        # _pick_mode_cmd = '1' if self.rb_pickV.isChecked() else '2'
        _pick_mode_cmd = round(float(self.le_baggageAngle.text()), 2)
        # 组装指令字符串
        _stackcmd = f'START|{_pick_x}|{_pos_x}|{_pos_y}|{_pos_z}|{_length}|{_width}|{_thick}|{_angle}|0|0|0|0|{_pick_mode_cmd}|END'

        # 二次确认
        _reply = QMessageBox.question(self, '指令发送二次确认', f'即将发送机器人动作指令：\n'
                                                                f'{_stackcmd}\n'
                                                                f'指令内容如下\n'
                                                                f'行李尺寸：{_length} * {_width} * {_thick}；行李角度{_angle}\n'
                                                                f'接位置：{_pick_x}；接方式：{_pick_mode_cmd}\n'
                                                                f'放位置：({_pos_x} , {_pos_y} , {_pos_z})\n'
                                                                f'调试模式下无超限检测功能，机器人将按指令进行动作，请确认机器人运行在手动非全速状态下。\n'
                                                                f'确定发送指令吗？', QMessageBox.Yes, QMessageBox.No)
        if _reply == QMessageBox.Yes:
            self.sendSocketData(_stackcmd)
        else:
            # _stackcmd = ''
            pass

    def toggleSocketServer(self):
        """
        调试面板中的Socket服务启动/停止方法
            :return: 无
        """
        if self.socket_status:  # 如果服务器已经启动
            self.pb_toggleListen.setText('开始侦听')
            self.tw_interfacePanel.setTabEnabled(0, True)
            self.tw_interfacePanel.setTabEnabled(2, True)
            self.tw_interfacePanel.setTabEnabled(3, True)
            self.tcp_close()
        else:
            self.pb_toggleListen.setText('结束侦听')
            self.tw_interfacePanel.setTabEnabled(0, False)
            self.tw_interfacePanel.setTabEnabled(2, False)
            self.tw_interfacePanel.setTabEnabled(3, False)
            self.tcp_server_start(int(gv.getParam('hostPort')))
        self.socket_status = not self.socket_status  # 变更服务器状态

    def clearData(self, clear_type='send'):
        """
        清空调试区内容
            :param clear_type: 要清空的目标编号，send-发送区，rec-接收区
            :return: 无
        """
        if clear_type == 'send':
            self.te_sendData.clear()
        elif clear_type == 'rec':
            self.pt_receivedData.clear()

    def sendCustomData(self, data: str):
        """
        发送自定义数据到已连接的客户端
            :param data: 要发送的数据
            :return: 无
        """
        self.sendSocketData(data)

    def sendSocketData(self, data: str):
        """
        复用方法，向已连接的客户端发送消息
            :param data: 要发送的内容
            :return: 无
        """
        try:
            self.tcp_send(data)
        except Exception as ret:
            self.socketMsg.emit(f'发送失败: {ret}', self.ERROR)
            pass

    def settingsLogic(self):
        """
        解锁设置/保存设置按钮的业务逻辑
            :return: 无
        """
        _status = gv.getValue('lockStatus')  # 首先获取锁定状态
        if not _status:  # 如果是False，说明此时界面已经解锁，应执行save动作
            # TODO: 验证输入内容合法性，如不合法则pass
            self.saveSettings()
        # 切换控件锁定状态，注意该方法的参数中True为解锁，False为锁定，因此无需取反
        self.toggleWidgets(_status)  # <---注意锁定状态切换后，lockStatus变量由该方法负责翻转
        # 在解锁设置面板时其他面板应锁定，因此此处应该取反
        self.tw_interfacePanel.setTabEnabled(0, not _status)
        self.tw_interfacePanel.setTabEnabled(1, not _status)

    def loadDefault(self):
        """
        恢复默认按钮的业务逻辑，保存到公共变量中并显示
            :return: 无
        """
        self.loadFromFile(True)

    def saveAsDefault(self):
        """
        存为默认按钮的业务逻辑
            :return: 无
        """
        self.saveToFile(True)

    def refreshIPList(self):
        """
        刷新IP列表内容
            :return: 无
        """
        ip_list = get_host_ip()
        current_ip = self.cb_hostIP.currentText()
        self.cb_hostIP.clear()
        self.cb_hostIP.addItems(ip_list)
        if current_ip in ip_list:
            self.cb_hostIP.setCurrentIndex(ip_list.index(current_ip))
        else:
            self.cb_hostIP.setCurrentIndex(0)

    def loadStackModel(self):
        """
        选择码垛模型
            :return: 无
        """
        import tkinter as tk
        from tkinter import filedialog
        # 使用tkinter的filedialog选择文件，此处不能使用QFileDialog，否则会引发死循环bug
        root = tk.Tk()
        root.withdraw()
        path = filedialog.askopenfilename(initialdir='./model', title='请选择模型文件', filetypes=[('pt模型', '*.pt')])
        gv.setParam('stackModelPath', path)
        file_name = os.path.basename(path)
        self.lbl_stackModelName.setText(file_name)

    def loadClassifyModel(self):
        """
        选择分类模型
            :return: 无
        """
        import tkinter as tk
        from tkinter import filedialog
        # 使用tkinter的filedialog选择文件，此处不能使用QFileDialog，否则会引发死循环bug
        root = tk.Tk()
        root.withdraw()
        path = filedialog.askopenfilename(initialdir='./model', title='请选择模型文件',
                                          filetypes=[('onnx模型', '*.onnx')])
        gv.setParam('classifyModelPath', path)
        file_name = os.path.basename(path)
        self.lbl_stackModelName.setText(file_name)

    # 控件重载和定义
    # 注意：以下内容直至模块尾部包含了大量的控件属性定义，可通过对控件进行重载的方式来去除不必要的代码，如无修改必要，建议将其折叠
    # 但为保证多端一致性，尚未使用此方案，如有需求，可对LineEdit控件进行重载，添加Validator和defaultValue等属性
    def toggleWidgets(self, status=False):
        """
        切换设置界面的使能状态
            :param status: 使能状态；True-解锁，False-锁定，默认为False
            :return: 无
        """
        # 保存和加载按钮
        self.pb_loadStackDefault.setEnabled(status)
        self.pb_saveStackAsDefault.setEnabled(status)
        self.pb_loadAlgorithmDefault.setEnabled(status)
        self.pb_saveAlgorithmAsDefault.setEnabled(status)
        # 码垛设置面板
        self.pb_unlockStackSettings.setText('保存设置' if status else '解锁设置')
        self.pb_unlockAlgorithmSettings.setText('保存设置' if status else '解锁设置')
        self.gb_baggageModify.setEnabled(status)  # 行李微调
        self.gb_stackModify.setEnabled(status)  # 垛型微调
        self.gb_spaceModify.setEnabled(status)  # 码垛空间设置
        self.gb_deviceInfo.setEnabled(status)  # 设备信息
        # 算法设置面板
        self.gb_stackAlgorithmModify.setEnabled(status)  # 码垛算法设置
        self.gb_classificationAlgorithmModify.setEnabled(status)  # 分类算法设置
        gv.setValue('lockStatus', not status)  # 设置完成后取反

    def loadSettings(self):
        """
        从配置文件中载入设置参数的业务逻辑，仅在窗体初始化时才调用
            :return: 无
        """
        self.loadFromFile(False)

    def saveSettings(self):
        """
        保存设置内容到公共变量，同时保存到Settings.cfg文件
            :return: 无
        """
        self.saveToFile(False)

    def loadFromFile(self, isdefault: bool):
        """
        复用方法，读取配置文件，显示在对应位置并写入公共变量
            :param isdefault: 配置文件位置，True-默认配置，False-当前配置
            :return: 无
        """
        # 仅当从默认配置读取时才执行加载，从设置配置读取时无需加载，因为主程序的init方法中已将配置加载到了globalvalue中
        if isdefault:
            gv.loadFromFile(isdefault)
        # 显示配置
        self.showFromGlobalValue()
        self.lbl_stackModelName.setText(os.path.basename(str(gv.getParam('stackModelPath'))))
        self.lbl_classifyModelName.setText(os.path.basename(str(gv.getParam('stackModelPath'))))
        self.rb_modelDiscrete.setChecked(not bool(gv.getParam('modelContinuous')))
        self.rb_modelContinuous.setChecked(bool(gv.getParam('modelContinuous')))
        # 显示可用IP
        ip_list = get_host_ip()
        saved_ip = gv.getParam('hostIP')
        if saved_ip in ip_list:
            self.cb_hostIP.setCurrentIndex(ip_list.index(saved_ip))
        else:
            self.cb_hostIP.setCurrentIndex(0)

    def saveToFile(self, isdefault: bool):
        """
        复用方法，将设置界面的内容写入公共变量并保存
            :param isdefault: 写入位置，True-默认配置，False-当前配置
            :return: 无
        """
        # 写入公共变量，遇空则置默认值
        # 码垛设置
        # --行李点云预处理
        gv.setParam('baggageAlpha', float(self.le_baggageAlpha.text() if self.le_baggageAlpha.text() != '' else 0))
        gv.setParam('baggageBeta', float(self.le_baggageBeta.text() if self.le_baggageBeta.text() != '' else 0))
        gv.setParam('baggageGamma', float(self.le_baggageGamma.text() if self.le_baggageGamma.text() != '' else 0))
        gv.setParam('baggageXmin', float(self.le_baggageXmin.text() if self.le_baggageXmin.text() != '' else 0))
        gv.setParam('baggageYmin', float(self.le_baggageYmin.text() if self.le_baggageYmin.text() != '' else 0))
        gv.setParam('baggageZmin', float(self.le_baggageZmin.text() if self.le_baggageZmin.text() != '' else 0))
        gv.setParam('baggageXmax', float(self.le_baggageXmax.text() if self.le_baggageXmax.text() != '' else 0))
        gv.setParam('baggageYmax', float(self.le_baggageYmax.text() if self.le_baggageYmax.text() != '' else 0))
        gv.setParam('baggageZmax', float(self.le_baggageZmax.text() if self.le_baggageZmax.text() != '' else 0))
        gv.setParam('baggageMinPts',
                    int(self.le_baggageMinPts.text() if self.le_baggageMinPts.text() != '' else 0))
        gv.setParam('baggageRadius',
                    float(self.le_baggageRadius.text() if self.le_baggageRadius.text() != '' else 0))
        gv.setParam('baggageZOffset',
                    float(self.le_baggageZOffset.text() if self.le_baggageZOffset.text() != '' else 0))
        gv.setParam('baggageEps', float(self.le_baggageEps.text() if self.le_baggageEps.text() != '' else 0))
        gv.setParam('baggageStd', float(self.le_baggageStd.text() if self.le_baggageStd.text() != '' else 0))
        gv.setParam('baggagePts', int(self.le_baggagePts.text() if self.le_baggagePts.text() != '' else 0))
        # --垛型点云预处理
        gv.setParam('stackAlpha', float(self.le_stackAlpha.text() if self.le_stackAlpha.text() != '' else 0))
        gv.setParam('stackBeta', float(self.le_stackBeta.text() if self.le_stackBeta.text() != '' else 0))
        gv.setParam('stackGamma', float(self.le_stackGamma.text() if self.le_stackGamma.text() != '' else 0))
        gv.setParam('stackXmin', float(self.le_stackXmin.text() if self.le_stackXmin.text() != '' else 0))
        gv.setParam('stackYmin', float(self.le_stackYmin.text() if self.le_stackYmin.text() != '' else 0))
        gv.setParam('stackZmin', float(self.le_stackZmin.text() if self.le_stackZmin.text() != '' else 0))
        gv.setParam('stackXmax', float(self.le_stackXmax.text() if self.le_stackXmax.text() != '' else 0))
        gv.setParam('stackYmax', float(self.le_stackYmax.text() if self.le_stackYmax.text() != '' else 0))
        gv.setParam('stackZmax', float(self.le_stackZmax.text() if self.le_stackZmax.text() != '' else 0))
        gv.setParam('stackMinPts', int(self.le_stackMinPts.text() if self.le_stackMinPts.text() != '' else 0))
        gv.setParam('stackRadius', float(self.le_stackRadius.text() if self.le_stackRadius.text() != '' else 0))
        gv.setParam('stackZOffset', float(self.le_stackZOffset.text() if self.le_stackZOffset.text() != '' else 0))
        gv.setParam('stackEps', float(self.le_stackEps.text() if self.le_stackEps.text() != '' else 0))
        gv.setParam('stackStd', float(self.le_stackStd.text() if self.le_stackStd.text() != '' else 0))
        gv.setParam('stackPts', int(self.le_stackPts.text() if self.le_stackPts.text() != '' else 0))
        # --尺寸映射参数
        gv.setParam('pickIncrementX',
                    float(self.le_pickIncrementX.text() if self.le_pickIncrementX.text() != '' else 0))
        gv.setParam('putIncrementX',
                    float(self.le_putIncrementX.text() if self.le_putIncrementX.text() != '' else 0))
        gv.setParam('putIncrementY',
                    float(self.le_putIncrementY.text() if self.le_putIncrementY.text() != '' else 0))
        gv.setParam('putIncrementY', float(self.le_putIncrementY.text() if self.le_putIncrementY.text() != '' else 0))
        gv.setParam('putIncrementZ', float(self.le_putIncrementZ.text() if self.le_putIncrementZ.text() != '' else 0))
        # --设备信息
        gv.setParam('hostIP', str(self.cb_hostIP.currentText()))
        gv.setParam('hostPort', int(self.le_hostPort.text() if self.le_hostPort.text() != '' else 9000))
        gv.setParam('stackCameraIP', str(self.le_stackCameraIP.text()))
        gv.setParam('stackCameraPort', int(self.le_stackCameraPort.text() if self.le_hostPort.text() != '' else 2114))
        # 算法设置-
        # --码垛算法设置
        gv.setParam('modelContinuous', True if self.rb_modelContinuous.isChecked() else False)
        gv.setParam('gridX', int(self.le_gridX.text() if self.le_gridX.text() != '' else 288))
        gv.setParam('gridY', int(self.le_gridY.text() if self.le_gridY.text() != '' else 147))
        gv.setParam('gridZ', int(self.le_gridZ.text() if self.le_gridZ.text() != '' else 55))
        gv.setParam('scaleX', float(self.le_scaleX.text() if self.le_scaleX.text() != '' else 10.0))
        gv.setParam('scaleY', float(self.le_scaleY.text() if self.le_scaleY.text() != '' else 10.0))
        gv.setParam('scaleZ', float(self.le_scaleZ.text() if self.le_scaleZ.text() != '' else 10.0))
        gv.setParam('baggageLengthMin',
                    int(self.le_baggageLengthMin.text() if self.le_baggageLengthMin.text() != '' else 0))
        gv.setParam('baggageLengthMax',
                    int(self.le_baggageLengthMax.text() if self.le_baggageLengthMax.text() != '' else 0))
        gv.setParam('baggageLengthIncrement',
                    float(self.le_baggageLengthIncrement.text() if self.le_baggageLengthIncrement.text() != '' else 0))
        gv.setParam('baggageWidthMin',
                    int(self.le_baggageWidthMin.text() if self.le_baggageWidthMin.text() != '' else 0))
        gv.setParam('baggageWidthMax',
                    int(self.le_baggageWidthMax.text() if self.le_baggageWidthMax.text() != '' else 0))
        gv.setParam('baggageWidthIncrement',
                    float(self.le_baggageWidthIncrement.text() if self.le_baggageWidthIncrement.text() != '' else 0))
        gv.setParam('baggageThickMin',
                    int(self.le_baggageThickMin.text() if self.le_baggageThickMin.text() != '' else 0))
        gv.setParam('baggageThickMax',
                    int(self.le_baggageThickMax.text() if self.le_baggageThickMax.text() != '' else 0))
        gv.setParam('baggageThickIncrement',
                    float(self.le_baggageThickIncrement.text() if self.le_baggageThickIncrement.text() != '' else 0))
        gv.setParam('stackLengthIncrement',
                    float(self.le_stackLengthIncrement.text() if self.le_stackLengthIncrement.text() != '' else 0))
        gv.setParam('stackWidthIncrement',
                    float(self.le_stackWidthIncrement.text() if self.le_stackWidthIncrement.text() != '' else 0))
        gv.setParam('stackThickIncrement',
                    float(self.le_stackThickIncrement.text() if self.le_stackThickIncrement.text() != '' else 0))
        gv.setParam('leafNode', int(self.le_leafNode.text() if self.le_leafNode.text() != '' else 80))
        gv.setParam('internalNode', int(self.le_internalNode.text() if self.le_internalNode.text() != '' else 50))
        # --分类算法设置
        gv.setParam('baggageImgCropXmin',
                    float(self.le_baggageImgCropXmin.text() if self.le_baggageImgCropXmin.text() != '' else 0))
        gv.setParam('baggageImgCropXmax',
                    float(self.le_baggageImgCropXmax.text() if self.le_baggageImgCropXmax.text() != '' else 0))
        gv.setParam('baggageImgCropYmin',
                    float(self.le_baggageImgCropYmin.text() if self.le_baggageImgCropYmin.text() != '' else 0))
        gv.setParam('baggageImgCropYmax',
                    float(self.le_baggageImgCropYmax.text() if self.le_baggageImgCropYmax.text() != '' else 0))
        gv.setParam('baggageImageSize',
                    int(self.le_baggageImageSize.text() if self.le_baggageImageSize.text() != '' else 224))
        # 保存到配置文件
        gv.saveToFile(isdefault)
        # 回显
        self.showFromGlobalValue()

    def showFromGlobalValue(self):
        """
        复用方法，从公共变量读取所有参数写入回LineEdit，实现回显，因为可能存在lineEdit为空的情况，该操作使其与gv内容同步
            :return: 无
        """
        # 码垛设置
        # --行李点云预处理
        self.le_baggageAlpha.setText(str(gv.getParam('baggageAlpha')))
        self.le_baggageBeta.setText(str(gv.getParam('baggageBeta')))
        self.le_baggageGamma.setText(str(gv.getParam('baggageGamma')))
        self.le_baggageXmin.setText(str(gv.getParam('baggageXmin')))
        self.le_baggageYmin.setText(str(gv.getParam('baggageYmin')))
        self.le_baggageZmin.setText(str(gv.getParam('baggageZmin')))
        self.le_baggageXmax.setText(str(gv.getParam('baggageXmax')))
        self.le_baggageYmax.setText(str(gv.getParam('baggageYmax')))
        self.le_baggageZmax.setText(str(gv.getParam('baggageZmax')))
        self.le_baggageMinPts.setText(str(gv.getParam('baggageMinPts')))
        self.le_baggageRadius.setText(str(gv.getParam('baggageRadius')))
        self.le_baggageZOffset.setText(str(gv.getParam('baggageZOffset')))
        self.le_baggageEps.setText(str(gv.getParam('baggageEps')))
        self.le_baggageStd.setText(str(gv.getParam('baggageStd')))
        self.le_baggagePts.setText(str(gv.getParam('baggagePts')))
        # --垛型点云预处理
        self.le_stackAlpha.setText(str(gv.getParam('stackAlpha')))
        self.le_stackBeta.setText(str(gv.getParam('stackBeta')))
        self.le_stackGamma.setText(str(gv.getParam('stackGamma')))
        self.le_stackXmin.setText(str(gv.getParam('stackXmin')))
        self.le_stackYmin.setText(str(gv.getParam('stackYmin')))
        self.le_stackZmin.setText(str(gv.getParam('stackZmin')))
        self.le_stackXmax.setText(str(gv.getParam('stackXmax')))
        self.le_stackYmax.setText(str(gv.getParam('stackYmax')))
        self.le_stackZmax.setText(str(gv.getParam('stackZmax')))
        self.le_stackMinPts.setText(str(gv.getParam('stackMinPts')))
        self.le_stackRadius.setText(str(gv.getParam('stackRadius')))
        self.le_stackZOffset.setText(str(gv.getParam('stackZOffset')))
        self.le_stackEps.setText(str(gv.getParam('stackEps')))
        self.le_stackStd.setText(str(gv.getParam('stackStd')))
        self.le_stackPts.setText(str(gv.getParam('stackPts')))
        # --尺寸映射参数
        self.le_pickIncrementX.setText(str(gv.getParam('pickIncrementX')))
        self.le_putIncrementX.setText(str(gv.getParam('putIncrementX')))
        self.le_putIncrementY.setText(str(gv.getParam('putIncrementY')))
        self.le_putIncrementZ.setText(str(gv.getParam('putIncrementZ')))
        # --设备信息
        self.le_hostPort.setText(str(gv.getParam('hostPort')))
        self.le_stackCameraIP.setText(str(gv.getParam('stackCameraIP')))
        self.le_stackCameraPort.setText(str(gv.getParam('stackCameraPort')))
        # 算法设置
        # --码垛算法设置
        self.le_gridX.setText(str(gv.getParam('gridX')))
        self.le_gridY.setText(str(gv.getParam('gridY')))
        self.le_gridZ.setText(str(gv.getParam('gridZ')))
        self.le_scaleX.setText(str(gv.getParam('scaleX')))
        self.le_scaleY.setText(str(gv.getParam('scaleY')))
        self.le_scaleZ.setText(str(gv.getParam('scaleZ')))
        self.le_baggageLengthMin.setText(str(gv.getParam('baggageLengthMin')))
        self.le_baggageLengthMax.setText(str(gv.getParam('baggageLengthMax')))
        self.le_baggageLengthIncrement.setText(str(gv.getParam('baggageLengthIncrement')))
        self.le_baggageWidthMin.setText(str(gv.getParam('baggageWidthMin')))
        self.le_baggageWidthMax.setText(str(gv.getParam('baggageWidthMax')))
        self.le_baggageWidthIncrement.setText(str(gv.getParam('baggageWidthIncrement')))
        self.le_baggageThickMin.setText(str(gv.getParam('baggageThickMin')))
        self.le_baggageThickMax.setText(str(gv.getParam('baggageThickMax')))
        self.le_baggageThickIncrement.setText(str(gv.getParam('baggageThickIncrement')))
        self.le_stackLengthIncrement.setText(str(gv.getParam('stackLengthIncrement')))
        self.le_stackWidthIncrement.setText(str(gv.getParam('stackWidthIncrement')))
        self.le_stackThickIncrement.setText(str(gv.getParam('stackThickIncrement')))
        self.le_leafNode.setText(str(gv.getParam('leafNode')))
        self.le_internalNode.setText(str(gv.getParam('internalNode')))
        # --分类算法设置
        self.le_baggageImgCropXmin.setText(str(gv.getParam('baggageImgCropXmin')))
        self.le_baggageImgCropXmax.setText(str(gv.getParam('baggageImgCropXmax')))
        self.le_baggageImgCropYmin.setText(str(gv.getParam('baggageImgCropYmin')))
        self.le_baggageImgCropYmax.setText(str(gv.getParam('baggageImgCropYmax')))
        self.le_baggageImageSize.setText(str(gv.getParam('baggageImageSize')))

    def validatorInit(self):
        """
        设置每个输入框的输入格式，只在界面初始化时执行一次
            :return: 无
        """
        # 调试面板: 码垛数据
        self.le_baggageLength.setValidator(self.validator_float_1)
        self.le_baggageWidth.setValidator(self.validator_float_1)
        self.le_baggageThick.setValidator(self.validator_float_1)
        self.le_baggageAngle.setValidator(self.validator_angle_90_1)
        self.le_pickX.setValidator(self.validator_float_1)
        self.le_positionX.setValidator(self.validator_float_1)
        self.le_positionY.setValidator(self.validator_float_1)
        self.le_positionZ.setValidator(self.validator_float_1)
        # 码垛设置
        # --行李点云预处理
        self.le_baggageAlpha.setValidator(self.validator_angle_180_1)
        self.le_baggageBeta.setValidator(self.validator_angle_180_1)
        self.le_baggageGamma.setValidator(self.validator_angle_180_1)
        self.le_baggageXmin.setValidator(self.validator_float_1)
        self.le_baggageYmin.setValidator(self.validator_float_1)
        self.le_baggageZmin.setValidator(self.validator_float_1)
        self.le_baggageXmax.setValidator(self.validator_float_1)
        self.le_baggageYmax.setValidator(self.validator_float_1)
        self.le_baggageZmax.setValidator(self.validator_float_1)
        self.le_baggageMinPts.setValidator(QtGui.QIntValidator(0, 9999))
        self.le_baggageRadius.setValidator(self.validator_float_3)
        self.le_baggageZOffset.setValidator(self.validator_float_1)
        self.le_baggageEps.setValidator(self.validator_float_3)
        self.le_baggageStd.setValidator(self.validator_float_3)
        self.le_baggagePts.setValidator(QtGui.QIntValidator())
        # --垛型点云预处理
        self.le_stackAlpha.setValidator(self.validator_angle_180_1)
        self.le_stackBeta.setValidator(self.validator_angle_180_1)
        self.le_stackGamma.setValidator(self.validator_angle_180_1)
        self.le_stackXmin.setValidator(self.validator_float_1)
        self.le_stackYmin.setValidator(self.validator_float_1)
        self.le_stackZmin.setValidator(self.validator_float_1)
        self.le_stackXmax.setValidator(self.validator_float_1)
        self.le_stackYmax.setValidator(self.validator_float_1)
        self.le_stackZmax.setValidator(self.validator_float_1)
        self.le_stackMinPts.setValidator(QtGui.QIntValidator(0, 9999))
        self.le_stackRadius.setValidator(self.validator_float_3)
        self.le_stackZOffset.setValidator(self.validator_float_1)
        self.le_stackEps.setValidator(self.validator_float_3)
        self.le_stackStd.setValidator(self.validator_float_3)
        self.le_stackPts.setValidator(QtGui.QIntValidator())
        # --尺寸映射参数
        self.le_pickIncrementX.setValidator(self.validator_float_3)
        self.le_putIncrementX.setValidator(self.validator_float_3)
        self.le_putIncrementY.setValidator(self.validator_float_3)
        self.le_putIncrementZ.setValidator(self.validator_float_3)
        # --设备信息
        self.le_hostPort.setValidator(QtGui.QIntValidator(0, 65535))
        self.le_stackCameraIP.setValidator(self.validator_ip_addr)
        self.le_stackCameraPort.setValidator(QtGui.QIntValidator(0, 65535))
        # 算法设置
        # --码垛算法设置
        self.le_gridX.setValidator(QtGui.QIntValidator())
        self.le_gridY.setValidator(QtGui.QIntValidator())
        self.le_gridZ.setValidator(QtGui.QIntValidator())
        self.le_scaleX.setValidator(self.validator_float_3)
        self.le_scaleY.setValidator(self.validator_float_3)
        self.le_scaleZ.setValidator(self.validator_float_3)
        self.le_baggageLengthMin.setValidator(QtGui.QIntValidator())
        self.le_baggageLengthMax.setValidator(QtGui.QIntValidator())
        self.le_baggageLengthIncrement.setValidator(self.validator_float_3)
        self.le_baggageWidthMin.setValidator(QtGui.QIntValidator())
        self.le_baggageWidthMax.setValidator(QtGui.QIntValidator())
        self.le_baggageWidthIncrement.setValidator(self.validator_float_3)
        self.le_baggageThickMin.setValidator(QtGui.QIntValidator())
        self.le_baggageThickMax.setValidator(QtGui.QIntValidator())
        self.le_baggageThickIncrement.setValidator(self.validator_float_3)
        self.le_stackLengthIncrement.setValidator(self.validator_float_3)
        self.le_stackWidthIncrement.setValidator(self.validator_float_3)
        self.le_stackThickIncrement.setValidator(self.validator_float_3)
        # --分类算法设置
        self.le_baggageImgCropXmin.setValidator(self.validator_percent_2)
        self.le_baggageImgCropXmax.setValidator(self.validator_percent_2)
        self.le_baggageImgCropYmin.setValidator(self.validator_percent_2)
        self.le_baggageImgCropYmax.setValidator(self.validator_percent_2)
        self.le_baggageImageSize.setValidator(QtGui.QIntValidator())


def hypertextAssembly(data, color):
    """
    静态方法，组装用于显示的超文本
        :param data: 要显示的数据
        :param color: 颜色，要求符合html颜色规范
        :return: html超文本字符串
    """
    dt = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]  # 获取时间和日期
    return f'[{dt}]<font color="{color}">{data}</font>\n'  # 组装数据和对应的颜色为html文本


def openFolder():
    """
    静态方法，打开数据目录
        :return: 无
    """
    base_path = os.getcwd()
    os.startfile(os.path.join(base_path, 'data'))
