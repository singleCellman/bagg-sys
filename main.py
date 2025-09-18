# _*_ coding:utf-8 _*_
"""
@File       : main.py
@Project    : baggage-system-pyqt-rebuild
@Author     : DaveHao
@Contact_1  : dave@fcsoft.cc
@Software   : PyCharm
@License    : MulanPSL2
@Description: 主程序入口，调用窗体模型并实例化后给出启动命令
"""

"""

# code is far away from bugs with the god animal protecting
    I love animals. They taste delicious.
             ┏┓       ┏┓
            ┏┛┻━━━━━━━┛┻┓
            ┃     ☃     ┃
            ┃  ┳┛   ┗┳  ┃
            ┃     ┻     ┃
            ┗━┓       ┏━┛
              ┃       ┗━━━┓
              ┃  神兽保佑    ┣┓
              ┃　 永无BUG   ┏┛
              ┗┓┓┏━  ┳┓┏┛
               ┃┫┫   ┃┫┫
               ┗┻┛   ┗┻┛
"""


import cv2
import sys
import global_value as gv
import numpy as np
from PyQt5.QtWidgets import QApplication
from main_window_overload import MainWindowFrame


# 实例化窗体前应进行的业务初始化逻辑
def init():
    gv.init()  # 初始化全局变量
    gv.loadFromFile(False)  # 轻加载配置文件
    gv.setValue('debugFlag', True)
    gv.setValue('lockStatus', False)  # 设定设置面板的默认锁定状态为False解锁，在初始化界面中进行锁定后才变为True锁定
    gv.setValue('baggageCaptureFlag', False)
    gv.setValue('baggagePointcloud', None)
    gv.setValue('stackLCaptureFlag', False)
    gv.setValue('stackRCaptureFlag', False)
    gv.setValue('stackLPointcloud', None)
    gv.setValue('stackRPointcloud', None)
    gv.setValue('saveBaggageFlag', False)
    gv.setValue('saveStackFlag', False)
    img_wait = cv2.imread('./UI/camera_wait.jpg')
    gv.setValue('baggageImg', img_wait)
    gv.setValue('stackImg', img_wait)
    gv.setValue("boxNum", int(0)) # 初始是0
    gv.setValue("boxNum_small", int(0)) # 初始是0
    gv.setValue("stackFloor", int(1)) # 初始是1
    #todo:------------------6.18--------------------------------------------------------------------------

    gv.setValue("lengthstack", [])
    gv.setValue("widthstack", [])
    gv.setValue("anglestack", [])
    gv.setValue("thickstack", [])
    gv.setValue("centerXstack", [])
    gv.setValue("centerYstack", [])
    gv.setValue("size", [])


    # todo:-------------------------------------------------------------------------------------------------------------------


#####liangziyang 大帅逼
    gv.setValue("boxNum_1", int(0)) # 初始是0，位置 +1 如果继续的话，从第二层继续，注意还要将第一层改成7，从第三层继续，注意还要将第一层改成7，第二层改成10
    gv.setValue("boxNum_2", int(0))
    gv.setValue("boxNum_3", int(0))
    gv.setValue("floor", int(1))# 层
    gv.setValue("num_baggage", int(0))
    gv.setValue("floor_baggage", int(1))
if __name__ == "__main__":
    init()
    # print(gv.getValue('第二个行李相机拍照标志'))   没有赋值none
    app = QApplication(sys.argv)  # 创建一个app实例，用于承载所有内容
    mainWindow = MainWindowFrame()  # 创建对应的窗口实例
    mainWindow.showMaximized()  # 将窗口设置为最大化
    # sys.exit(app.exec_())  # 启动app，注意，该行代码适用于python2和python3环境下的PyQt4和PyQt5
    app.exec()  # 启动app，注意，该行代码仅适用于python3.6及以上环境下的PyQt5
