# _*_ coding:utf-8 _*_
"""
@File       : stack_deconstruction.py
@Project    : baggage-system-pyqt-rebuild
@Author     : DaveHao
@Contact_1  : dave@fcsoft.cc
@Software   : PyCharm
@License    : MulanPSL2
@Description: 垛型解构逻辑
"""
import os

import open3d as o3d
import numpy as np
import global_value as gv
from PyQt5.QtCore import QThread, pyqtSignal
import time

class StackDeconstruction(QThread):
    stack_deconstruction_done = pyqtSignal(str, str, bool)  # 行李测量成功的信号

    def __init__(self, parent=None):
        super(StackDeconstruction, self).__init__(parent)

    def run(self):
        """
        垛型解构入口函数
            :return: 无
        """
        print('解构')
        pcd = None
        scale_x = gv.getParam('scaleX')
        scale_y = gv.getParam('scaleY')
        scale_z = gv.getParam('scaleZ')
        grid_x = gv.getParam('gridX')
        grid_y = gv.getParam('gridY')
        grid_z = gv.getParam('gridZ')

        if gv.getValue('debugFlag'):  # 如果是调试模式
            if os.path.exists('./data/stack_latest.ply'):  # 如果点云文件存在
                pcd = o3d.io.read_point_cloud('./data/stack_latest.ply')  # 从文件加载点云
            else:
                self.stack_deconstruction_done.emit('垛型解构结束', '错误：没有检测到垛型点云数据，请确保垛型点云位于./data/stack_latest.jpg', True)
        else:  # 如果不是调试模式
            pcd = gv.getValue('stackPointCloud')  # 从公共变量加载点云
        try:
            s_time = time.time()
            pcd = self.point_cloud_presolve(pcd)  # 点云预处理
            e_time= time.time()
            print("耗时：{}秒".format(e_time-s_time))
            if len(np.asarray(pcd.points)) == 0:  # 如果检测不到点云
                self.stack_deconstruction_done.emit('垛型解构结束', '错误：没有检测到垛型主体，请检查 码垛设置->垛型点云预处理 参数设置是否有误',
                                                    gv.getValue('debugFlag'))  # 发送信号通知stackEventHandle方法行李测量失败
            labels = np.array(
                pcd.cluster_dbscan(eps=gv.getParam('stackEps'), min_points=gv.getParam('stackMinPts'),
                                   print_progress=False))  # DBSCAN聚类，滤掉噪点
            cluster_idx = [[] for j in range(labels.max() + 2)]
            i = 0
            # 聚类结果处理，标签转换为点的index
            for label in labels:
                cluster_idx[label + 1].append(i)
                i = i + 1  # 注意，cluster_idx中的内容从-1开始，因此应将label值加以转换为从0开始
            # 定义一堆列表，一会有用
            position_list = []
            size_list = []
            stack_items = []
            pcd_cluster = o3d.geometry.PointCloud()  # 定义一个空点云
            for i in range(1, len(cluster_idx)):  # 遍历聚类结果，其中cluster_idx[0]为噪点，不要，因此从1开始
                _pcd = pcd.select_by_index(cluster_idx[i])  # 筛选出对应的聚类结果，即车上行李
                _pcd.paint_uniform_color(np.random.rand(3))  # 随机上色
                pcd_cluster += _pcd  # 合并到结果点云中
                _aabb = _pcd.get_axis_aligned_bounding_box()  # 获取每个聚类结果的aabb包围框
                _x_min = _aabb.min_bound[0]
                _y_min = _aabb.min_bound[1]
                _x_max = _aabb.max_bound[0]
                _y_max = _aabb.max_bound[1]  # 求出xy坐标最值
                # 根据坐标计算出尺寸
                _xl = _x_max - _x_min
                _yl = _y_max - _y_min
                _zl = abs(_pcd.get_center()[-1]) + gv.getParam('stackThickIncrement')
                # 行李方向判断，用于增加补偿量
                if _xl > _yl:  # 如果行李是横着的
                    # 对位置进行补偿，注意位置不能为负
                    _pos_x = max(0, _x_min - gv.getParam('stackXmin') - gv.getParam('stackLengthIncrement') / 2)
                    _pos_y = max(0, gv.getParam('stackYmax') - _y_max - gv.getParam('stackWidthIncrement') / 2)
                    # 对尺寸进行补偿，同样，削去位置为负时多出来的一块
                    _yl = min(_yl + gv.getParam('stackWidthIncrement'),
                              _yl + gv.getParam('stackWidthIncrement') + _y_min - gv.getParam(
                                  'stackYmin') - gv.getParam('stackWidthIncrement') / 2)
                    _xl = min(_xl + gv.getParam('stackLengthIncrement'),
                              _xl + gv.getParam('stackLengthIncrement') + _x_min - gv.getParam(
                                  'stackXmin') - gv.getParam('stackLengthIncrement') / 2)
                else:  # 如果行李是竖着的
                    # 对位置进行补偿，注意位置不能为负
                    _pos_x = max(0, _x_min - gv.getParam('stackXmin') - gv.getParam('stackWidthIncrement') / 2)
                    _pos_y = max(0, gv.getParam('stackYmax') - _y_max - gv.getParam('stackLengthIncrement') / 2)
                    # 对尺寸进行补偿，同样，削去位置为负时多出来的一块
                    _yl = min(_yl + gv.getParam('stackLengthIncrement'),
                              _yl + gv.getParam('stackLengthIncrement') + _y_min - gv.getParam(
                                  'stackYmin') - gv.getParam('stackLengthIncrement') / 2)
                    _xl = min(_xl + gv.getParam('stackWidthIncrement'),
                              _xl + gv.getParam('stackWidthIncrement') + _x_min - gv.getParam(
                                  'stackXmin') - gv.getParam('stackWidthIncrement') / 2)
                # 对处理后的位置坐标和尺寸进行缩放，以使其适用于虚拟垛型和码垛算法
                _pos_x /= scale_x
                _pos_y /= scale_y
                _xl /= scale_x
                _yl /= scale_y
                _zl /= scale_z
                # 再对超出范围的尺寸进行消除
                if _pos_x + _xl > gv.getParam('gridX'):
                    _xl = gv.getParam('gridX') - _pos_x
                if _pos_y + _yl > gv.getParam('gridY'):
                    _yl = gv.getParam('gridY') - _pos_y
                if _zl > gv.getParam('gridZ'):
                    _zl = gv.getParam('gridZ')
                # 把坐标和位置装入列表保存
                position_list.append([_pos_x, _pos_y, 0])
                size_list.append(([_xl, _yl, _zl]))
                # 单独保存一份整数形式的供码垛使用
                stack_items.append([int(_pos_x), int(_pos_y), 0, int(_xl), int(_yl), int(_zl)])
            o3d.io.write_point_cloud('./data/stack_5_cluster.ply', pcd_cluster)
            # 位置和坐标列表转换为数组
            position_array = np.array(position_list)
            size_array = np.array(size_list)
            # 装入公共变量
            gv.setValue('drawStackPositionArray', position_array)
            gv.setValue('drawStackSizeArray', size_array)
            gv.setValue('stackItems', stack_items)
            # 发送信号通知垛型解构结束
            self.stack_deconstruction_done.emit('垛型解构结束', '', gv.getValue('debugFlag'))
        except Exception as e:
            self.stack_deconstruction_done.emit('垛型解构结束', f'错误：{str(e)}', gv.getValue('debugFlag'))
            gv.setValue('stackItems', None)
        finally:
            pass

    def point_cloud_presolve(self, cloud):
        """
        点云预处理，与行李测量部分类似
            :param cloud: 输入点云
            :return: 预处理后的点云
        """
        # 计算旋转矩阵
        rotate_matrix = o3d.geometry.Geometry3D.get_rotation_matrix_from_xyz(
            [gv.getParam('stackAlpha') / 180. * np.pi, gv.getParam('stackBeta') / 180. * np.pi,
             gv.getParam('stackGamma') / 180. * np.pi])
        # 执行旋转，旋转中心固定为（0，0，0）
        cloud.rotate(rotate_matrix, center=(0, 0, 0))
        o3d.io.write_point_cloud('./data/stack_0_rotated.ply', cloud)  # 保存
        # 定义裁剪边框
        vol = o3d.visualization.SelectionPolygonVolume()
        # ------------------------给定任意裁剪范围的顶点坐标-----------------------
        bounding_polygon = np.array([[gv.getParam('stackXmin'), gv.getParam('stackYmin'), 0.],
                                     [gv.getParam('stackXmin'), gv.getParam('stackYmax'), 0.],
                                     [gv.getParam('stackXmax'), gv.getParam('stackYmax'), 0.],
                                     [gv.getParam('stackXmax'), gv.getParam('stackYmin'), 0.], ], dtype=np.float64)
        vol.bounding_polygon = o3d.utility.Vector3dVector(bounding_polygon)
        # 定义裁剪区域的高度
        vol.orthogonal_axis = "z"
        vol.axis_min = gv.getParam('stackZmin')  # 设置取值范围的最小值
        vol.axis_max = gv.getParam('stackZmax')  # 设置取值范围的最大值
        _pcd = vol.crop_point_cloud(cloud)  # 执行裁剪
        o3d.io.write_point_cloud('./data/stack_1_cropped.ply', _pcd)
        # 平移，将车底板平面高度调整为0
        _pcd.translate((0, 0, gv.getParam('stackZOffset')))
        o3d.io.write_point_cloud('./data/stack_2_transformed.ply', _pcd)
        _pcd = _pcd.voxel_down_sample(voxel_size=0.01)  # 1cm体素下采样
        o3d.io.write_point_cloud('./data/stack_3_downsampled.ply', _pcd)
        r = gv.getParam('stackPts')
        r2 = gv.getParam('stackRadius')
        _, ind = _pcd.remove_radius_outlier(nb_points=gv.getParam('stackPts'),
                                            radius=gv.getParam('stackRadius'))  # 半径滤波
        _pcd = _pcd.select_by_index(ind)  # 应用滤波器
        _pcd.paint_uniform_color((1, 0, 0))  # 红色
        o3d.io.write_point_cloud('./data/stack_4_flittered_radius.ply', _pcd)
        _, ind = _pcd.remove_statistical_outlier(nb_neighbors=gv.getParam('stackPts'),
                                                 std_ratio=gv.getParam('stackStd'))  # 统计滤波
        _pcd.paint_uniform_color((0, 0, 1))  # 蓝色
        _pcd = _pcd.select_by_index(ind)  # 应用滤波器
        o3d.io.write_point_cloud('./data/stack_4_flittered_sta.ply', _pcd)
        # 保存点云
        return _pcd
