import os
import open3d as o3d
import numpy as np
import global_value as gv
from PyQt5.QtCore import QThread, pyqtSignal
import time


pcd = o3d.io.read_point_cloud('./data/stack_4_flittered_sta.ply')
labels = np.array(pcd.cluster_dbscan(eps=55, min_points=20,print_progress=False))  # DBSCAN聚类，滤掉噪点
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
    _zl = abs(_pcd.get_center()[-1]) + 20
    # 行李方向判断，用于增加补偿量
    if _xl > _yl:  # 如果行李是横着的
        # 对位置进行补偿，注意位置不能为负
        _pos_x = max(0, _x_min - (-1460) - 40 / 2)
        _pos_y = max(0, 900 - _y_max - 40 / 2)
        # 对尺寸进行补偿，同样，削去位置为负时多出来的一块
        _yl = min(_yl + 40, _yl + 40 + _y_min - (-580) - 40 / 2)
        _xl = min(_xl + 40, _xl + 40 + _x_min - (-1460) - 40 / 2)
    else:  # 如果行李是竖着的
        # 对位置进行补偿，注意位置不能为负
        _pos_x = max(0, _x_min - (-1460) - 40 / 2)
        _pos_y = max(0, 900 - _y_max - 40 / 2)
        # 对尺寸进行补偿，同样，削去位置为负时多出来的一块
        _yl = min(_yl + 40, _yl + 40 + _y_min - (-580) - 40 / 2)
        _xl = min(_xl + 40, _xl + 40 + _x_min - (-1460) - 40 / 2)
    # 对处理后的位置坐标和尺寸进行缩放，以使其适用于虚拟垛型和码垛算法
    _pos_x /= 10
    _pos_y /= 10
    _xl /= 10
    _yl /= 10
    _zl /= 10
    # 再对超出范围的尺寸进行消除
    if _pos_x + _xl > 288:
        _xl = 288 - _pos_x
    if _pos_y + _yl > 147:
        _yl = 147 - _pos_y
    if _zl > 84:
        _zl = 84
    # 把坐标和位置装入列表保存
    position_list.append([_pos_x, _pos_y, 0])
    size_list.append(([_xl, _yl, _zl]))
    # 单独保存一份整数形式的供码垛使用
    stack_items.append([int(_pos_x), int(_pos_y), 0, int(_xl), int(_yl), int(_zl)])
o3d.io.write_point_cloud('./data/stack_5_cluster.ply', pcd_cluster)