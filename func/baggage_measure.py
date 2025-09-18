# _*_ coding:utf-8 _*_
"""
@File       : baggage_measure.py
@Project    : baggage-system-pyqt-rebuild
@Author     : DaveHao
@Contact_1  : dave@fcsoft.cc
@Software   : PyCharm
@License    : MulanPSL2
@Description: 行李测量职能脚本，对传入点云进行预处理和测量后更新对应的全局变量，并发送信号更新GUI中的对应元素
"""
import os
import open3d as o3d
import numpy as np
import global_value as gv
from PyQt5.QtCore import QThread, pyqtSignal


class BaggageMeasure(QThread):
    baggage_measure_done = pyqtSignal(str, str, bool)  # 行李测量成功的信号
    baggagesize1st = []
    def __init__(self, parent=None):
        super(BaggageMeasure, self).__init__(parent)

    def run(self):
        """
        测量入口函数
        :return: 无
        """
        pcd = None
        if gv.getValue('debugFlag'):  # 如果是调试模式
            if os.path.exists('./data/baggage_latest.ply'):  # 如果点云文件存在
                pcd = o3d.io.read_point_cloud('./data/baggage_latest.ply')  # 从文件加载点云
            else:
                self.baggage_measure_done.emit('行李测量结束', '错误：没有检测到行李点云数据，请确保行李点云位于./data/baggage_latest.ply', True)
        else:  # 如果不是调试模式
            pcd = gv.getValue('baggagePointCloud')  # 从公共变量加载点云
        try:
            pcd = self.point_cloud_presolve(pcd)  # 点云预处理
            if len(np.asarray(pcd.points)) == 0:  # 如果检测不到点云
                self.baggage_measure_done.emit('行李测量结束', '错误：没有检测到行李主体，请检查行李微调数据设置是否有误',
                                               gv.getValue('debugFlag'))  # 发送信号通知stackEventHandle方法行李测量失败

            labels = np.array(
                pcd.cluster_dbscan(eps=gv.getParam('baggageEps'), min_points=gv.getParam('baggageMinPts'),
                                   print_progress=False))  # DBSCAN聚类，滤掉噪点
            cluster_idx = [[] for j in range(labels.max() + 2)]
            i = 0
            # 聚类结果处理
            for label in labels:
                cluster_idx[label + 1].append(i)
                i = i + 1
            cluster_idx = sorted(cluster_idx, key=len)  # 各部分点云的序号数组，按长度排序
            pcd.paint_uniform_color((255 / 255, 165 / 255, 0))  # 橙色
            pcd = pcd.select_by_index(cluster_idx[-1])  # 列表长度最长的就是主体的点云
            o3d.io.write_point_cloud('./data/baggage_6_cluster.ply', pcd)
            gv.setValue('currentBaggageThick',
                        round(abs(pcd.get_center()[-1]), 2))  # 行李的厚度：质心z坐标绝对值
#todo:___________________________________6.19___________________________
            thk_tmp = gv.getValue('thickstack')
            thk_tmp.append(round(abs(pcd.get_center()[-1]), 2))
            gv.setValue('thickstack', thk_tmp)
            print(gv.getValue('thickstack'))
#todo:_________________________________________________________________________________
            pcd = self.point_cloud_plane_project(pcd, [0, 0, 1, 0])  # 投影到xy平面
            w, v = self.PCA(np.asarray(pcd.points))  # PCA分析
            r_vec = v[:, 0]  # 计算旋转矩阵，使用第一列（最大特征值对应的列）作为主方向矢量
            r_angle_rad = np.arctan(r_vec[1] / r_vec[0])  # 计算出行李的旋转角度(弧度)
            r_angle_deg = r_angle_rad / np.pi * 180.0  # 转换为角度等待使用
            # 取余角，转换为真实角度
            if r_angle_deg > 0:
                r_angle_deg = 90 - r_angle_deg
            else:
                r_angle_deg = - 90 - r_angle_deg
            r_positive = o3d.geometry.Geometry3D.get_rotation_matrix_from_xyz([0, 0, -r_angle_rad])  # 计算出反向旋转的矩阵
            r_negative = o3d.geometry.Geometry3D.get_rotation_matrix_from_xyz([0, 0, r_angle_rad])  # 计算出正向旋转的矩阵
            pcd.rotate(r_positive)  # 应用旋转
            aabb = pcd.get_axis_aligned_bounding_box()  # 创建旋转后点云的AABB包围盒
            obb = aabb.get_oriented_bounding_box()  # 转换为obb包围盒以便于旋转
            obb.rotate(r_negative)  # 反向旋转回原位以计算坐标
            # print(round(obb.extent[0], 2))
            # print(round(obb.extent[1], 2))
            # print(round(r_angle_deg, 2))
            # print(round(-obb.get_center()[0], 2))
            # print(round(obb.get_center()[1], 2))
            gv.setValue('currentBaggageLength', round(obb.extent[0], 2))  # 保存行李的长度和宽度数据到公共变量
            gv.setValue('currentBaggageWidth', round(obb.extent[1], 2))  # 单位：mm
            gv.setValue('currentBaggageAngle', round(r_angle_deg, 2))  # 保存行李的形位数据到公共变量
            gv.setValue('currentBaggageCenterX', round(-obb.get_center()[0], 2))  # 注意方向不同，x值应为相反数
            gv.setValue('currentBaggageCenterY', round(obb.get_center()[1], 2))

            # todo:------------------6.18--------------------------------------------------------------------------
            len_tmp = gv.getValue('lengthstack')
            # print(len_tmp)
            len_tmp.append(round(obb.extent[0], 2))
            # print(len_tmp)
            gv.setValue('lengthstack', len_tmp)
            print(gv.getValue('lengthstack'))

            wid_tmp = gv.getValue('widthstack')
            wid_tmp.append(round(obb.extent[1], 2))
            gv.setValue('widthstack', wid_tmp)
            print(gv.getValue('widthstack'))
            if round(obb.extent[1], 2) >440:
                size_tmp = gv.getValue('size')
                size_tmp.append(int(1))
                gv.setValue('size', size_tmp)
                print(gv.getValue('size'))
            else:
                size_tmp = gv.getValue('size')
                size_tmp.append(int(0))
                gv.setValue('size', size_tmp)
                print(gv.getValue('size'))

            ang_tmp = gv.getValue('anglestack')
            ang_tmp.append(round(r_angle_deg, 2))
            gv.setValue('anglestack', ang_tmp)
            print(gv.getValue('anglestack'))

            CeX_tmp = gv.getValue('centerXstack')
            CeX_tmp.append(round(-obb.get_center()[0], 2))
            gv.setValue('centerXstack',CeX_tmp)
            print(gv.getValue('centerXstack'))

            CeY_tmp = gv.getValue('centerYstack')
            CeY_tmp.append(round(obb.get_center()[1], 2))
            gv.setValue('centerYstack', CeY_tmp)
            print(gv.getValue('centerYstack'))

            # todo:-------------------------------------------------------------------------------------------------------------------

            print(gv.getValue('currentBaggageAngle'))
            print(gv.getValue('currentBaggageThick'))

            self.baggagesize1st.append(int(gv.getValue('currentBaggageLength')))
            self.baggagesize1st.append(int(gv.getValue('currentBaggageWidth')))
            chang1 = self.baggagesize1st[0]
            kuan1 = self.baggagesize1st[1]


            gv.setValue('chang1', chang1)
            gv.setValue('kuan1', kuan1)
            # print(f"fgfgfg", self.baggagesize1st)
            #
            # print(chang1)
            # print(kuan1)
            #
            # print(gv.getValue('currentBaggageCenterY'))
            self.baggage_measure_done.emit('行李测量结束', '', gv.getValue('debugFlag'))  # 发送信号通知stackEventHandle方法行李测量结束
        except Exception as e:
            self.baggage_measure_done.emit('行李测量结束', f'错误：{str(e)}', gv.getValue('debugFlag'))

    # 点云预处理
    def point_cloud_presolve(self, cloud):
        """
        点云预处理
        :param cloud: 输入点云
        :return: 预处理后的点云
        """
        # 计算旋转矩阵
        rotate_matrix = o3d.geometry.Geometry3D.get_rotation_matrix_from_xyz(
            [gv.getParam('baggageAlpha') / 180 * np.pi, gv.getParam('baggageBeta') / 180 * np.pi,
             gv.getParam('baggageGamma') / 180 * np.pi])
        # 执行旋转，旋转中心固定为（0，0，0）
        cloud.rotate(rotate_matrix, center=(0, 0, 0))
        cloud.paint_uniform_color((240 / 255, 128 / 255, 128 / 255))  # 浅红
        o3d.io.write_point_cloud('./data/baggage_0_rotated.ply', cloud)  # 保存
        # 定义裁剪边框
        vol = o3d.visualization.SelectionPolygonVolume()
        # ------------------------给定任意裁剪范围的顶点坐标-----------------------
        bounding_polygon = np.array([[gv.getParam('baggageXmin'), gv.getParam('baggageYmin'), 0.],
                                     [gv.getParam('baggageXmin'), gv.getParam('baggageYmax'), 0.],
                                     [gv.getParam('baggageXmax'), gv.getParam('baggageYmax'), 0.],
                                     [gv.getParam('baggageXmax'), gv.getParam('baggageYmin'), 0.], ], dtype=np.float64)
        vol.bounding_polygon = o3d.utility.Vector3dVector(bounding_polygon)
        # 定义裁剪区域的高度
        vol.orthogonal_axis = "z"
        vol.axis_min = gv.getParam('baggageZmin')  # 设置取值范围的最小值
        vol.axis_max = gv.getParam('baggageZmax')  # 设置取值范围的最大值
        _pcd = vol.crop_point_cloud(cloud)  # 执行裁剪
        _pcd.paint_uniform_color((1, 0, 0))  # 深蓝
        o3d.io.write_point_cloud('./data/baggage_1_cropped.ply', _pcd)
        # 平移，将传送带平面高度调整为0
        _pcd.translate((0, 0, gv.getParam('baggageZOffset')))
        _pcd.paint_uniform_color((25 / 255, 25 / 255, 112 / 255))  # 深蓝
        o3d.io.write_point_cloud('./data/baggage_2_transformed.ply', _pcd)
        _pcd = _pcd.voxel_down_sample(voxel_size=0.01)  # 1cm体素下采样
        _pcd.paint_uniform_color((1, 0, 1))  # 紫色
        o3d.io.write_point_cloud('./data/baggage_3_downsampled.ply', _pcd)
        _, ind = _pcd.remove_statistical_outlier(nb_neighbors=gv.getParam('baggagePts'),
                                                 std_ratio=gv.getParam('baggageStd'))  # 统计滤波
        _pcd = _pcd.select_by_index(ind)  # 应用滤波器
        _pcd.paint_uniform_color((64 / 255, 224 / 255, 208 / 255))  # 青色
        o3d.io.write_point_cloud('./data/baggage_4_flittered_sta.ply', _pcd)
        _, ind = _pcd.remove_radius_outlier(nb_points=gv.getParam('baggagePts'),
                                            radius=gv.getParam('baggageRadius'))  # 半径滤波
        _pcd = _pcd.select_by_index(ind)  # 应用滤波器
        # _pcd.paint_uniform_color((1, 215 / 255, 0))  # 金色
        # o3d.io.write_point_cloud('./data/baggage_5_flittered_rad.ply', _pcd)
        # 保存点云
        return _pcd

    def point_cloud_plane_project(self, cloud, coefficients):
        """
        点云投影到平面
        :param cloud:输入点云
        :param coefficients: 待投影的平面Ax+By+Cz+D=0
        :return: 投影后的点云
        """
        # 获取平面系数
        A = coefficients[0]
        B = coefficients[1]
        C = coefficients[2]
        D = coefficients[3]
        # 构建投影函数
        Xcoff = np.array([B * B + C * C, -A * B, -A * C])
        Ycoff = np.array([-B * A, A * A + C * C, -B * C])
        Zcoff = np.array([-A * C, -B * C, A * A + B * B])
        # 三维坐标执行投影
        points = np.asarray(cloud.points)
        xp = np.dot(points, Xcoff) - A * D
        yp = np.dot(points, Ycoff) - B * D
        zp = np.dot(points, Zcoff) - C * D
        project_points = np.c_[xp, yp, zp]  # 投影后的三维坐标
        project_cloud = o3d.geometry.PointCloud()  # 使用numpy生成点云
        project_cloud.points = o3d.utility.Vector3dVector(project_points)
        project_cloud.colors = cloud.colors  # 点云颜色传递（可选）
        return project_cloud

    def PCA(self, cloud, correlation=False, sort=True):
        """
        点云PCA主方向计算
            :param cloud: 输入点云
            :param correlation: 区分np的cov和corrcoef
            :param sort: 是否对输出进行排序
            :return: v-特征值 w-特征向量
        """
        mean_data = np.mean(cloud, axis=0)  # k均值计算
        normal_data = cloud - mean_data  # 归一化
        H = np.dot(normal_data.T, normal_data)  # 计算对称的协方差矩阵
        # SVD奇异值分解，得到H矩阵的特征值和特征向量
        eigenvectors, eigenvalues, _ = np.linalg.svd(H)
        if sort:
            sort = eigenvalues.argsort()[::-1]
            eigenvalues = eigenvalues[sort]
            eigenvectors = eigenvectors[:, sort]
        return eigenvalues, eigenvectors
