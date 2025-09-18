# _*_ coding:utf-8 _*_
"""
@File       : opengl_helper.py
@Project    : baggage-system-pyqt-rebuild
@Author     : DaveHao
@Contact_1  : dave@fcsoft.cc
@Software   : PyCharm
@License    : MulanPSL2
@Description:
"""
__all__ = ['createVoxelMesh']

import numpy as np
from pyqtgraph.opengl.items.GLMeshItem import GLMeshItem
from pyqtgraph.opengl.MeshData import MeshData


def createVoxelMesh(pos, size, facecolor, edgecolor=(1, 1, 1, 1), edge=False, shader='shaded'):
    """
    将坐标和尺寸转为体素
        :param pos: xyz位置坐标，形状为
        :param size: 长、宽、高
        :return: MeshData类型的体素对象
    """
    nCubes = np.prod(pos.shape[:-1])
    cubeVerts = np.mgrid[0:2, 0:2, 0:2].reshape(3, 8).transpose().reshape(1, 8, 3)
    cubeFaces = np.array([
        [0, 1, 2], [3, 2, 1],
        [4, 5, 6], [7, 6, 5],
        [0, 1, 4], [5, 4, 1],
        [2, 3, 6], [7, 6, 3],
        [0, 2, 4], [6, 4, 2],
        [1, 3, 5], [7, 5, 3]]).reshape(1, 12, 3)
    size = size.reshape((nCubes, 1, 3))
    pos = pos.reshape((nCubes, 1, 3))
    verts = cubeVerts * size + pos
    faces = cubeFaces + (np.arange(nCubes) * 8).reshape(nCubes, 1, 1)
    md = MeshData(verts.reshape(nCubes * 8, 3), faces.reshape(nCubes * 12, 3))
    mesh = GLMeshItem(meshdata=md, shader=shader, drawEdges=edge, smooth=False, color=facecolor,
                      edgecolor=edgecolor)
    return mesh