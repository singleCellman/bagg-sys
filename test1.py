import numpy as np

_space_length = int('288')
_space_width = int('147')
_space_height = int('84')
p_array = np.array([[0, 0, -1], [-1, 0, -1], [_space_length, 0, -1], [-1, _space_width, -1]])
size_array = np.array([[_space_length, _space_width, 1], [1, _space_width + 1, _space_height + 21],
                       [1, _space_width + 1, _space_height + 21],
                       [_space_length + 2, 1, _space_height + 21]])
test1 = p_array.shape[:-1]
test2 = p_array.shape[0]
nCubes = np.prod(p_array.shape[:-1])
cubeVerts = np.mgrid[0:2, 0:2, 0:2].reshape(3, 8).transpose().reshape(1, 8, 3)
cubeFaces = np.array([
    [0, 1, 2], [3, 2, 1],
    [4, 5, 6], [7, 6, 5],
    [0, 1, 4], [5, 4, 1],
    [2, 3, 6], [7, 6, 3],
    [0, 2, 4], [6, 4, 2],
    [1, 3, 5], [7, 5, 3]]).reshape(1, 12, 3)
size = size_array.reshape((nCubes, 1, 3))
pos = p_array.reshape((nCubes, 1, 3))
verts = cubeVerts * size + pos
faces = cubeFaces + (np.arange(nCubes) * 8).reshape(nCubes, 1, 1)
t3 = verts.reshape(nCubes * 8, 3)
t4 = faces.reshape(nCubes * 12, 3)
print()


