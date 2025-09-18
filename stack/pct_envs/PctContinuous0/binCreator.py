import numpy as np
import copy
import torch

import sys
sys.path.append("../..")
import configData as cfg


class BoxCreator(object):
    def __init__(self):
        self.box_list = []

    def reset(self):
        self.box_list.clear()

    def generate_box_size(self, **kwargs):
        pass

    def preview(self, length):
        while len(self.box_list) < length:
            self.generate_box_size()
        return copy.deepcopy(self.box_list[:length])

    def drop_box(self):
        assert len(self.box_list) >= 0
        self.box_list.pop(0)


class RandomBoxCreator(BoxCreator):
    default_box_set = []
    for i in range(5):
        for j in range(5):
            for k in range(5):
                default_box_set.append((2 + i, 2 + j, 2 + k))

    def __init__(self, box_size_set=None):
        super().__init__()
        self.box_set = box_size_set
        if self.box_set is None:
            self.box_set = RandomBoxCreator.default_box_set
        print(self.box_set)

    def generate_box_size(self, **kwargs):
        # idx = np.random.randint(0, len(self.box_set))
        x_input = 1e5
        y_input = 1e5
        z_input = 1e5
        while x_input > 5 or y_input > 5 or z_input > 5:
            try:
                input_data = input('请输入目标行李的长、宽、高尺寸，以英文半角逗号分隔：\n')
                if input_data == 'exit':
                    sys.exit('手动中断，程序停止运行')
                x_input, y_input, z_input = map(float, input_data.split(','))
            except Exception as e:
                print(e, '\t请重新输入')
        self.box_list.append([x_input, y_input, z_input])
        self.box_list.append(self.box_set[idx])


class LoadBoxCreator(BoxCreator):
    def __init__(self, data_name=None):
        super().__init__()
        self.data_name = data_name
        print("load data set successfully!")
        self.index = 0
        self.box_index = 0
        self.traj_nums = len(torch.load(self.data_name))
        self.box_trajs = torch.load(self.data_name)

    def reset(self, index=None):
        self.box_list.clear()
        self.recorder = []
        if index is None:
            self.index += 1
        else:
            self.index = index
        self.boxes = np.array(self.box_trajs[self.index])
        self.boxes = self.boxes.tolist()
        self.box_index = 0
        self.box_set = self.boxes
        self.box_set.append([cfg.spaceLength * 10, cfg.spaceWidth * 10, cfg.spaceHeight * 10])

    def generate_box_size(self, **kwargs):
        if self.box_index < len(self.box_set):
            self.box_list.append(self.box_set[self.box_index])
            self.recorder.append(self.box_set[self.box_index])
            self.box_index += 1
        else:
            self.box_list.append((cfg.spaceLength, cfg.spaceWidth, cfg.spaceHeight))
            self.recorder.append((cfg.spaceLength, cfg.spaceWidth, cfg.spaceHeight))
            self.box_index += 1
