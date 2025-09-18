import numpy as np
import torch, copy
import time
import sys
import global_value as gv
from stack import configData as cfg


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
    for i in range(cfg.baggageLength_min, cfg.baggageLength_max + 1):
        for j in range(cfg.baggageWidth_min, cfg.baggageWidth_max + 1):
            for k in range(cfg.baggageThick_min, cfg.baggageThick_max + 1):
                default_box_set.append((i, j, k))

    def __init__(self, box_size_set=None):
        super().__init__()
        self.box_set = box_size_set
        if self.box_set is None:
            self.box_set = RandomBoxCreator.default_box_set
        # print(self.box_set)

    def generate_box_size(self, **kwargs):
        # 获取箱子尺寸，补偿尺寸后转换为厘米，最后向上取整
        x_input = int((gv.getValue('currentBaggageLength') + gv.getParam('baggageLengthIncrement')) / 10) + 1
        y_input = int((gv.getValue('currentBaggageWidth') + gv.getParam('baggageWidthIncrement')) / 10) + 1
        z_input = int((gv.getValue('currentBaggageThick')+ gv.getParam('baggageThickIncrement')) / 10) + 1
        self.box_list.append([x_input, y_input, z_input])


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
