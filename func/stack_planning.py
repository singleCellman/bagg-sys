import gym
from PyQt5.QtCore import QThread, pyqtSignal
import torch
from stack.model import DRL_GAT
from stack.tools import get_args, registration_envs, load_policy
from stack import tools
import os.path
import global_value as gv


class StackPlanning(QThread):
    stack_plan_done = pyqtSignal(str, str, bool)

    def __init__(self, parent=None):
        super(StackPlanning, self).__init__(parent)
        registration_envs()  # 注册环境
        self.args = get_args()
        if self.args.no_cuda:  # 判断推理所使用的设备
            self.device = torch.device('cpu')
        else:
            self.device = torch.device('cuda', self.args.device)
            torch.cuda.set_device(self.args.device)
        torch.set_num_threads(1)  # 单线程推理
        torch.manual_seed(self.args.seed)  # 初始化随机数种子
        torch.cuda.manual_seed_all(self.args.seed)  # 应用种子
        # 创建gym环境
        self.envs = gym.make(self.args.id,
                             setting=self.args.setting,
                             container_size=self.args.container_size,
                             item_set=self.args.item_size_set,
                             data_name=self.args.dataset_path,
                             load_test_data=self.args.load_dataset,
                             internal_node_holder=self.args.internal_node_holder,
                             leaf_node_holder=self.args.leaf_node_holder,
                             LNES=self.args.lnes,
                             shuffle=self.args.shuffle)
        # 创建PCT策略下的actor&critic网络
        self.PCT_policy = DRL_GAT(self.args)
        self.PCT_policy = self.PCT_policy.to(self.device)
        # 加载预训练模型
        if self.args.load_model:
            if os.path.exists(self.args.model_path):
                self.PCT_policy = load_policy(self.args.model_path, self.PCT_policy)
                self.loaded = True
            else:
                self.loaded = False

    def run(self):
        # 行李尺寸合法判据
        if not self.loaded:
            self.stack_plan_done.emit('码垛规划结束', '错误：未找到模型，请在 算法设置->码垛算法设置 中选择码垛模型后重试', gv.getValue('debugFlag'))
        if gv.getValue('currentBaggageLength') is None or gv.getValue('currentBaggageWidth') is None or gv.getValue(
                'currentBaggageThick') is None:
            self.stack_plan_done.emit('码垛规划结束', '错误：行李尺寸不合法，请先进行行李测量！', gv.getValue('debugFlag'))
        else:
            self.PCT_policy.eval()
            obs = self.envs.reset()
            obs = torch.FloatTensor(obs).to(self.device).unsqueeze(dim=0)
            all_nodes, leaf_nodes = tools.get_leaf_nodes_with_factor(obs, self.args.num_processes,
                                                                     self.args.internal_node_holder,
                                                                     self.args.leaf_node_holder)
            batchX = torch.arange(self.args.num_processes)
            with torch.no_grad():
                selectedlogProb, selectedIdx, policy_dist_entropy, value = self.PCT_policy(all_nodes, True,
                                                                                           normFactor=1)
            selected_leaf_node = leaf_nodes[batchX, selectedIdx.squeeze()]
            obs, reward, done, infos = self.envs.step(selected_leaf_node.cpu().numpy()[0][0:6])
            # print(reward)
            if done:  # 无可用空间，通知码垛结束，下一车
                self.stack_plan_done.emit('码垛规划结束', '空间已满', gv.getValue('debugFlag'))
            else:  # 通知码垛规划成功
                self.stack_plan_done.emit('码垛规划结束', '', gv.getValue('debugFlag'))
