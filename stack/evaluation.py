import gym
from PyQt5.QtCore import QThread, pyqtSignal

from .model import *
from .tools import *
from . import tools


class stackPlanning(QThread):
    stack_plan_done = pyqtSignal(str, str, bool)
    def __init__(self, parent=None):
        super(stackPlanning, self).__init__(parent)
        registration_envs()
        self.args = get_args()
        if self.args.no_cuda:
            self.device = torch.device('cpu')
        else:
            self.device = torch.device('cuda', self.args.device)
            torch.cuda.set_device(self.args.device)
        torch.set_num_threads(1)
        torch.manual_seed(self.args.seed)
        torch.cuda.manual_seed_all(self.args.seed)
        # Create single packing environment and load existing dataset.
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
        # Create the main actor & critic networks of PCT
        self.PCT_policy = DRL_GAT(self.args)
        self.PCT_policy = self.PCT_policy.to(self.device)
        # Load the trained model
        if self.args.load_model:
            self.PCT_policy = load_policy(self.args.model_path, self.PCT_policy)
            print('Pre-train model loaded!', self.args.model_path)

    def run(self):
        # 行李尺寸合法判据
        if gv.getValue('currentBaggageLength') is None or gv.getValue('currentBaggageWidth') is None or gv.getValue('currentBaggageThick') is None:
            self.stack_plan_done.emit('码垛规划结束', '错误：行李尺寸不合法，请先进行行李测量！', gv.getValue('debugFlag'))
        else:
            # if gv.getValue('stackItems') is None:
            #     pass
            #     self.stack_plan_done.emit('码垛规划结束', '错误：未检测到可用垛型，请先进行垛型解构！', gv.getValue('debugFlag'))
            # else:
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
            # items = self.envs.packed
            # action = selected_leaf_node.cpu().numpy()[0][0:6]  # debug用
            obs, reward, done, infos = self.envs.step(selected_leaf_node.cpu().numpy()[0][0:6])
            if done:  # 无可用空间，通知码垛结束，下一车
                self.stack_plan_done.emit('码垛规划结束', '空间已满', gv.getValue('debugFlag'))
            else: # 通知码垛规划成功
                self.stack_plan_done.emit('码垛规划结束', '', gv.getValue('debugFlag'))
            #     obs = self.envs.reset()
            # obs = torch.FloatTensor(obs).to(self.device).unsqueeze(dim=0)
            # all_nodes, leaf_nodes = tools.get_leaf_nodes_with_factor(obs, self.args.num_processes,
            #                                                          self.args.internal_node_holder,
            #                                                          self.args.leaf_node_holder)
            # all_nodes, leaf_nodes = all_nodes.to(self.device), leaf_nodes.to(self.device)
