# _*_ coding:utf-8 _*_
"""
@File       : socket_tcp_helper.py
@Project    : baggage-system-pyqt-rebuild
@Author     : DaveHao
@Contact_1  : dave@fcsoft.cc
@Software   : PyCharm
@License    : MulanPSL2
@Description: Socket助手类，实现了Tcp下的Socket服务端创建
"""
import socket
import threading
from time import sleep

from PyQt5.QtCore import pyqtSignal
from func import stop_thread as st

import global_value as gv

def get_host_ip() -> list:
    """
    获取本机IP
    :return: IP地址，字符串形式
    """
    # try:
    #     s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    #     s.connect(("8.8.8.8", 80))
    #     ip = s.getsockname()[0]
    # finally:
    #     s.close()
    # return ip

    hostname = socket.gethostname()  # 获取主机名
    addrs = socket.getaddrinfo(hostname, None)  # 获取主机中的所有地址列表
    ip_list = []
    for item in addrs:
        if item[0] == socket.AddressFamily.AF_INET:  # 枚举属于IPV4的地址
            ip_list.append(item[-1][0])  # 提取出IPV4地址并装入列表
    return ip_list


class SocketHelper:
    socketMsg = pyqtSignal(object, str)

    def __init__(self):
        self.tcp_socket = None
        self.sever_th = None
        self.client_socket_list = list()
        self.clientConnected = False

    def tcp_server_start(self, port: int) -> None:
        """
        功能函数，TCP服务端开启的方法
            :param port: 端口号
            :return: 无
        """
        try:
            self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # 取消主动断开连接四次握手后的TIME_WAIT状态
            self.tcp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            # 设定套接字为非阻塞式
            self.tcp_socket.setblocking(False)
            self.tcp_socket.bind((gv.getParam('hostIP'), port))
            self.tcp_socket.listen(5)  # 限制最大连接数
            self.sever_th = threading.Thread(target=self.tcp_server_concurrency)
            self.sever_th.start()
            msg = f"在端口:{port}上开始侦听\n"
            self.socketMsg.emit(msg, '信息')
        except:
            msg = '监听失败，请在 码垛设置->设备信息 中刷新本机IP后重试！'
            self.socketMsg.emit(msg, '错误')

    def tcp_server_concurrency(self):
        """
        功能函数，提供创建线程的方法；
        使用子线程用于监听并创建连接，使主线程可以继续运行，以免无响应
        使用非阻塞式并发用于接收客户端消息
        :return: 无
        """
        while True:
            try:
                client_socket, client_address = self.tcp_socket.accept()  # 尝试获取连接成功的客户端列表
            except Exception as ret:
                if self.clientConnected:
                    # self.socketMsg.emit('客户端断开连接', self.ERROR)
                    self.clientConnected = False
                sleep(0.002)  # 如果没有客户端连接成功，等待2ms后再次查询
            else:
                client_socket.setblocking(False)  # 设置为非阻塞式
                # 将创建的客户端套接字存入列表,client_address为ip和端口的元组
                self.client_socket_list.append((client_socket, client_address))
                self.clientConnected = True
                msg = f"已连接到 {client_address[0]}:{client_address[1]}\n"
                self.socketMsg.emit(msg, '信息')
            # 轮询客户端套接字列表，接收数据
            for client, address in self.client_socket_list:
                try:
                    recv_msg = client.recv(4096)  # 尝试接收数据
                except Exception:
                    pass  # 如果没有数据引发错误，则等待下一次尝试
                else:
                    if recv_msg:  # 如果接收到数据
                        info = recv_msg.decode("utf-8")  # 转码为utf-8
                        # msg = f'{address[0]}:{address[1]}->{info}'
                        msg = f'{info}'
                        self.socketMsg.emit(msg, '数据')
                        # print(msg)
                        # print(f"Message from client: {msg}")
                        # if info.strip() == "ready":  # Check if the received message is "ready"
                        #     # msg = f'{address[0]}:{address[1]}->{info}'
                        #     # self.socketMsg.emit(info, '111')
                        #     while True:
                        #         try:
                        #             recv_msg = client.recv(4096)  # Attempt to receive data from the client
                        #         except Exception:
                        #             pass
                        #         else:
                        #             if recv_msg:
                        #                 info = recv_msg.decode("utf-8")
                        #                 msg = info
                        #                 self.socketMsg.emit(msg, '111')
                        #                 print(f"Message from client: {address[0]}:{address[1]}->{msg}")
                        #             else:
                        #                 client.close()
                        #                 self.client_socket_list.remove((client, address))
                        #                 break
                    else:
                        client.close()
                        self.client_socket_list.remove((client, address))

    def tcp_send(self, send_info: str) -> None:
        """
        功能函数，用于TCP服务端和TCP客户端发送消息
        :param send_info: 等待发送的消息内容
        :return: 无
        """
        try:
            send_info_encoded = send_info.encode("utf-8")  # 转码为utf-8后准备发送
            if self.client_socket_list:
                for client, address in self.client_socket_list:
                    client.send(send_info_encoded)
                    msg = f'成功向{address[0]}:{address[1]}发送消息: {send_info}\n'
                    self.socketMsg.emit(msg, '成功')
            else:
                msg = f'发送失败，没有连接的客户端\n'
                self.socketMsg.emit(msg, '错误')

        except Exception as ret:
            msg = f'发送失败：{ret}\n'
            self.socketMsg.emit(msg, '错误')

    # def tcp_server_concurrency(self) -> None:

    def tcp_close(self) -> None:
        """
        功能函数，关闭网络连接的方法
        :return: 无
        """
        for client, address in self.client_socket_list:
            client.shutdown(socket.SHUT_RDWR)
            client.close()
        self.client_socket_list = list()  # 把已连接的客户端列表重新置为空列表
        self.tcp_socket.close()
        msg = "Socket服务端已关闭\n"
        self.socketMsg.emit(msg, '信息')
        # noinspection PyBroadException
        try:
            st.stop_thread(self.sever_th)
        except Exception:
            pass

if __name__ == '__main__':
    import socket

    IP = socket.gethostbyname(socket.gethostname())
    print(IP)