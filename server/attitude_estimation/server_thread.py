# -*- coding: utf-8 -*-
import socket
import threading
import time

import numpy as np


class ServerThreadUDP(threading.Thread):
    def __init__(self, server_ip='127.0.0.1', port=50009, ini_val=np.ones((3, 3))):
        threading.Thread.__init__(self)
        # initial value
        self.data = ini_val

        self.kill_flag = False
        # line information
        self.host = server_ip
        # self.host = socket.gethostname()
        self.port = port
        self.buffsize = 1024
        self.addr = (socket.gethostbyname(self.host), self.port)
        # bind
        self.udpServSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udpServSock.bind(self.addr)  # HOST, PORTでbinding

    def run(self):
        while True:
            try:
                data, self.addr = self.udpServSock.recvfrom(self.buffsize)  # データ受信
                data = data.decode().split(',')
                self.data = np.array(data, dtype=np.float32).reshape((3, 3))

            except KeyboardInterrupt:
                break
            except:
                pass


if __name__ == '__main__':
    sock = ServerThreadUDP(server_ip='127.0.0.1', port=50009, ini_val=np.zeros((3, 3)))
    sock.setDaemon(True)
    sock.start()
    # sock.run()

    while True:
        if not sock.data:
            break
        print(sock.data)
        time.sleep(0.1)
