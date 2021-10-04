import socket
import numpy as np
import math
class Interface:
    class Data:
        pass
    def __init__(self):
        self.recv_port = 45454
        self.send_port = 45455
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.socket2.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.socket.bind(('', self.recv_port))
        self.cmd = self.Data()
    
    def getSignedCharValue(self,a):
        a = a%256
        if a > 127:
            return a - 256
        return a

    def getVelCmd(self):
        data, _ = self.socket.recvfrom(1024)
        self.cmd.velX = self.getSignedCharValue(data[0])
        self.cmd.velY = -self.getSignedCharValue(data[1])
        self.cmd.velW = self.getSignedCharValue(data[2])
        return self.cmd
        
    def controlWheel(self,a,b,c,d):
        data = np.array([a,b,c,d])*60/(2*math.pi)/2.5
        # data = np.array([a,b,c,d])
        data = data.clip(-127,127)
        data = bytes(data.astype(np.int8))
        self.socket2.sendto(data,('127.0.0.1',self.send_port))
