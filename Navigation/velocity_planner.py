import numpy as np

class StupidTrack:
    def __init__(self):
        self.maxVel = 3 # [3m/s]
        self.maxAccV = 3 # [3m/s^2]
        self.maxVw = 2 # [rad/s]
        self.maxAccVw = 5 # [rad/s^2]
        self.expectTime = 1.2 # [s]
        self.dt = 0.01
        self.dist_th = 1000 # reduce speed
        self.lastVx = 0
        self.lastVw = 0

    def plan(self, *args):
        self.now_pos = args[0] # [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        self.goal = args[1] # [x(m), y(m)]
        theta = self.now_pos[2]
        beta = np.arctan2(self.goal[1] - self.now_pos[1], self.goal[0] - self.now_pos[0])
        # self.lastVx, self.lastVw = self.now_pos[3], self.now_pos[4]
        expectVx = self.maxVel
        if abs(beta - theta) >= np.pi: 
            expectVw = (theta - beta) / self.expectTime
        else:
            expectVw = (beta - theta) / self.expectTime
        if expectVw > self.maxVw:
            expectVw = self.maxVw
        if expectVx > self.maxVel:
            expectVx = self.maxVel 
        accV = (expectVx - self.lastVx) / self.dt
        accVw = (expectVw - self.lastVw) / self.dt
        vx, vw = expectVx, expectVw # Init
        if np.abs(accV) > self.maxAccV:
            vx = self.lastVx + self.maxAccV * self.dt * accV / np.abs(accV)
        if np.abs(accVw) > self.maxAccVw:
            vw = self.lastVw + self.maxAccVw * self.dt * accVw / np.abs(accVw)

        self.lastVx, self.lastVw = vx, vw
        vy = 0
        return vx, vy, vw
    
class FB_Tracking:
    def __init__(self):
        self.k1 = 0.1 # 1
        self.k2 = 300 # 300
        self.mu = 0.5
        self.lambd = 2
        self.maxVel = 3.0

    def plan(self, *args):
        now_pos = args[0] # [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        goal = args[1] # [x(m), y(m)]
        theta = now_pos[2]
        rho = np.sqrt((goal[1] - now_pos[1])**2 + (goal[0] - now_pos[0])**2)
        beta = np.arctan2(goal[1] - now_pos[1], goal[0] - now_pos[0])
        tmp_angle = beta - theta
        alpha = np.arctan2(np.sin(tmp_angle), np.cos(tmp_angle))
        k = 1.0 / rho * (self.k2 * (alpha - np.arctan(-self.k1 * beta)) + \
            (1 + self.k1/(1 + (self.k1 * beta)**2)) * np.sin(alpha))
        vx = self.maxVel / (1 + self.mu * (k ** self.lambd))
        vw = vx * k
        vy = 0 # TODO
        return vx, vy, vw