import numpy as np

class Planner():
    def __init__(self, resolution=30):
        self.resolution = resolution
        self.grid_size = 2
        self.maxX = 4500
        self.maxY = 3000
        self.robot_size = 200
        self.avoid_dist = 200
        self.offsets = [(1,0),(0,1),(-1,0),(0,-1),(1,1),(-1,1),(-1,-1),(1,-1)]
        # self.obstacle_offsets = [(1,0),(0,1),(-1,0),(0,-1),(1,1),(-1,1),(-1,-1),(1,-1)]
        self.obstacle_offsets = [(1,0),(0.707,0.707),(0,1),(-0.707,0.707),(-1,0),(-0.707,-0.707),(0,-1),(0.707,-0.707)]

    def plan(self, vision, start_x, start_y, goal_x, goal_y):
        # Obstacles
        obstacle_x = []
        obstacle_y = []
        for robot_blue in vision.blue_robot:
            if robot_blue.visible and robot_blue.id > 0:
                obstacle_x.append(int((robot_blue.x + self.maxX) / self.resolution))
                obstacle_y.append(int((robot_blue.y + self.maxY) / self.resolution))
        for robot_yellow in vision.yellow_robot:
            if robot_yellow.visible:
                obstacle_x.append(int((robot_yellow.x + self.maxX) / self.resolution))
                obstacle_y.append(int((robot_yellow.y + self.maxY) / self.resolution))
        
        sx, sy = start_x, start_y
        start_x = int((start_x + self.maxX) / self.resolution)
        start_y = int((start_y + self.maxY) / self.resolution)
        goal_x = int((goal_x + self.maxX) / self.resolution)
        goal_y = int((goal_y + self.maxY) / self.resolution)
        self.map_data = np.zeros((int(2 * self.maxX / self.resolution), \
            int(2 * self.maxY / self.resolution)), dtype=np.uint8) # {0 : no obstacle; 255 : obstacle}
        self.map_data[obstacle_x, obstacle_y] = np.uint8(255)
        self.dilate_map = self.map_data.copy()
        # dilate_ox, dilate_oy = self.dilate_ob(obstacle_x, obstacle_y)

        ''' Planning Choices '''
        path_x, path_y = self.A_star(start_x, start_y, goal_x, goal_y)

        path_x = [i * self.resolution - self.maxX for i in path_x] + [sx]
        path_y = [i * self.resolution - self.maxY for i in path_y] + [sy]

        return path_x, path_y

    def A_star(self, sx, sy, gx, gy):
        visit = np.zeros((self.map_data.shape[0], self.map_data.shape[1]))
        last = [[None for _ in range(self.map_data.shape[1])] for _ in range(self.map_data.shape[0])]
        found = False
        openlist = [[self.dist(sx, sy, gx, gy),0,(sx,sy)]]
        opendict = {(sx,sy): (self.dist(sx, sy, gx, gy),0)}
        cnt = 1
        while (cnt > 0):
            openlist.sort()
            [f,g,(now_x,now_y)] = openlist.pop(0)
            while visit[now_x, now_y]:
                [f,g,(now_x,now_y)] = openlist.pop(0)
            cnt = cnt - 1
            visit[now_x, now_y] = 1
            if (abs(now_x - gx) < self.grid_size) and (abs(now_y - gy) < self.grid_size):
                if not (now_x == gx and now_y == gy):
                    last[gx][gy] = (now_x, now_y)
                found = True
                break
            for (dx, dy) in self.offsets:
                dx *= self.grid_size
                dy *= self.grid_size
                if (now_x + dx) >= self.map_data.shape[0] or (now_y + dy) >= self.map_data.shape[1]: # out of map
                    continue
                if not visit[now_x + dx, now_y + dy]:
                    flag = True
                    if self.map_data[now_x + dx, now_y + dy] != 0 or self.dilate_map[now_x + dx, now_y + dy] == 255:
                        flag = False
                    elif self.dilate_map[now_x + dx, now_y + dy] != -255:
                        for scale in [0.5, 0.75, 1, 1.5, 2]:
                            for (robot_x, robot_y) in self.obstacle_offsets: # check obstacles
                                robot_x = int(np.ceil(self.robot_size * robot_x * scale / self.resolution)) 
                                robot_y = int(np.ceil(self.robot_size * robot_y * scale / self.resolution))
                                if (now_x + dx + robot_x) >= self.map_data.shape[0] or (now_y + dy + robot_y) >= self.map_data.shape[1]:
                                    continue
                                if self.map_data[now_x + dx + robot_x, now_y + dy + robot_y] != 0:
                                    self.dilate_map[now_x + dx, now_y + dy] = 255 # update map
                                    flag = False
                                    break
                            if flag is False:
                                break
                    if flag is False: # its nearby exists obstacles
                        visit[now_x + dx, now_y + dy] = 1
                    else:
                        self.dilate_map[now_x + dx, now_y + dy] = -255 # Represent being checked and no obstacles surrounding
                        if (now_x + dx, now_y + dy) in opendict:
                            (f_old, g_old) = opendict[(now_x + dx, now_y + dy)]
                            g_new = g + np.sqrt(dx**2 + dy**2)
                            f_new = g_new + f_old - g_old
                            if f_new < f_old:
                                opendict[(now_x + dx, now_y + dy)] = (f_new, g_new)
                                last[now_x + dx][now_y + dy] = (now_x, now_y)
                                openlist.append([f_new, g_new, (now_x + dx, now_y + dy)])
                        else:
                            g_new = g + np.sqrt(dx**2 + dy**2)
                            f_new = g_new + self.dist(now_x + dx, now_y + dy, gx, gy)
                            opendict[(now_x + dx, now_y + dy)] = (f_new, g_new)
                            last[now_x + dx][now_y + dy] = (now_x, now_y)
                            openlist.append([f_new, g_new, (now_x + dx, now_y + dy)])
                            cnt = cnt + 1
        if not found:
            print("No feasiable path found!")
            return None, None
        else:
            print("The path is found.")
            px, py = [gx], [gy]
            tmp_x, tmp_y = gx, gy
            while last[tmp_x][tmp_y] is not None:
                (tmp_x, tmp_y) = last[tmp_x][tmp_y]
                px.append(tmp_x)
                py.append(tmp_y)
            return px, py   

    def dilate_ob(self, ox, oy):
        # Dilate obstacles [cost 14s to play!!!]
        len_x = 2 * self.maxX + 1 # = 9001
        len_y = 2 * self.maxY + 1 # = 6001
        mapx = np.tile(np.expand_dims(np.arange(-self.maxX, self.maxX + 1, 1), axis=1), (1, len_y)) # (len_x, len_y)
        mapy = np.tile(np.expand_dims(np.arange(-self.maxY, self.maxY + 1, 1), axis=0), (len_x, 1)) # (len_x, len_y)
        dilate_ox, dilate_oy = [], []
        for (ox, oy) in zip(ox, oy):
            dx = ox - mapx 
            dy = oy - mapy 
            dist = np.hypot(dx, dy) # (len_x, len_y)
            mask = dist < self.robot_size + self.avoid_dist
            dilate_ox += list(mapx[mask] + self.maxX)
            dilate_oy += list(mapy[mask] + self.maxY)

        # Visualize [map]
        # cv2.imshow("MAP", np.transpose(self.map))
        # cv2.waitKey(1)
        return dilate_ox, dilate_oy

    def dist(self, sx, sy, gx, gy):
        return np.sqrt((sx - gx) ** 2 + (sy - gy) ** 2)