from sys import path
from scipy.spatial import KDTree
import numpy as np
import random
import math
import time


class Node(object):
    def __init__(self, x, y, cost, parent):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = parent


class RRT(object):
    def __init__(self):
        self.samplePro = 0.1
        self.step = 60
        self.dist_th = 300
        self.search_MaxNum = 10000
        self.optimize_len = 10

        self.KNN = 10
        self.MAX_EDGE_LEN = 5000
        self.count = 0
        self.minx = -4500
        self.maxx = 4500
        self.miny = -3000
        self.maxy = 3000
        self.robot_size = 200
        self.avoid_dist = 200

    def plan(self, vision, start_x, start_y, goal_x, goal_y):
        # Obstacles
        self.obstacle_x = [-999999]
        self.obstacle_y = [-999999]
        for robot_blue in vision.blue_robot:
            if robot_blue.visible and robot_blue.id > 0:
                self.obstacle_x.append(robot_blue.x)
                self.obstacle_y.append(robot_blue.y)
        for robot_yellow in vision.yellow_robot:
            if robot_yellow.visible:
                self.obstacle_x.append(robot_yellow.x)
                self.obstacle_y.append(robot_yellow.y)

        self.obstree = KDTree(np.vstack((self.obstacle_x, self.obstacle_y)).T)
        # RRT
        path_x, path_y = [start_x], [start_y]
        sx, sy = start_x, start_y
        while(True):
            self.count += 1 # Count searching times
            sample_x, sample_y = self.sampling(sx, sy, goal_x, goal_y)
            path_x.append(sample_x)
            path_y.append(sample_y)
            # Optimize path
            if self.count % self.optimize_len == 0:
                path_x, path_y = self.optimize(path_x, path_y)
            sx, sy = sample_x, sample_y

            if self.count >= self.search_MaxNum:
                print("Could not found path!\n")
                break

            if len(path_x) > 1600:
                print("The path is too long\n")
                break

            if (sample_x - goal_x) ** 2 + (sample_y - goal_y) ** 2 < self.dist_th ** 2:
                # while(len(path_x) > 30):
                #     for _ in range(int(len(path_x) / self.optimize_len)):
                #         path_x, path_y = self.optimize(path_x, path_y)
                print("Oh yeah, I found the path to the destination!\n")
                print("Length of path is ", len(path_x), "\n")
                break

        return path_x, path_y

    def sampling(self, start_x, start_y, goal_x, goal_y):
        theta0 = np.arctan2(goal_y - start_y, goal_x - start_x)
        if np.random.rand() > self.samplePro:
            theta = theta0
        else:
            theta = np.random.normal(theta0, 100)
        sample_x = start_x + self.step * np.cos(theta)
        sample_y = start_y + self.step * np.sin(theta)
        
        while (not self.obstacle_check(sample_x, sample_y)):
            theta = np.random.randint(-180, 180) * np.pi / 180
            sample_x = start_x + self.step * np.cos(theta)
            sample_y = start_y + self.step * np.sin(theta)

        return sample_x, sample_y

    def optimize(self, path_x, path_y):
        path_x_tem, path_y_tem = path_x[-self.optimize_len:], path_y[-self.optimize_len:]
        road_map = self.generate_roadmap(path_x_tem[1:-1], path_x_tem[1:-1], self.obstree)
        start_x, start_y = path_x_tem[0], path_y_tem[0]
        goal_x, goal_y = path_x_tem[-1], path_y_tem[-1]
        path_x_tem, path_y_tem = self.dijkstra_search(start_x, start_y, 
            goal_x, goal_y, road_map, path_x_tem[1:-1], path_y_tem[1:-1])
        path_x = path_x[:len(path_x) - self.optimize_len] + path_x_tem
        path_y = path_y[:len(path_y) - self.optimize_len] + path_y_tem
        return path_x, path_y

    def obstacle_check(self, sx, sy):
        dist = np.hypot(self.obstacle_x - sx, self.obstacle_y - sy)
        if np.min(dist) < self.avoid_dist + self.robot_size: # Too near to the obstacles
            # print("Obstacle!")
            return False
        else:
            return True

    def generate_roadmap(self, sample_x, sample_y, obstree):
        road_map = []
        nsample = len(sample_x)
        sampletree = KDTree(np.vstack((sample_x, sample_y)).T)

        for (i, ix, iy) in zip(range(nsample), sample_x, sample_y):
            distance, index = sampletree.query(np.array([ix, iy]), k=nsample)
            edges = []

            for ii in range(1, len(index)):
                nx = sample_x[index[ii]]
                ny = sample_y[index[ii]]

                # check collision
                if not self.check_obs(ix, iy, nx, ny, obstree):
                    edges.append(index[ii])

                if len(edges) >= self.KNN:
                    break

            road_map.append(edges)

        return road_map

    def check_obs(self, ix, iy, nx, ny, obstree):
        x = ix
        y = iy
        dx = nx - ix
        dy = ny - iy
        angle = math.atan2(dy, dx)
        dis = math.hypot(dx, dy)

        if dis > self.MAX_EDGE_LEN:
            return True

        step_size = self.robot_size + self.avoid_dist
        steps = round(dis / step_size)
        for i in range(steps):
            distance, index = obstree.query(np.array([x, y]))
            if distance <= self.robot_size + self.avoid_dist:
                return True
            x += step_size * math.cos(angle)
            y += step_size * math.sin(angle)

        # check for goal point
        distance, index = obstree.query(np.array([nx, ny]))
        if distance <= self.robot_size + self.avoid_dist:
            return True

        return False

    def dijkstra_search(self, start_x, start_y, goal_x, goal_y, road_map,
            sample_x, sample_y):
        path_x, path_y = [], []
        start = Node(start_x, start_y, 0.0, -1)
        goal = Node(goal_x, goal_y, 0.0, -1)

        openset, closeset = dict(), dict()
        openset[len(road_map)-2] = start

        path_found = True
        while True:
            if not openset:
                print("Cannot find path")
                path_found = False
                # TODO
                return sample_x, sample_y
                # break

            c_id = min(openset, key=lambda o: openset[o].cost)
            current = openset[c_id]

            if c_id == (len(road_map) - 1):
                # print("Goal is found!")
                goal.cost = current.cost
                goal.parent = current.parent
                break

            del openset[c_id]
            closeset[c_id] = current

            # expand
            for i in range(len(road_map[c_id])):
                n_id = road_map[c_id][i]
                dx = sample_x[n_id] - current.x
                dy = sample_y[n_id] - current.y
                d = math.hypot(dx, dy)
                node = Node(sample_x[n_id], sample_y[n_id],
                    current.cost + d, c_id)
                if n_id in closeset:
                    continue
                if n_id in openset:
                    if openset[n_id].cost > node.cost:
                        openset[n_id].cost = node.cost
                        openset[n_id].parent = c_id
                else:
                    openset[n_id] = node

        if path_found:
            path_x.append(goal.x)
            path_y.append(goal.y)
            parent = goal.parent
            while parent != -1:
                path_x.append(closeset[parent].x)
                path_y.append(closeset[parent].y)
                parent = closeset[parent].parent

        return path_x, path_y
