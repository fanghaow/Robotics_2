from vision import Vision
from action import Action
from debug import Debugger
from zss_debug_pb2 import Debug_Msgs
from rrt import RRT
from path_planner import Planner
from velocity_planner import StupidTrack, FB_Tracking
import time

isFirstInit = True
count = 0

if __name__ == '__main__':
	vision = Vision()
	action = Action()
	debugger = Debugger()
	Controler = FB_Tracking()
	# Controler = StupidTrack()
	while True:
		# 1. path planning & velocity planning
		# Do something
		resolution = 100
		arrive_th = 75
		robot0 = vision.blue_robot[0]
		if isFirstInit:
			goal_x, goal_y = -2400, -1500
			lastVw = 0
			isFirstInit = False
		if robot0.visible and robot0.id == 0 and robot0.x > -4501:
			start_x, start_y, orientation = int(robot0.x), int(robot0.y), robot0.orientation
			Vx, Vy, Vw = robot0.vel_x, 0, lastVw
		else:
			print("[WARN]: No robot0 found!")
			continue
		
		if (start_x - goal_x) ** 2 + (start_y - goal_y) ** 2 < arrive_th ** 2:
			action.sendCommand(vx=0, vy=0, vw=0)
			print("Arrive the goal!")
			count += 1
			if count >= 10:
				action.sendCommand(vx=0, vy=0, vw=0)
				break
			goal_x *= -1
			goal_y *= -1
			continue

		start_t = time.time()
		# planner = RRT() 
		planner = Planner(resolution=resolution)
		try:
			path_x, path_y = planner.plan(vision=vision, start_x=start_x, start_y=start_y, goal_x=goal_x, goal_y=goal_y)
		except:
			print("[ERROR]: A_star doesn't work!")
			continue
		end_t = time.time()
		# print("It takes me %f seconds to accomplish single [plan] step\n" %(end_t - start_t))
		# 2. send command TODO
		pos_vel = [start_x, start_y, orientation, Vx, Vw]
		length = 4 # length of local goal in path
		if len(path_x) > length:
			goal = [path_x[-length], path_y[-length]]
		else:
			goal = [path_x[0], path_y[0]]
		# goal = [goal_x, goal_y]
		Vx, Vy, lastVw = Controler.plan(pos_vel, goal)
		print("I send: [%f %f %f]" %(Vx, Vy, lastVw))
		action.sendCommand(vx=Vx*100, vy=Vy*100, vw=lastVw)

		# 3. draw debug msg
		package = Debug_Msgs()
		debugger.draw_circle(package, x=goal[0], y=goal[1], radius=100)
		debugger.draw_circle(package, x=goal_x, y=goal_y, radius=150)
		for i in range(len(path_x) - 1):
			debugger.draw_line(package, path_x[i], path_y[i], path_x[i+1], path_y[i+1])
		debugger.send(package)

		time.sleep(0.01)
