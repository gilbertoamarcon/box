#!/usr/bin/env python

import csv 
import sys
import rospy
import sys
from geometry_msgs.msg import Point
import rospkg
from box.msg import Map
from box.msg import Problem
from box.msg import Plan
from box.srv import *

def request_plan(map,problem):
	rospy.wait_for_service('box_plan')
	try:
		box_plan = rospy.ServiceProxy('box_plan', BoxPlan)
		resp1 = box_plan(map,problem)
		return resp1.plan
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def load_map(map_filename):

	# Initializing the map message
	map = Map()

	# Reading file
	f = open(map_filename,'rb')
	reader = csv.reader(f)
	data = list(reader)

	# Parsing data
	map.height = len(data)
	max_h = 0
	for row in data:
		for entry in row:
			if entry == '1':
				map.data.append(int(1))
			else:
				map.data.append(int(0))
		if len(row) > max_h:
			max_h = len(row)
	map.width = max_h

	# Closing file
	f.close()

	return map

def print_plan(plan):
	plan_size = len(plan.rover_pos)/plan.num_rovers
	print "num_rovers: %d"%plan.num_rovers
	print "num_boxes: %d"%plan.num_boxes
	print "plan_size: %d"%plan_size
	for i in range(0,plan_size):
		sys.stdout.write("Plan step %3d "%i)
		sys.stdout.write("Rovers: ")
		rs = (i+0)*plan.num_rovers
		re = (i+1)*plan.num_rovers
		for r in plan.rover_pos[rs:re]:
			sys.stdout.write("%d,%d "%(r.x,r.y))
		sys.stdout.write("Boxes: ")
		rs = (i+0)*plan.num_boxes
		re = (i+1)*plan.num_boxes
		for r in plan.box_pos[rs:re]:
			sys.stdout.write("%d,%d "%(r.x,r.y))
		print ""

if __name__ == "__main__":

	# File parameters
	rospack = rospkg.RosPack()
	map_filename = rospack.get_path('box')+"/res/map.csv"

	map = load_map(map_filename)

	# Initializing the problem message
	problem = Problem()

	# Number of rovers and boxes
	problem.num_rovers = 2
	problem.num_boxes = 2

	# Initial Rover Positions
	problem.initial_rover.append(Point(5,7,0))
	problem.initial_rover.append(Point(4,7,0))

	# Initial Box Positions
	problem.initial_box.append(Point(2,2,0))
	problem.initial_box.append(Point(6,2,0))

	# Goal Box Positions
	problem.final_box.append(Point(6,2,0))
	problem.final_box.append(Point(2,2,0))
	
	# Requesting plan service
	plan = request_plan(map,problem)

	print_plan(plan)