#!/usr/bin/env python
import rospy
import os
import re
from geometry_msgs.msg import Point
from box.msg import *
from box.srv import *

verbose = False
box_plan_service=""
problem_csv=""
solution=""

def gen_problem_csv(req):

	# Map sze
	w = req.grid.width
	h = req.grid.height

	# Map data
	data = [" "]*w*h
	for y in range(h):
		for x in range(w):
			if req.grid.data[x+y*w] == 0:
				data[x+y*w] = " "
			if req.grid.data[x+y*w] == 1:
				data[x+y*w] = "#"

	# Initial Robot Positions
	for i,r in enumerate(req.problem.initial_robot):
		x = int(r.x)
		y = int(r.y)
		data[x+y*w] = str(i)

	# Initial Box Positions
	for i,r in enumerate(req.problem.initial_box):
		x = int(r.x)
		y = int(r.y)
		data[x+y*w] = chr(65+i)

	# Final Box Positions
	for i,r in enumerate(req.problem.final_box):
		x = int(r.x)
		y = int(r.y)
		data[x+y*w] = chr(97+i)

	# Parsing to filebuffer
	file_buffer = ""
	for y in range(h):
		buffer_list = []
		for x in range(w):
			buffer_list.append(data[x+y*w])			
		file_buffer += ','.join(buffer_list)+"\n"

	# Writing to file
	with open(problem_csv,'wb') as f:
		f.write(file_buffer)
	
def read_plan(req):

	plan = Plan()
	plan.num_robots = req.problem.num_robots
	plan.num_boxes = req.problem.num_boxes

	rospy.loginfo("box_plan: Reading plan file %s" % solution)

	# Flags and buffers
	initial_step = True
	pre_timestamp = -1
	robots = {}
	boxes = {}
	action = []

	# Initial robot positions
	for i,r in enumerate(req.problem.initial_robot):
		robots['r'+str(i)] = r

	# Initial box positions
	for i,b in enumerate(req.problem.initial_box):
		boxes[chr(97+i)] = b

	# Main parsing loop
	with open(solution, 'rb') as f:

		# For each line of file
		for line in f:

			# Flags
			new_step = False
			new_time_step = False

			# Move action
			move = re.findall('(\d+\.\d+):\s+\(\w+\s+(\w+)\s+p_(\d+)_(\d+)\s+p_(\d+)_(\d+)\)\s+\[(\d+\.*\d*)\]', line)
			if len(move) > 0:
				action = list(move[0])
				for i in range(2,6):
					action[i] = int(action[i])
				ptr = Point()
				ptr.x = action[4];
				ptr.y = action[5];
				ptr.z = 0.00;
				robots[action[1]] = ptr
				new_step = True

			# Push action
			move = re.findall('(\d+\.\d+):\s+\(\w+\s+(\w+)\s+(\w+)\s+p_(\d+)_(\d+)\s+p_(\d+)_(\d+)\s+p_(\d+)_(\d+)\)\s+\[(\d+\.*\d*)\]', line)
			if len(move) > 0:
				action = list(move[0])
				for i in range(3,9):
					action[i] = int(action[i])
				ptr = Point()
				ptr.x = action[5];
				ptr.y = action[6];
				ptr.z = 0.00;
				ptb = Point()
				ptb.x = action[7];
				ptb.y = action[8];
				ptb.z = 0.00;
				robots[action[1]] = ptr
				boxes[action[2]] = ptb
				new_step = True

			# New action step
			if new_step:
				if int(float(action[0])) != pre_timestamp:
					new_time_step = True
				pre_timestamp = int(float(action[0]))

			# Pushing to plan
			if new_time_step or initial_step:
				step = Step()
				for k in sorted(robots.keys()):
					step.robot_pos.append(robots[k]);
				for k in sorted(boxes.keys()):
					step.box_pos.append(boxes[k]);
				plan.steps.append(step);
				initial_step = False

	return plan

def plan(req):

	# Generating problem description file
	gen_problem_csv(req)

	# Planning
	os.system("%s %s %s"%(planner_script,problem_csv,solution))

	# Reading plan from file
	return read_plan(req)

if __name__ == "__main__":

	#  Initializing node
	rospy.init_node('box_plan')

	# Getting parameters
	box_plan_service	= rospy.get_param('/box_plan_service', 'box_plan')
	verbose				= rospy.get_param('~verbose', False)
	problem_csv			= rospy.get_param('~problem_csv', '$(find box)/temp/problem_csv.csv')
	planner_script		= rospy.get_param('~planner_script', '$(find box)/scripts/planning.sh')
	solution			= rospy.get_param('~solution', '$(find box)/temp/solution')

	# Initializing planning service
	service = rospy.Service(box_plan_service, BoxPlan, plan)

	# Ready message
	rospy.loginfo("box_plan: Ready to box-plan.")

	# Waiting for requests
	rospy.spin()
