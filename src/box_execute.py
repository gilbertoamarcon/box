#!/usr/bin/env python
import math
import copy
import rospy
import actionlib
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from box.msg import Map
from box.msg import Problem
from box.msg import Plan
from box.msg import Step
from box.srv import *

def send_angle_goal(prev_xy, next_xy):
	goal.target_pose.pose.position.x = prev_xy[0]
	goal.target_pose.pose.position.y = prev_xy[1]
	goal.target_pose.pose.position.z = 0
	quaternion = quaternion_from_euler(0, 0, math.atan2(next_xy[1] - prev_xy[1], next_xy[0] - prev_xy[0]))
	goal.target_pose.pose.orientation.x = quaternion[0]
	goal.target_pose.pose.orientation.y = quaternion[1]
	goal.target_pose.pose.orientation.z = quaternion[2]
	goal.target_pose.pose.orientation.w = quaternion[3]
	client.send_goal(goal)
	client.wait_for_result(rospy.Duration.from_sec(5))

	goal.target_pose.pose.position.x = next_xy[0]
	goal.target_pose.pose.position.y = next_xy[1]
	goal.target_pose.pose.position.z = 0
	quaternion = quaternion_from_euler(0, 0, math.atan2(next_xy[1] - prev_xy[1], next_xy[0] - prev_xy[0]))	
	goal.target_pose.pose.orientation.x = quaternion[0]
	goal.target_pose.pose.orientation.y = quaternion[1]
	goal.target_pose.pose.orientation.z = quaternion[2]
	goal.target_pose.pose.orientation.w = quaternion[3]
	client.send_goal(goal)
	client.wait_for_result(rospy.Duration.from_sec(5))

# ini_robots = [69]
# ini_boxes = [109]
# end_boxes = [57]
ini_robots = [69,68]
ini_boxes = [109,110]
end_boxes = [57,58]

def index_to_pos(index_list):
	for index in index_list:
		j = index % box_map.info.width
		i = (index - j)/box_map.info.width
		yield Point(i,j,0)

def pos_to_index(points):
	for p in points:
		yield int(p.x*box_map.info.width + p.y)

def request_plan(map,problem):
	rospy.wait_for_service('box_plan')
	try:
		box_plan = rospy.ServiceProxy('box_plan', BoxPlan)
		resp1 = box_plan(map,problem)
		return resp1.plan
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e


def solve_problem(robotStart, boxStart, boxEnd):
	problem = Problem()
	problem.num_robots = len(robotStart)
	problem.num_boxes = len(boxStart)

	for pt in robotStart:
		problem.initial_robot.append(pt)
	for pt in boxStart:
		problem.initial_box.append(pt)
	for pt in boxEnd:
		problem.final_box.append(pt)

	# Initializing the Map message
	plan_map = Map()

	# Parsing  data
	plan_map.width = box_map.info.width
	plan_map.height = box_map.info.height
	plan_map.data = []
	for entry in box_map.data:
			if entry == 0:
				plan_map.data.append(int(0))
			else:
				plan_map.data.append(int(1))


	print "Waiting for plan.."
	plan = request_plan(plan_map,problem)
	# print_plan(plan)
	return plan

def get_map(msg):
	global box_map
	box_map = msg

def get_pos_index_markers(msg):
	global pos_index_markers
	pos_index_markers = msg

def get_marker(index,text=None,color=None,id=None):
	for marker in pos_index_markers.markers:
		if marker.id == index:
			m = copy.deepcopy(marker)
			if not text is None:
				m.text = text
			if not color is None:
				m.color = color
			if not id is None:
				m.id = id
			return m
	return None

def get_world_pos(index):
	for marker in pos_index_markers.markers:
		if marker.id == index:
			return marker.pose.position.x, marker.pose.position.y
	return None

rospy.init_node('boxnav')

box_map = None
pos_index_markers = None

sub_pos_markers = rospy.Subscriber('/boxmap_marker', MarkerArray, get_pos_index_markers)
sub_map = rospy.Subscriber('/boxmap', OccupancyGrid, get_map)

# Marker Array Publishers
ini_robots_markers = rospy.Publisher('/ini_robots_markers', MarkerArray, queue_size=10,latch=True)
ini_boxes_markers = rospy.Publisher('/ini_boxes_markers', MarkerArray, queue_size=10,latch=True)
end_boxes_markers = rospy.Publisher('/end_boxes_markers', MarkerArray, queue_size=10,latch=True)
cur_boxes_markers = rospy.Publisher('/cur_boxes_markers', MarkerArray, queue_size=10,latch=True)

# Wait for box_map
print "Waiting for box_map.."
while box_map is None:
	pass

# Wait for markers
print "Waiting for markers.."
while pos_index_markers is None:
	pass

# Marker Arrays
ini_robots_marker_array = MarkerArray()
ini_boxes_marker_array = MarkerArray()
end_boxes_marker_array = MarkerArray()

# Init robot Marker Arrays
for i,b in enumerate(ini_robots):
	text = "R%d"%i
	color=ColorRGBA(0,0,1,1)
	ini_robots_marker_array.markers.append(get_marker(b,text=text,color=color,id=i))

# Init Boxes Marker Arrays
for i,b in enumerate(ini_boxes):
	text = "%c"%(i+65)
	color=ColorRGBA(1,0,0,1)
	ini_boxes_marker_array.markers.append(get_marker(b,text=text,color=color,id=i))

# Final robot Marker Arrays
for i,b in enumerate(end_boxes):
	text = "%c"%(i+97)
	color=ColorRGBA(1,0,0,1)
	end_boxes_marker_array.markers.append(get_marker(b,text=text,color=color,id=i))

# Publishing Marker Arrays
ini_robots_markers.publish(ini_robots_marker_array)
ini_boxes_markers.publish(ini_boxes_marker_array)
end_boxes_markers.publish(end_boxes_marker_array)

ini_robots_pts = list(index_to_pos(ini_robots))
ini_boxes_pts = list(index_to_pos(ini_boxes))
end_boxes_pts = list(index_to_pos(end_boxes))

plan = solve_problem(ini_robots_pts, ini_boxes_pts, end_boxes_pts)
print plan

# Action client
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
client.wait_for_server()
goal = MoveBaseGoal()
goal.target_pose.header.frame_id = "map"

print "GO!:"
previous_goal = (0,0)
current_goal = (0,0)
for step in plan.steps:

	# print list(pos_to_index(plan.box_pos))[g]
	# Publishing current box positions
	# cur_boxes_marker_array = MarkerArray()
	# for i,b in enumerate(list(pos_to_index(plan.box_pos))[g]):
	# 	text = "%c"%(i+65)
	# 	color=ColorRGBA(0,1,0,1)
	# 	end_boxes_marker_array.markers.append(get_marker(b,text=text,color=color,id=i))	
	# cur_boxes_markers.publish(cur_boxes_marker_array)

	robot_indexes = list(pos_to_index(step.robot_pos))

	# Single Robot action execution
	current_goal = get_world_pos(robot_indexes[0])
	send_angle_goal(previous_goal, current_goal)
	previous_goal = current_goal

rospy.spin()