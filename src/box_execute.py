#!/usr/bin/env python
import math
import copy
import rospy
import actionlib
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
from box.msg import *
from box.srv import *

# Problem Description Indexes
# ini_robot = [69]
# ini_boxes = [109]
# end_boxes = [57]
ini_robot = [69,68]
ini_boxes = [109,110]
end_boxes = [57,58]


# ============================================
# Coordinate conversion
# ============================================

# Grid index to grid position
def index_to_grid(index_list):
	for index in index_list:
		j = index % map.info.width
		i = (index - j)/map.info.width
		yield Point(i,j,0)

# Grid position to grid index
def grid_to_index(points):
	for p in points:
		yield int(p.x*map.info.width + p.y)

# Grid index to marker
def index_to_marker(index,text=None,color=None,id=None):
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

# Index to Map Coordinates
def index_to_map(index):
	for marker in pos_index_markers.markers:
		if marker.id == index:
			return marker.pose.position.x, marker.pose.position.y
	return None


# ============================================
# Message Callbacks
# ============================================

# Map Callback
def get_map(msg):
	global map
	map = msg

# Position Marker Callback
def get_pos_index_markers(msg):
	global pos_index_markers
	pos_index_markers = msg


# ============================================
# Action Execution
# ============================================

def send_subgoal(prev_xy, next_xy):
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.pose.position.x = next_xy[0]
	goal.target_pose.pose.position.y = next_xy[1]
	goal.target_pose.pose.position.z = 0
	quaternion = quaternion_from_euler(0, 0, math.atan2(next_xy[1] - prev_xy[1], next_xy[0] - prev_xy[0]))	
	goal.target_pose.pose.orientation.x = quaternion[0]
	goal.target_pose.pose.orientation.y = quaternion[1]
	goal.target_pose.pose.orientation.z = quaternion[2]
	goal.target_pose.pose.orientation.w = quaternion[3]
	client.send_goal(goal)
	client.wait_for_result(rospy.Duration.from_sec(15))

def request_plan(grid,problem):
	rospy.wait_for_service(box_plan_service)
	try:
		box_plan	= rospy.ServiceProxy(box_plan_service, BoxPlan)
		resp1 = box_plan(grid,problem)
		return resp1.plan
	except rospy.ServiceException, e:
		rospy.loginfo("box_execute: Service call failed: %s"%e)

def solve_problem(ini_robot_grid, ini_boxes_grid, end_boxes_grid):

	# Setting up the Problem message
	problem = Problem()
	problem.num_robots = len(ini_robot_grid)
	problem.num_boxes = len(ini_boxes_grid)
	for pt in ini_robot_grid:
		problem.initial_robot.append(pt)
	for pt in ini_boxes_grid:
		problem.initial_box.append(pt)
	for pt in end_boxes_grid:
		problem.final_box.append(pt)

	# Setting up the Map message
	grid = Grid()
	grid.width	= map.info.width
	grid.height	= map.info.height
	grid.data	= []
	for entry in map.data:
		if entry == 0:
			grid.data.append(int(0))
		else:
			grid.data.append(int(1))

	rospy.loginfo("box_execute: Waiting for plan..")
	plan = request_plan(grid,problem)
	return plan


# ============================================
# Main
# ============================================

rospy.init_node('box_execute')

# Getting parameters
box_plan_service		= rospy.get_param('/box_plan_service', 'box_plan')
grid_topic				= rospy.get_param('/grid_topic', '/grid')
grid_marker_topic		= rospy.get_param('/box_execute/grid_marker_topic', '/grid_marker')
ini_robot_markers_topic	= rospy.get_param('/grid_markerini_robot_markers_topic', '/ini_robot_markers')
ini_boxes_markers_topic	= rospy.get_param('/grid_markerini_boxes_markers_topic', '/ini_boxes_markers')
end_boxes_markers_topic	= rospy.get_param('/grid_markerend_boxes_markers_topic', '/end_boxes_markers')
cur_boxes_markers_topic	= rospy.get_param('/grid_markercur_boxes_markers_topic', '/cur_boxes_markers')
move_base_topic			= rospy.get_param('/box_execute/move_base_topic', 'move_base')

map = None
pos_index_markers = None

# Map and Marker Subscribers
sub_pos_markers		= rospy.Subscriber(grid_marker_topic, MarkerArray, get_pos_index_markers)
sub_map				= rospy.Subscriber(grid_topic, OccupancyGrid, get_map)

# Marker Array Publishers
ini_robot_markers	= rospy.Publisher(ini_robot_markers_topic, MarkerArray, queue_size=10,latch=True)
ini_boxes_markers	= rospy.Publisher(ini_boxes_markers_topic, MarkerArray, queue_size=10,latch=True)
end_boxes_markers	= rospy.Publisher(end_boxes_markers_topic, MarkerArray, queue_size=10,latch=True)
cur_boxes_markers	= rospy.Publisher(cur_boxes_markers_topic, MarkerArray, queue_size=10,latch=True)

# Wait for map
rospy.loginfo("box_execute: Waiting for map..")
while map is None:
	pass

# Wait for markers
rospy.loginfo("box_execute: Waiting for markers..")
while pos_index_markers is None:
	pass


# ============================================
# Publishing Problem Representation Markers
# ============================================

# Marker Arrays
ini_robot_marker_array	= MarkerArray()
ini_boxes_marker_array	= MarkerArray()
end_boxes_marker_array	= MarkerArray()

# Init robot Marker Arrays
for i,b in enumerate(ini_robot):
	text = "R%d"%i
	color=ColorRGBA(0,0,1,1)
	ini_robot_marker_array.markers.append(index_to_marker(b,text=text,color=color,id=i))

# Init Boxes Marker Arrays
for i,b in enumerate(ini_boxes):
	text = "%c"%(i+65)
	color=ColorRGBA(1,0,0,1)
	ini_boxes_marker_array.markers.append(index_to_marker(b,text=text,color=color,id=i))

# Final robot Marker Arrays
for i,b in enumerate(end_boxes):
	text = "%c"%(i+97)
	color=ColorRGBA(1,0,0,1)
	end_boxes_marker_array.markers.append(index_to_marker(b,text=text,color=color,id=i))

# Publishing Marker Arrays
ini_robot_markers.publish(ini_robot_marker_array)
ini_boxes_markers.publish(ini_boxes_marker_array)
end_boxes_markers.publish(end_boxes_marker_array)


# ============================================
# Indexes to Grid Coordinates
# ============================================

ini_robot_grid = list(index_to_grid(ini_robot))
ini_boxes_grid = list(index_to_grid(ini_boxes))
end_boxes_grid = list(index_to_grid(end_boxes))


# ============================================
# Planning
# ============================================

plan = solve_problem(ini_robot_grid, ini_boxes_grid, end_boxes_grid)


# ============================================
# Execution
# ============================================

rospy.loginfo("box_execute: Starting plan execution...")

# Action client setup
client = actionlib.SimpleActionClient(move_base_topic, MoveBaseAction)
client.wait_for_server()

# Execution loop
previous_goal	= (0,0)
current_goal	= (0,0)
for step in plan.steps:

	# Current/Goal Robot and Box Positions
	boxes_indexes	= list(grid_to_index(step.box_pos))
	robot_indexes	= list(grid_to_index(step.robot_pos))

	# Publishing current box positions
	cur_boxes_marker_array = MarkerArray()
	for i,b in enumerate(boxes_indexes):
		text = "%c"%(i+65)
		color=ColorRGBA(1,0.65,0,1)
		cur_boxes_marker_array.markers.append(index_to_marker(b,text=text,color=color,id=i))	
	cur_boxes_markers.publish(cur_boxes_marker_array)

	# Single Robot action execution
	current_goal = index_to_map(robot_indexes[0])
	send_subgoal(previous_goal, current_goal)
	previous_goal = current_goal

rospy.loginfo("box_execute: Plan Executed Successfully.")

rospy.spin()