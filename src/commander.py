#!/usr/bin/env python
import copy
import rospy
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from box.msg import *
from box.srv import *
from FileComm import *

# ============================================
# Coordinate conversion
# ============================================

# Grid index to grid position
def index_to_grid(index_list):
	for index in index_list:
		x = index % grid.info.width
		y = (index - x)/grid.info.width
		yield Point(x,y,0)

# Grid position to grid index
def grid_to_index(points):
	for p in points:
		yield int(p.x+p.y*grid.info.width)

# Grid index to marker
def index_to_marker(index,label=None,color=None,id=None):
	for marker in pos_index_markers.markers:
		if marker.id == index:
			m = copy.deepcopy(marker)
			if not label is None:
				m.text = label
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
			return Point(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z)
	return None

# Map Coordinates to Index
def map_to_index(pos):
	min_dist_sqr = 1e12
	closest_marker_id = 0
	for marker in pos_index_markers.markers:
		dist_sqr = (marker.pose.position.x - pos.x)**2 + (marker.pose.position.y - pos.y)**2
		if dist_sqr < min_dist_sqr:
			min_dist_sqr = dist_sqr
			closest_marker_id = marker.id
	return closest_marker_id


# ============================================
# Message Callbacks
# ============================================

# Map Callback
def get_grid(msg):
	global grid
	grid = msg

# Position Marker Callback
def get_pos_index_markers(msg):
	global pos_index_markers
	pos_index_markers = msg


# ============================================
# Action Execution
# ============================================

def request_plan(grid_map,problem):
	try:
		rospy.wait_for_service(box_plan_service)
		box_plan	= rospy.ServiceProxy(box_plan_service, BoxPlan)
		resp1 = box_plan(grid_map,problem)
		return resp1.plan
	except rospy.ServiceException, e:
		rospy.loginfo("commander: Service call failed: %s"%e)

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
	grid_map = Grid()
	grid_map.width	= grid.info.width
	grid_map.height	= grid.info.height
	grid_map.data	= []
	for entry in grid.data:
		if entry == 0:
			grid_map.data.append(int(0))
		else:
			grid_map.data.append(int(1))

	rospy.loginfo("commander: Waiting for plan...")
	plan = request_plan(grid_map,problem)
	return plan


# ============================================
# Utils
# ============================================

def publish_markers(marker_pub, index_list, label_format="%d", label_offset=0, color=ColorRGBA(1,1,1,1)):
	marker_array = MarkerArray()
	for id,index in enumerate(index_list):
		label = label_format%(id+label_offset)
		marker_array.markers.append(index_to_marker(index,label=label,color=color,id=id))
	marker_pub.publish(marker_array)



# ============================================
# Main
# ============================================

rospy.init_node('commander')

# Getting parameters
box_plan_service		= rospy.get_param('/box_plan_service')
grid_topic				= rospy.get_param('/grid_topic')
grid_marker_topic		= rospy.get_param('/grid_marker_topic')
ini_robot_markers_topic	= rospy.get_param('/grid_markerini_robot_markers_topic')
ini_boxes_markers_topic	= rospy.get_param('/grid_markerini_boxes_markers_topic')
cur_robot_markers_topic	= rospy.get_param('/grid_markercur_robot_markers_topic')
cur_boxes_markers_topic	= rospy.get_param('/grid_markercur_boxes_markers_topic')
end_boxes_markers_topic	= rospy.get_param('/grid_markerend_boxes_markers_topic')
goal_pos_file_format	= rospy.get_param('/goal_pos_file_format')
current_pos_file_format	= rospy.get_param('/current_pos_file_format')
shared_dir				= rospy.get_param('/shared_dir')
num_robots				= rospy.get_param('~num_robots')

grid				= None
pos_index_markers	= None
robot_pos			= None

# Map and Marker Subscribers
sub_pos_markers		= rospy.Subscriber(grid_marker_topic, MarkerArray, get_pos_index_markers)
sub_grid			= rospy.Subscriber(grid_topic, OccupancyGrid, get_grid)

# Marker Array Publishers
ini_robot_markers	= rospy.Publisher(ini_robot_markers_topic, MarkerArray, queue_size=10,latch=True)
ini_boxes_markers	= rospy.Publisher(ini_boxes_markers_topic, MarkerArray, queue_size=10,latch=True)
cur_robot_markers	= rospy.Publisher(cur_robot_markers_topic, MarkerArray, queue_size=10,latch=True)
cur_boxes_markers	= rospy.Publisher(cur_boxes_markers_topic, MarkerArray, queue_size=10,latch=True)
end_boxes_markers	= rospy.Publisher(end_boxes_markers_topic, MarkerArray, queue_size=10,latch=True)

# Wait for map
rospy.loginfo("commander: Waiting for map...")
while grid is None:
	if rospy.is_shutdown():
		exit(0)

# Wait for markers
rospy.loginfo("commander: Waiting for markers...")
while pos_index_markers is None:
	if rospy.is_shutdown():
		exit(0)

# File names
goal_pos_file = {}
current_pos_file = {}
for i in range(num_robots):
	goal_pos_file[i]		= goal_pos_file_format % (shared_dir,i)
	current_pos_file[i]		= current_pos_file_format % (shared_dir,i)

# Initial robot positions
rospy.loginfo("commander: Waiting for robot positions...")
ini_robot = []
for i in range(num_robots):
	ini_robot.append(map_to_index(FileComm.read_pos(current_pos_file[i])))
publish_markers(ini_robot_markers, ini_robot, label_format="R%d", color=ColorRGBA(0,0,1,1))

while not rospy.is_shutdown():

	# Initial Box Positions
	while True:
		print "Enter the current box position indexes: ",
		try:
			s = raw_input()
			ini_boxes = map(int, s.split())
			break
		except KeyboardInterrupt:
			exit(0)
		except:
			print "Bad input, please provide space-separated integers."
		if rospy.is_shutdown():
			exit(0)
	publish_markers(ini_boxes_markers, ini_boxes, label_format="%c", label_offset=65, color=ColorRGBA(1,0,0,1))

	# Goal Box Positions
	while True:
		print "Enter the goal box position indexes: ",
		try:
			s = raw_input()
			end_boxes = map(int, s.split())
			break
		except KeyboardInterrupt:
			exit(0)
		except:
			print "Bad input, please provide space-separated integers."
		if rospy.is_shutdown():
			exit(0)
	publish_markers(end_boxes_markers, end_boxes, label_format="%c", label_offset=97, color=ColorRGBA(1,0,0,1))

	# ============================================
	# Planning
	# ============================================

	ini_robot_grid = list(index_to_grid(ini_robot))
	ini_boxes_grid = list(index_to_grid(ini_boxes))
	end_boxes_grid = list(index_to_grid(end_boxes))

	plan = solve_problem(ini_robot_grid, ini_boxes_grid, end_boxes_grid)

	if len(plan.steps) == 0:
		rospy.loginfo("commander: Planning Failed.")
		continue


	# ============================================
	# Execution
	# ============================================

	rospy.loginfo("commander: Plan execution...")

	# Execution loop
	for step_num, step in enumerate(plan.steps):
	
		rospy.loginfo("commander: Executing step %03d..." % step_num)

		# Goal Robot and Box Positions
		robot_indexes	= list(grid_to_index(step.robot_pos))
		boxes_indexes	= list(grid_to_index(step.box_pos))

		# Publishing current goal positions
		publish_markers(cur_robot_markers, robot_indexes, label_format="r%d", color=ColorRGBA(0,0.8,1,1))
		publish_markers(cur_boxes_markers, boxes_indexes, label_format="%c", label_offset=97, color=ColorRGBA(1,0.7,0,1))

		# Writing goal robot positions
		for i in range(num_robots):
			FileComm.write_pos(goal_pos_file[i],index_to_map(robot_indexes[i]))

		# Waiting for all robots to complete their actions
		for i in range(num_robots):
			FileComm.hang_while_exists(goal_pos_file[i])

		# Current robot positions
		ini_robot = []
		for i in range(num_robots):
			ini_robot.append(map_to_index(FileComm.read_pos(current_pos_file[i])))
		publish_markers(ini_robot_markers, ini_robot, label_format="R%d", color=ColorRGBA(0,0,1,1))

	rospy.loginfo("commander: Plan Executed Successfully.")

