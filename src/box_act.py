#!/usr/bin/env python
import math
import rospy
import actionlib
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from file_utils import *

# ============================================
# Message Callbacks
# ============================================

# Robot Position Callback
def get_robot_pos(msg):
	global robot_pos
	robot_pos = msg.pose.pose.position

# ============================================
# Action Execution
# ============================================

def send_subgoal(prev_xy, next_xy):
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.pose.position.x = next_xy.x
	goal.target_pose.pose.position.y = next_xy.y
	goal.target_pose.pose.position.z = 0
	quaternion = quaternion_from_euler(0, 0, math.atan2(next_xy.y - prev_xy.y, next_xy.x - prev_xy.x))	
	goal.target_pose.pose.orientation.x = quaternion[0]
	goal.target_pose.pose.orientation.y = quaternion[1]
	goal.target_pose.pose.orientation.z = quaternion[2]
	goal.target_pose.pose.orientation.w = quaternion[3]
	client.send_goal(goal)
	client.wait_for_result(rospy.Duration.from_sec(15))

# ============================================
# Main
# ============================================

rospy.init_node('box_act')

# Getting parameters
robot_id				= rospy.get_param('/box_act/robot_id')
robot_pos_topic			= rospy.get_param('/box_act/robot_pos_topic')
move_base_topic			= rospy.get_param('/box_act/move_base_topic')
goal_pos_file_format	= rospy.get_param('/goal_pos_file_format')
current_pos_file_format	= rospy.get_param('/current_pos_file_format')
shared_dir				= rospy.get_param('/shared_dir')

# File names
goal_pos_file			= goal_pos_file_format % (shared_dir,robot_id)
current_pos_file		= current_pos_file_format % (shared_dir,robot_id)

# Cleaning files from previous execution
remove_file(goal_pos_file)
remove_file(current_pos_file)

# Wait for robot pos
robot_pos			= None
sub_robot_pos		= rospy.Subscriber(robot_pos_topic, PoseWithCovarianceStamped, get_robot_pos)
rospy.loginfo("box_act: Waiting for robot position..")
while robot_pos is None:
	pass

# Current robot position
write_pos(current_pos_file,robot_pos)

# Action client setup
client = actionlib.SimpleActionClient(move_base_topic, MoveBaseAction)
client.wait_for_server()

# Execution loop
previous_goal	= Point(0,0,0)
current_goal	= Point(0,0,0)
action_counter = 0
while True:

	# Reading Action
	# rospy.loginfo("box_act: Waiting for action..")
	current_goal = read_pos(goal_pos_file)

	# Action Execution
	# rospy.loginfo("box_act: Starting action execution...")
	send_subgoal(previous_goal, current_goal)
	previous_goal = current_goal
	remove_file(goal_pos_file)
	rospy.loginfo("box_act: Action %d Executed Successfully."%action_counter)
	action_counter += 1

	# Current robot position
	# write_pos(current_pos_file,robot_pos)

rospy.spin()
