#!/usr/bin/env python
import math
import rospy
import actionlib
from time import sleep
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from FileComm import *

# ============================================
# Message Callbacks
# ============================================

# Robot Position Callback
def get_robot_pos(msg):
	global robot_pos
	global robot_pos_cov
	robot_pos = msg.pose.pose.position
	robot_pos_cov = max(msg.pose.covariance)

# ============================================
# Action Execution
# ============================================

def send_subgoal(prev_xy, next_xy):
	global map_frame_id
	global action_exec_timeout
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = map_frame_id
	goal.target_pose.pose.position.x = next_xy.x
	goal.target_pose.pose.position.y = next_xy.y
	goal.target_pose.pose.position.z = 0
	quaternion = quaternion_from_euler(0, 0, math.atan2(next_xy.y - prev_xy.y, next_xy.x - prev_xy.x))	
	goal.target_pose.pose.orientation.x = quaternion[0]
	goal.target_pose.pose.orientation.y = quaternion[1]
	goal.target_pose.pose.orientation.z = quaternion[2]
	goal.target_pose.pose.orientation.w = quaternion[3]
	client.send_goal(goal)
	client.wait_for_result(rospy.Duration.from_sec(action_exec_timeout))

	twist = Twist()
	for i in range(6):
		twist.linear.x = +0.1
		cmd_vel_pub.publish(twist)
		sleep(0.250)
	for i in range(6):
		twist.linear.x = -0.1
		cmd_vel_pub.publish(twist)
		sleep(0.250)

# ============================================
# Main
# ============================================

rospy.init_node('supervisor')

# Getting parameters
goal_pos_file_format	= rospy.get_param('/goal_pos_file_format')
current_pos_file_format	= rospy.get_param('/current_pos_file_format')
shared_dir				= rospy.get_param('/shared_dir')
robot_id				= rospy.get_param('~robot_id')
robot_pos_topic			= rospy.get_param('~robot_pos_topic')
move_base_topic			= rospy.get_param('~move_base_topic')
map_frame_id			= rospy.get_param('~map_frame_id')
robot_pos_cov_limit		= rospy.get_param('~robot_pos_cov_limit')
localize_rotate_vel		= rospy.get_param('~localize_rotate_vel')
action_exec_timeout		= rospy.get_param('~action_exec_timeout')

# File names
goal_pos_file			= goal_pos_file_format % (shared_dir,robot_id)
current_pos_file		= current_pos_file_format % (shared_dir,robot_id)

# Cleaning files from previous execution
FileComm.remove_file(goal_pos_file)
FileComm.remove_file(current_pos_file)

# Wait for acurate robot pos
robot_pos			= None
robot_pos_cov		= 1e9
sub_robot_pos		= rospy.Subscriber(robot_pos_topic, PoseWithCovarianceStamped, get_robot_pos)
cmd_vel_pub			= rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rospy.loginfo("supervisor: Waiting for acurate robot position...")
while robot_pos is None or robot_pos_cov > robot_pos_cov_limit:
	twist = Twist()
	twist.angular.z = localize_rotate_vel
	cmd_vel_pub.publish(twist)
	rospy.loginfo("supervisor: Current pose covariance: % 12.3f"%robot_pos_cov)
	if rospy.is_shutdown():
		exit(0)

# Current robot position
FileComm.write_pos(current_pos_file,robot_pos)

# Action client setup
client = actionlib.SimpleActionClient(move_base_topic, MoveBaseAction)
client.wait_for_server()

# Execution loop
previous_goal	= Point(0,0,0)
current_goal	= Point(0,0,0)
while not rospy.is_shutdown():

	# Reading Action
	rospy.loginfo("supervisor: Waiting for action command...")
	current_goal = FileComm.read_pos(goal_pos_file)

	# Action Execution
	rospy.loginfo("supervisor: Executing action...")
	send_subgoal(previous_goal, current_goal)
	previous_goal = current_goal

	# Current robot position
	FileComm.write_pos(current_pos_file,robot_pos)
	FileComm.remove_file(goal_pos_file)
	rospy.loginfo("supervisor: Action Executed Successfully.")

