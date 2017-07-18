#!/usr/bin/env python
import os
import csv
import rospy
from geometry_msgs.msg import Point

# File communication protocol
class FileComm(object):

	# Wait for file to be removed
	@staticmethod
	def hang_while_exists(filename):
		while os.path.isfile(filename):
			if rospy.is_shutdown():
				exit(0)

	# If file exists, remove it
	@staticmethod
	def remove_file(filename):
		if os.path.isfile(filename):
			os.remove(filename)

	# Write position
	@staticmethod
	def write_pos(filename,pos):
		with open(filename, 'w') as f:
			f.write('%f %f %f\n'%(pos.x,pos.y,pos.z))

	# Wait until position available, then read it
	@staticmethod
	def read_pos(filename):
		while not os.path.isfile(filename):
			if rospy.is_shutdown():
				exit(0)
		buffer_list = []
		while len(buffer_list) == 0:
			with open(filename, 'rb') as f:
				buffer_list = list(csv.reader(f, delimiter=' ', quotechar='"'))
			if rospy.is_shutdown():
				exit(0)
		pos = buffer_list[0]
		x = float(pos[0])
		y = float(pos[1])
		z = float(pos[2])
		return Point(x,y,z)

