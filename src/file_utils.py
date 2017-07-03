#!/usr/bin/env python
import os
import csv
import rospy
from geometry_msgs.msg import Point

def write_pos(filename,pos):
	with open(filename, 'w') as f:
		f.write('%f %f %f\n'%(pos.x,pos.y,pos.z))

def read_pos(filename):
	while not os.path.isfile(filename):
		pass
	buffer_list = []
	while len(buffer_list) == 0:
		with open(filename, 'rb') as f:
			buffer_list = list(csv.reader(f, delimiter=' ', quotechar='"'))
	pos = buffer_list[0]
	x = float(pos[0])
	y = float(pos[1])
	z = float(pos[2])
	return Point(x,y,z)

