#!/usr/bin/env python
import rospy
import sys
import math
import copy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray

def downsample(map):

	# Scaling factors
	scale = box_size/map.info.resolution
	grid = copy.deepcopy(map)
	grid.info.resolution = box_size
	grid.info.width = int(math.ceil(map.info.width/scale))
	grid.info.height = int(math.ceil(map.info.height/scale))
	rospy.loginfo("grid_mapper: Map size: (%d,%d)" % (map.info.width,map.info.height))
	rospy.loginfo("grid_mapper: Grid size: (%d,%d)" % (grid.info.width,grid.info.height))
	rospy.loginfo("grid_mapper: Ratio: %.3f" % scale)

	# Allocating buffers
	inp = np.array(map.data).reshape((map.info.height, map.info.width))
	out = np.zeros((grid.info.height, grid.info.width))

	# Downsampling
	for j in range(len(inp)):
		for i in range(len(inp[j])):
			oi = int(math.floor(i/scale))
			oj = int(math.floor(j/scale))
			map_val = inp[j][i]
			if map_val > 50 or map_val == -1:
				out[oj][oi] = 100

	# Formatting message data
	grid.data = out.reshape((grid.info.width*grid.info.height,1))

	# Preparing marker array
	marker_array = MarkerArray()
	for j in range(len(out)):
		for i in range(len(out[j])):
			marker = Marker()
			marker.header.frame_id = map_frame_id
			marker.type = marker.TEXT_VIEW_FACING
			marker.action = marker.ADD
			marker.scale.x = box_size/3
			marker.scale.y = box_size/3
			marker.scale.z = box_size/3
			marker.color.a = 0.25
			marker.color.r = 0.50
			marker.color.g = 0.50
			marker.color.b = 0.50
			marker.pose = copy.deepcopy(grid.info.origin)
			marker.pose.position.x += i*box_size + box_size/2
			marker.pose.position.y += j*box_size + box_size/2
			marker_array.markers.append(marker)

	# Marker IDs
	id = 0
	for marker in marker_array.markers:
		marker.id = id
		marker.text = str(id);
		id += 1

	# Publishing map
	grid_pub.publish(grid)

	# Publish the MarkerArray
	marker_pub.publish(marker_array)

# Initializing node
rospy.init_node('grid_mapper')

# Getting parameters
map_inflated_topic	= rospy.get_param('/map_inflated_topic','/map_inflated')
box_size			= rospy.get_param('/box_size', 0.45)
grid_topic			= rospy.get_param('/grid_topic','/box/grid')
grid_marker_topic	= rospy.get_param('/grid_marker_topic','/box/grid_marker')
map_frame_id		= rospy.get_param('~map_frame_id','/map')

# Setting up Publishers/Subscribers
marker_pub			= rospy.Publisher(grid_marker_topic, MarkerArray, queue_size=10,latch=True)
grid_pub			= rospy.Publisher(grid_topic, OccupancyGrid, queue_size=10,latch=True)
subscriber			= rospy.Subscriber(map_inflated_topic, OccupancyGrid, downsample)

# Waiting for maps
rospy.spin()