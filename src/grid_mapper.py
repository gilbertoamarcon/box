#!/usr/bin/env python
import rospy
import math
import copy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray


def downsample(map):

	# Initializing the map message
	grid = copy.deepcopy(map)

	# Scaling factors
	scale = box_size/map.info.resolution
	grid.info.resolution = box_size
	grid.info.width = int(math.floor(map.info.width/scale))
	grid.info.height = int(math.floor(map.info.height/scale))
	rospy.loginfo("grid_mapper: Map size: (%d,%d)" % (map.info.width,map.info.height))
	rospy.loginfo("grid_mapper: Grid size: (%d,%d)" % (grid.info.width,grid.info.height))
	rospy.loginfo("grid_mapper: Ratio: %.3f" % scale)

	# Allocating buffers
	inp = np.array(map.data).reshape((map.info.width, map.info.height))
	out = np.zeros((grid.info.width, grid.info.height))

	# Downsampling
	for i in range(len(inp)):
		for j in range(len(inp[i])):
			oi = int(math.floor(i/scale))
			oj = int(math.floor(j/scale))
			if inp[i][j] > 50 or inp[i][j] == -1:
				out[oi][oj] = 100

	# Formatting message data
	grid.data = out.reshape((grid.info.width*grid.info.height,1))

	# Preparing marker array
	marker_array = MarkerArray()
	x_offset = map.info.resolution*map.info.width/2
	y_offset = map.info.resolution*map.info.height/2
	for i in range(len(out)):
		for j in range(len(out[i])):
			marker = Marker()
			marker.header.frame_id = "/map"
			marker.type = marker.TEXT_VIEW_FACING
			marker.action = marker.ADD
			marker.scale.x = box_size/2
			marker.scale.y = box_size/2
			marker.scale.z = box_size/2
			marker.color.a = 0.5
			marker.color.r = 0.5
			marker.color.g = 0.5
			marker.color.b = 0.5
			marker.pose.orientation.w = 1.0
			marker.pose.position.x = j*box_size - x_offset + box_size/2
			marker.pose.position.y = i*box_size - y_offset + box_size/2
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
box_size			= rospy.get_param('/grid_mapper/box_size', 0.4572)
map_topic			= rospy.get_param('/map_topic', '/map')
grid_topic			= rospy.get_param('/grid_topic', '/grid')
grid_marker_topic	= rospy.get_param('/grid_marker_topic', '/grid_marker')

# Setting up Publishers/Subscribers
marker_pub	= rospy.Publisher(grid_marker_topic, MarkerArray, queue_size=10,latch=True)
grid_pub	= rospy.Publisher(grid_topic, OccupancyGrid, queue_size=10,latch=True)
subscriber	= rospy.Subscriber(map_topic, OccupancyGrid, downsample)

# Waiting for maps
rospy.spin()