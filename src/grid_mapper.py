#!/usr/bin/env python
import rospy
import sys
import math
import copy
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray

def downsample(map):

	# Scaling factors
	scale					= box_size/map.info.resolution
	grid					= copy.deepcopy(map)
	grid.info.resolution	= box_size
	grid.info.width			= int(math.ceil(map.info.width/scale))
	grid.info.height		= int(math.ceil(map.info.height/scale))
	rospy.loginfo("grid_mapper: Map size: (%d,%d)" % (map.info.width,map.info.height))
	rospy.loginfo("grid_mapper: Grid size: (%d,%d)" % (grid.info.width,grid.info.height))
	rospy.loginfo("grid_mapper: Ratio: %.3f" % scale)

	# Allocating buffers
	out = [0]*grid.info.width*grid.info.height

	# Downsampling
	for y in range(map.info.height):
		for x in range(map.info.width):
			oy = int(math.floor(y/scale))
			ox = int(math.floor(x/scale))
			map_val = map.data[x+y*map.info.width]
			free_thresh = 50
			if map_val > free_thresh or map_val == -1:
				out[ox+oy*grid.info.width] = map_val

	# Formatting message data
	grid.data = tuple(out)

	# Preparing marker array
	marker_array = MarkerArray()
	for y in range(grid.info.height):
		for x in range(grid.info.width):
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
			marker.pose.position.y += y*box_size + box_size/2
			marker.pose.position.x += x*box_size + box_size/2
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
box_size			= rospy.get_param('/box_size', 0.4572)
grid_topic			= rospy.get_param('/grid_topic','/box/grid')
grid_marker_topic	= rospy.get_param('/grid_marker_topic','/box/grid_marker')
map_frame_id		= rospy.get_param('~map_frame_id','/map')

# Setting up Publishers/Subscribers
marker_pub			= rospy.Publisher(grid_marker_topic, MarkerArray, queue_size=10,latch=True)
grid_pub			= rospy.Publisher(grid_topic, OccupancyGrid, queue_size=10,latch=True)
subscriber			= rospy.Subscriber(map_inflated_topic, OccupancyGrid, downsample)

# Waiting for maps
rospy.spin()