#!/usr/bin/env python
import rospy
import math
import copy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray

BOX_RES = 0.4572

def downsample(inp_map):

	# Initializing the map message
	out_map = copy.deepcopy(inp_map)

	# Scaling factors
	scale = BOX_RES/inp_map.info.resolution
	out_map.info.resolution = BOX_RES
	out_map.info.width = int(math.floor(inp_map.info.width/scale))
	out_map.info.height = int(math.floor(inp_map.info.height/scale))
	print "Ratio: %.3f" % scale
	print "Origiinal Map width: %.3f" % inp_map.info.width
	print "Origiinal Map height: %.3f" % inp_map.info.height
	print "New map width: %.3f" % out_map.info.width
	print "New map height: %.3f" % out_map.info.height

	# Allocating buffers
	inp = np.array(inp_map.data).reshape((inp_map.info.width, inp_map.info.height))
	out = np.zeros((out_map.info.width, out_map.info.height))

	# Downsampling
	for i in range(len(inp)):
		for j in range(len(inp[i])):
			oi = int(math.floor(i/scale))
			oj = int(math.floor(j/scale))
			if inp[i][j] > 50 or inp[i][j] == -1:
				out[oi][oj] = 100

	# Formatting message data
	out_map.data = out.reshape((out_map.info.width*out_map.info.height,1))

	# Preparing marker array
	markerArray = MarkerArray()
	x_offset = inp_map.info.resolution*inp_map.info.width/2
	y_offset = inp_map.info.resolution*inp_map.info.height/2
	for i in range(len(out)):
		for j in range(len(out[i])):
			marker = Marker()
			marker.header.frame_id = "/map"
			marker.type = marker.TEXT_VIEW_FACING
			marker.action = marker.ADD
			marker.scale.x = BOX_RES/2
			marker.scale.y = BOX_RES/2
			marker.scale.z = BOX_RES/2
			marker.color.a = 0.5
			marker.color.r = 0.5
			marker.color.g = 0.5
			marker.color.b = 0.5
			marker.pose.orientation.w = 1.0
			marker.pose.position.x = j*BOX_RES - x_offset + BOX_RES/2
			marker.pose.position.y = i*BOX_RES - y_offset + BOX_RES/2
			markerArray.markers.append(marker)

	# Marker IDs
	id = 0
	for marker in markerArray.markers:
		marker.id = id
		marker.text = str(id);
		id += 1

	# Publishing map
	boxmap_pub.publish(out_map)

	# Publish the MarkerArray
	marker_pub.publish(markerArray)


rospy.init_node('grid_mapper')

marker_pub = rospy.Publisher('/boxmap_marker', MarkerArray, queue_size=10,latch=True)
boxmap_pub = rospy.Publisher('/boxmap', OccupancyGrid, queue_size=10,latch=True)
subscriber = rospy.Subscriber('/map', OccupancyGrid, downsample)

rospy.spin()