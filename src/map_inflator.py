#!/usr/bin/env python
import rospy
import copy
import numpy as np
from nav_msgs.msg import OccupancyGrid

def inflate(map):

	# Initializing the map messages
	map_inflated	= copy.deepcopy(map)

	# Inflating map
	inflation_steps = int(inflation_ratio*box_size/map.info.resolution)
	rospy.loginfo("map_inflator: inflation_steps: %d" % inflation_steps)
	inp = np.array(map.data).reshape((map.info.height, map.info.width))
	inf = np.array(map.data).reshape((map.info.height, map.info.width))
	for j in range(len(inp)):
		for i in range(len(inp[j])):
			map_val = inp[j][i]
			if map_val > 50:
				index_js = max(j-inflation_steps,0)
				index_je = min(j+inflation_steps,len(inp)-1)
				for ij in range(index_js,index_je):
					index_is = max(i-inflation_steps,0)
					index_ie = min(i+inflation_steps,len(inp[j])-1)
					for ii in range(index_is,index_ie):
						inf[ij][ii] = map_val
	map_inflated.data = inf.reshape((map.info.width*map.info.height,1))
	map_inflated_pub.publish(map_inflated)


# Initializing node
rospy.init_node('map_inflator')

# Getting parameters
map_inflated_topic	= rospy.get_param('/map_inflated_topic','/map_inflated')
box_size			= rospy.get_param('/box_size', 0.45)
map_topic			= rospy.get_param('~map_topic','/map')
inflation_ratio		= rospy.get_param('~inflation_ratio',0.0)

# Setting up Publishers/Subscribers
map_inflated_pub	= rospy.Publisher(map_inflated_topic, OccupancyGrid, queue_size=10,latch=True)
subscriber			= rospy.Subscriber(map_topic, OccupancyGrid, inflate)

# Waiting for maps
rospy.spin()