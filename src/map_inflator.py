#!/usr/bin/env python
import rospy
import copy
from nav_msgs.msg import OccupancyGrid

def inflate(map):

	# Inflating map
	free_thresh = 50
	inflation_steps = int(inflation_ratio*box_size/map.info.resolution)
	buffer_list = list(map.data)
	rospy.loginfo("map_inflator: inflation_steps: %d" % inflation_steps)
	for y in range(map.info.height):
		for x in range(map.info.width):
			map_val = map.data[x+y*map.info.width]
			if map_val > free_thresh:
				index_xs = max(x-inflation_steps,0)
				index_xe = min(x+inflation_steps+1,map.info.width)
				for infx in range(index_xs,index_xe):
					index_ys = max(y-inflation_steps,0)
					index_ye = min(y+inflation_steps+1,map.info.height)
					for infy in range(index_ys,index_ye):
						buffer_list[infx+infy*map.info.width] = map_val

	# Initializing the map messages
	map_inflated = copy.deepcopy(map)
	map_inflated.data = tuple(buffer_list)
	map_inflated_pub.publish(map_inflated)


# Initializing node
rospy.init_node('map_inflator')

# Getting parameters
map_inflated_topic	= rospy.get_param('/map_inflated_topic','/map_inflated')
box_size			= rospy.get_param('/box_size', 0.4572)
map_topic			= rospy.get_param('~map_topic','/map')
inflation_ratio		= rospy.get_param('~inflation_ratio',0.0)

# Setting up Publishers/Subscribers
map_inflated_pub	= rospy.Publisher(map_inflated_topic, OccupancyGrid, queue_size=10,latch=True)
subscriber			= rospy.Subscriber(map_topic, OccupancyGrid, inflate)

# Waiting for maps
rospy.spin()