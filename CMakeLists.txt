cmake_minimum_required(VERSION 2.8.3)
project(box)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	geometry_msgs
	message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
add_message_files(
	FILES
	Map.msg
	Plan.msg
	Problem.msg
)

# Generate services in the 'srv' folder
add_service_files(
	FILES
	BoxPlan.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
	LIBRARIES box_planner
	CATKIN_DEPENDS roscpp rospy message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(box_planner
	src/planner/Closed.cpp
	src/planner/main.cpp
	src/planner/Map.cpp
	src/planner/Open.cpp
	src/planner/Pos.cpp
	src/planner/Search.cpp
	src/planner/State.cpp
	 src/planner/timeIt.cpp
 )

add_executable(box_plan src/box_plan.cpp)
target_link_libraries(box_plan ${catkin_LIBRARIES})
add_dependencies(box_plan box)
