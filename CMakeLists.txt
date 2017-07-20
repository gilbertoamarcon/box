cmake_minimum_required(VERSION 2.8.3)
project(box)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	genmsg
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
	Grid.msg
	Plan.msg
	Problem.msg
	Step.msg
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
	LIBRARIES box_planner_optimal
	CATKIN_DEPENDS roscpp rospy message_runtime std_msgs geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########


add_library(box_planner_optimal
	lib/Closed.cpp
	lib/Map.cpp
	lib/Open.cpp
	lib/Pos.cpp
	lib/Search.cpp
	lib/State.cpp
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

link_directories(lib)

add_executable(box_plan_optimal src/box_plan_optimal.cpp)
add_dependencies(box_plan_optimal box_generate_messages_cpp)
target_link_libraries(box_plan_optimal ${catkin_LIBRARIES} box_planner_optimal)

install(TARGETS box_planner_optimal
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/
		DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})