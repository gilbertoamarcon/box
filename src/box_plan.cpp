#include "ros/ros.h"
#include "../include/Search.hpp"
#include "box/BoxPlan.h"
#include "box/Map.h"
#include "box/Plan.h"
#include "box/Problem.h"
#include "geometry_msgs/Point.h"

bool plan(box::BoxPlan::Request  &req, box::BoxPlan::Response &res){

	// Search params
	Search::max_iterations = 1e9;
	Search::time_lim_secs = 1e2;
	Search::epsilon = 1.0;

	// Initializing the map
	State::map = new Map();
	State::map->cols = req.map.width;
	State::map->rows = req.map.height;
	State::map->map = new int[State::map->cols*State::map->rows];
	for(int i = 0; i < State::map->rows; i++)
		for(int j = 0; j < State::map->cols; j++)
			State::map->map[State::map->coordinate2index(i,j)] = req.map.data[State::map->coordinate2index(i,j)];

	// Initial and goal states
	char init[BUFFER_SIZE];
	char final[BUFFER_SIZE];
	init[0] = '\0';
	final[0] = '\0';
	for(auto& pos : req.problem.initial_box)
		sprintf(init,"%s%d,%d,",init,int(pos.x),int(pos.y));
	sprintf(init,"%s:",init);
	for(auto& pos : req.problem.initial_rover)
		sprintf(init,"%s%d,%d,",init,int(pos.x),int(pos.y));
	for(auto& pos : req.problem.final_box)
		sprintf(final,"%s%d,%d,",final,int(pos.x),int(pos.y));
	sprintf(final,"%s:",final);
	State::start	= new State(init);
	State::goal		= new State(final);
	ROS_INFO("Problem received:");
	State::display_world(State::start);


	// Running and taking execution time
	ROS_INFO("Planning started...");
	Search::search();
	ROS_INFO("Planning finished.");

	// Presenting stats on screen
	ROS_INFO("%d expanded nodes.",Search::num_exp_nodes);
	ROS_INFO("%d actions.",int(Search::plan.size()));
	ROS_INFO("%f seconds.",Search::planning_time);

	// Setting up return value (plan) and printing results
	if(Search::num_exp_nodes > 0){
		Search::print_plan();
		res.plan.num_rovers = req.problem.num_rovers;
		res.plan.num_boxes = req.problem.num_boxes;
		res.plan.rover_pos.resize(int(Search::plan.size())*res.plan.num_rovers);
		res.plan.box_pos.resize(int(Search::plan.size())*res.plan.num_boxes);
		int ctr = 0;
		int i = 0;
		while(!Search::plan.empty()){
			ctr = 0;
			for(auto& pos : Search::plan.top().robots){
				geometry_msgs::Point pt;
				pt.x = pos.i;
				pt.y = pos.j;
				pt.z = 0.00;
				res.plan.rover_pos[res.plan.num_rovers*i+ctr++] = pt;
			}
			ctr = 0;
			for(auto& pos : Search::plan.top().boxes){
				geometry_msgs::Point pt;
				pt.x = pos.i;
				pt.y = pos.j;
				pt.z = 0.00;
				res.plan.box_pos[res.plan.num_boxes*i+ctr++] = pt;
			}
			Search::plan.pop();
			i++;
		}
	}
	else
		ROS_INFO("Plan failed.\n");

	// Cleanup
	delete State::map;

	return true;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "box_planner");
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("box_plan", plan);
	ROS_INFO("Ready to box-plan.");
	ros::spin();
	return 0;
}