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
	State::map = new Map(int(req.map.width),int(req.map.height),&req.map.data[0]);

	// Initial and goal states
	State::start	= new State();
	State::goal		= new State();
	for(auto& pos : req.problem.initial_box)
		State::start->boxes.push_back(Pos(int(pos.x),int(pos.y)));
	for(auto& pos : req.problem.initial_rover)
		State::start->robots.push_back(Pos(int(pos.x),int(pos.y)));
	for(auto& pos : req.problem.final_box)
		State::goal->boxes.push_back(Pos(int(pos.x),int(pos.y)));

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
	ROS_INFO("Ready to box-plan.");

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