#include "ros/ros.h"
#include "../include/Search.hpp"
#include "box/BoxPlan.h"
#include "box/Grid.h"
#include "box/Plan.h"
#include "box/Problem.h"
#include "box/Step.h"
#include "geometry_msgs/Point.h"

bool verbose = false;
string box_plan_service;

bool plan(box::BoxPlan::Request  &req, box::BoxPlan::Response &res){

	// Initializing the map
	State::map = new Map(int(req.grid.width),int(req.grid.height),&req.grid.data[0]);

	// Initial and goal states
	State::start	= new State();
	State::goal		= new State();
	for(auto& pos : req.problem.initial_box)
		State::start->boxes.push_back(Pos(int(pos.y),int(pos.x)));
	for(auto& pos : req.problem.initial_robot)
		State::start->robots.push_back(Pos(int(pos.y),int(pos.x)));
	for(auto& pos : req.problem.final_box)
		State::goal->boxes.push_back(Pos(int(pos.y),int(pos.x)));

	ROS_INFO("box_plan_optimal: Problem received.");
	if(verbose)
		State::display_world(State::start);

	// Running and taking execution time
	ROS_INFO("box_plan_optimal: Planning started...");
	Search::search();
	ROS_INFO("box_plan_optimal: Planning finished.");

	// Presenting stats on screen
	ROS_INFO("box_plan_optimal: %d expanded nodes.",Search::num_exp_nodes);
	ROS_INFO("box_plan_optimal: %d actions.",int(Search::plan.size()));
	ROS_INFO("box_plan_optimal: %f seconds.",Search::planning_time);

	// Setting up return value (plan) and printing results
	if(Search::num_exp_nodes > 0){

		if(verbose)
			Search::print_plan();
		res.plan.num_robots = req.problem.num_robots;
		res.plan.num_boxes = req.problem.num_boxes;

		// For all plan steps
		while(!Search::plan.empty()){
			box::Step step;

			// For all robot positions
			for(auto& pos : Search::plan.top().robots){
				geometry_msgs::Point pt;
				pt.x = pos.j;
				pt.y = pos.i;
				pt.z = 0.00;
				step.robot_pos.push_back(pt);
			}

			// For all box positions
			for(auto& pos : Search::plan.top().boxes){
				geometry_msgs::Point pt;
				pt.x = pos.j;
				pt.y = pos.i;
				pt.z = 0.00;
				step.box_pos.push_back(pt);
			}

			// Push step into plan
			res.plan.steps.push_back(step);
			Search::plan.pop();
		}
	}
	else
		ROS_INFO("box_plan_optimal: Plan failed.\n");
	ROS_INFO("box_plan_optimal: Ready to box-plan.");

	// Cleanup
	delete State::map;

	return true;
}

int main(int argc, char **argv){

	// Initializing node
	ros::init(argc, argv, "box_plan_optimal");

	// Getting parameters
	ros::NodeHandle ns;
	ns.param<string>("/box_plan_service", box_plan_service,"box_plan");

	ros::NodeHandle np("~");
	np.param<int>("max_iterations", Search::max_iterations,1e9);
	np.param<float>("time_lim_secs", Search::time_lim_secs,1e2);
	np.param<float>("epsilon", Search::epsilon,1.0);
	np.param<bool>("verbose", verbose,false);

	// Initializing planning service
	ros::ServiceServer service = ns.advertiseService(box_plan_service, plan);

	// Ready message
	ROS_INFO("box_plan_optimal: Ready to box-plan.");

	// Waiting for requests
	ros::spin();

	return 0;
}