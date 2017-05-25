#include "ros/ros.h"
#include "box/BoxPlan.h"
#include "box/Map.h"
#include "box/Plan.h"
#include "box/Problem.h"
#include "geometry_msgs/Point.h"

bool plan(box::BoxPlan::Request  &req, box::BoxPlan::Response &res){
	ROS_INFO("Problem received:");
	printf("Map:\n");
	printf("map.width: %d\n",req.map.width);
	printf("map.height: %d\n",req.map.height);
	for(int i = 0; i < req.map.width; i++){
		for(int j = 0; j < req.map.height; j++)
			printf("%d",req.map.data[i*req.map.height+j]);
		printf("\n");
	}
	printf("Problem:\n");
	printf("num_rovers: %d\n",req.problem.num_rovers);
	printf("num_boxes: %d\n",req.problem.num_boxes);
	printf("initial_rover: \n");
	for(auto& pos : req.problem.initial_rover)
		printf("%.0f,%.0f\n",pos.x,pos.y);
	printf("initial_box: \n");
	for(auto& pos : req.problem.initial_box)
		printf("%.0f,%.0f\n",pos.x,pos.y);
	printf("final_box: \n");
	for(auto& pos : req.problem.final_box)
		printf("%.0f,%.0f\n",pos.x,pos.y);
	fflush(stdout);

	int plan_size = 2;

	res.plan.num_rovers = req.problem.num_rovers;
	res.plan.num_boxes = req.problem.num_boxes;

	res.plan.rover_pos.resize(plan_size*res.plan.num_rovers);
    res.plan.box_pos.resize(plan_size*res.plan.num_boxes);
	int ctr = 0;
	for(int i = 0; i < plan_size; i++){
		ctr = 0;	
		for(auto& pos : req.problem.initial_rover){
			geometry_msgs::Point pt;
			pt.x = pos.x;
			pt.y = pos.y;
			pt.z = 0.00;
			res.plan.rover_pos[res.plan.num_rovers*i+ctr++] = pt;
		}
		ctr = 0;
		for(auto& pos : req.problem.initial_box){
			geometry_msgs::Point pt;
			pt.x = pos.x;
			pt.y = pos.y;
			pt.z = 0.00;
			res.plan.box_pos[res.plan.num_boxes*i+ctr++] = pt;
		}
	}


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