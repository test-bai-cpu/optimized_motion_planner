#include <ros/ros.h>

#include "example/generate_obstacle.hpp"

int main(int argc, char** argv) {
	ros::init(argc, argv, "generate_obstacle");
	optimized_motion_planner::GenerateObstacle generate_obstacle(ros::NodeHandle(""), ros::NodeHandle("~"));
	ros::spin();
	return 0;
}