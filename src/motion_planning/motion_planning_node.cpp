#include <ros/ros.h>

#include "motion_planning/RRTX.hpp"

int main(int argc, char** argv) {
	std::cout << "Hello! motion planning" << std::endl;
	ros::init(argc, argv, "motion_planner");
	optimized_motion_planner::RRTX RRTX(ros::NodeHandle(""), ros::NodeHandle("~"));
	ros::spin();
	return 0;
}