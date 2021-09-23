#pragma once

#include <ros/ros.h>
#include "utils/types.hpp"
#include "utils/utility_functions.hpp"

#include <optimized_motion_planner/obstacle_info.h>

namespace optimized_motion_planner {

	class GenerateObstacle {
	public:
		GenerateObstacle(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
		~GenerateObstacle() = default;

	private:
		ros::NodeHandle nh_;
		ros::NodeHandle pnh_;

		ros::Publisher obstacle_pub_;
		ros::Publisher dynamic_obstacle_movement_pub_;

		void publish_dynamic_obstacle_movement();
		void publish_static_obstacle();
		optimized_motion_planner::obstacle_info get_dynamic_obstacle_message(bool operation, int dynamic_obstacle_id, double size, double velocity, const Eigen::Vector3d& position);
	};
}