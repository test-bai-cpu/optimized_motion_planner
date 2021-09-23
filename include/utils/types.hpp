#pragma once

#include <eigen3/Eigen/Eigen>

namespace optimized_motion_planner_utils {
	enum Obstacle_type {
		cube = 0,
		sphere = 1,
		dynamic_obstacle = 3
	};
}