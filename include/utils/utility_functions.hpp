#pragma once

#include <Eigen/Dense>
#include <random>
namespace optimized_motion_planner_utils {

static const int DIFF_RULE_LENGTH = 7;

// the differentiation rules (centered at the center)
static const double DIFF_RULES[3][DIFF_RULE_LENGTH] = {
		{ 0, 0, -2 / 6.0, -3 / 6.0, 6 / 6.0, -1 / 6.0, 0 },                       // velocity
		{ 0, -1 / 12.0, 16 / 12.0, -30 / 12.0, 16 / 12.0, -1 / 12.0, 0 },         // acceleration
		{ 0, 1 / 12.0, -17 / 12.0, 46 / 12.0, -46 / 12.0, 17 / 12.0, -1 / 12.0 }  // jerk
};

inline double get_distance(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2) {
	return static_cast<double>(std::sqrt((point1 - point2).squaredNorm()));
}

inline double get_distance_2d(const Eigen::Vector2d& point1, const Eigen::Vector2d& point2) {
	return static_cast<double>(std::sqrt((point1 - point2).squaredNorm()));
}

inline double get_random_double()
{
	std::default_random_engine seed;
	std::uniform_real_distribution<> uniform(0.0, 1.0);
	return uniform(seed);
}

}