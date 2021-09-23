#pragma once

#include <iostream>
#include <string>
#include <Eigen/Dense>

#include "utils/types.hpp"

namespace optimized_motion_planner {

class Obstacle {
public:
	Obstacle(const std::string& name, optimized_motion_planner_utils::Obstacle_type type, bool operation, double size, const Eigen::Vector3d& position):
				name_(name), type_(type), operation_(operation), size_(size), position_(position) {}
	~Obstacle() = default;

	std::string get_name() const {
		return this->name_;
	};

	optimized_motion_planner_utils::Obstacle_type get_type() const {
		return this->type_;
	};

	bool get_operation() const {
		return this->operation_;
	};

	double get_size() const {
		return this->size_;
	};

	Eigen::Vector3d get_position() const {
		return this->position_;
	};

protected:
	std::string name_;
	optimized_motion_planner_utils::Obstacle_type type_;
	bool operation_;
	double size_;
	Eigen::Vector3d position_;
};

}