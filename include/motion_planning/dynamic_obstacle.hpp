#pragma once

#include <ros/ros.h>

#include <Eigen/Dense>
#include <queue>
#include <memory>

#include "motion_planning/RRTX_node.hpp"
#include "motion_planning/obstacle.hpp"

namespace optimized_motion_planner {

class DynamicObstacle {
public:
	DynamicObstacle(int dynamic_obstacle_id, double dynamic_obstacle_size, double dynamic_obstacle_velocity,
				    const Eigen::Vector3d& start_position, double start_time, double planning_horizon_time, double quadrotor_radius):
					dynamic_obstacle_id_(dynamic_obstacle_id), dynamic_obstacle_size_(dynamic_obstacle_size),
					dynamic_obstacle_velocity_(dynamic_obstacle_velocity), start_position_(start_position),
					start_time_(start_time), planning_horizon_time_(planning_horizon_time), quadrotor_radius_(quadrotor_radius) {}
	~DynamicObstacle() = default;

	int get_dynamic_obstacle_id() const {
		return this->dynamic_obstacle_id_;
	};

	double get_dynamic_obstacle_size() const {
		return this->dynamic_obstacle_size_;
	}

	bool get_if_move() const {
		return this->if_move_;
	}

	void set_if_move(bool if_move) {
		this->if_move_ = if_move;
	};

	bool get_if_add() const {
		return this->if_add_;
	}

	void set_if_add(bool if_add) {
		this->if_add_ = if_add;
	};

	void set_second_position(const Eigen::Vector3d& second_position) {
		this->second_position_ = second_position;
	};

	Eigen::Vector3d predict_path(double node_time);
	bool check_if_node_inside_dynamic_obstacle(const std::shared_ptr<RRTXNode>& node);

private:
	int dynamic_obstacle_id_{0};
	double dynamic_obstacle_size_;
	double dynamic_obstacle_velocity_;
	Eigen::Vector3d start_position_;
	double start_time_;
	double planning_horizon_time_;
	double quadrotor_radius_{0.5};
	Eigen::Vector3d second_position_;
	bool if_move_{false};
	bool if_add_{false};
};

}