#pragma once

#include <vector>
#include <Eigen/Dense>
#include <iostream>

#include <ros/ros.h>

#include "utils/utility_functions.hpp"

namespace optimized_motion_planner {
class CHOMPDynamicTrajectyory {
public:
	CHOMPDynamicTrajectyory(const std::vector<Eigen::Vector3d>& trajectory_points, double end_time, double discretization, double quadrotor_speed);
	CHOMPDynamicTrajectyory(const Eigen::Vector3d& start_point, const Eigen::Vector3d& end_point);
	~CHOMPDynamicTrajectyory() = default;

	Eigen::MatrixXd get_trajectory() const {
		return this->trajectory_;
	}

	int get_num_points() const {
		return this->num_points_;
	};

	int get_num_points_diff() const {
		return this->num_points_diff_;
	};

	int get_num_points_free() const {
		return this->num_points_free_;
	};

	int get_start_extra() const {
		return this->start_extra_;
	};

	int get_end_extra() const {
		return this->end_extra_;
	};

	int get_start_index() const {
		return this->start_index_;
	};

	int get_end_index() const {
		return this->end_index_;
	};

	int get_duration() const {
		return this->duration_;
	};

	double get_discretization() const {
		return this->discretization_;
	}

	Eigen::Vector3d get_position_by_index(int index) const {
		Eigen::Vector3d position(this->trajectory_(index + this->start_extra_, 0),
								 this->trajectory_(index + this->start_extra_, 1),
								 this->trajectory_(index + this->start_extra_, 2));
		return position;
	};

	Eigen::MatrixXd::ColXpr get_joint_trajectory(int joint) {
		return this->trajectory_.col(joint);
	}

	Eigen::MatrixXd::RowXpr get_point_trajectory(int point) {
		return this->trajectory_.row(point);
	}

	Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> get_free_trajectory_block(){
		return this->trajectory_.block(start_index_, 0, this->num_points_free_, this->num_joints_);
	}

	void add_increments_to_trajectory(Eigen::MatrixXd::ColXpr increment, int joint_number, double scale);

	double calculate_time_by_index(int index);
private:
	double start_time_;
	double end_time_;

	std::vector<Eigen::Vector3d> trajectory_points_;
	Eigen::MatrixXd trajectory_;
	double duration_;
	double duration_diff_;
	int num_points_;
	int num_points_diff_;
	int num_points_free_;
	int num_joints_{3};
	double discretization_{0.5}; // every interval is 0.5s
	double quadrotor_speed_{1.0};
	std::vector<int> group_number_;
	std::vector<int> group_number_add_;
	std::vector<Eigen::Vector3d> discretized_points_;

	int start_index_origin;
	int end_index_origin_;
	int start_index_;
	int end_index_;
	int start_extra_{0};
	int end_extra_{0};
	void calculate_duration_and_points_num_for_full_trajectory();
	void update_trajectory_for_diff();
	void fill_trajectory();
	void fill_discretized_points();
	Eigen::Vector3d calculate_position_by_index(int index);
	void set_start_time();
};




}