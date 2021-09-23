#include "optimizer/CHOMP_dynamic_trajectory.hpp"

namespace optimized_motion_planner {

CHOMPDynamicTrajectyory::CHOMPDynamicTrajectyory(const std::vector<Eigen::Vector3d>& trajectory_points, double end_time, double discretization, double quadrotor_speed) {
	this->trajectory_points_ = trajectory_points;
	this->end_time_ = end_time;
	this->discretization_ = discretization;
	this->quadrotor_speed_ = quadrotor_speed;
	this->set_start_time();
	this->calculate_duration_and_points_num_for_full_trajectory();
	this->update_trajectory_for_diff();
	this->fill_discretized_points();
	this->fill_trajectory();
}

CHOMPDynamicTrajectyory::CHOMPDynamicTrajectyory(const Eigen::Vector3d& start_point, const Eigen::Vector3d& end_point) {
	this->duration_ = optimized_motion_planner_utils::get_distance(start_point, end_point) / this->quadrotor_speed_;
	this->num_points_ = static_cast<int>(this->duration_ / this->discretization_);
}

void CHOMPDynamicTrajectyory::set_start_time() {
	this->start_time_ = this->end_time_ - optimized_motion_planner_utils::get_distance(this->trajectory_points_.front(), this->trajectory_points_.back())/this->quadrotor_speed_;
}

double CHOMPDynamicTrajectyory::calculate_time_by_index(int index) {
	// the index starts from 0
	return index * this->discretization_ + this->start_time_;
}

void CHOMPDynamicTrajectyory::update_trajectory_for_diff() {
	int diff_rule_length = optimized_motion_planner_utils::DIFF_RULE_LENGTH;
	this->start_extra_ = (diff_rule_length - 1) - this->start_index_origin; // 5
	this->end_extra_ = (diff_rule_length - 1) - ((this->num_points_ - 1) - this->end_index_origin_); // 5

	this->num_points_diff_ = this->num_points_ + this->start_extra_ + this->end_extra_;
	this->num_points_free_ = this->end_index_origin_ - this->start_index_origin + 1;
	this->start_index_ = diff_rule_length - 1; // start from 6, when original is start from 0, start_point and goal is fixed, not free.
	this->end_index_ = (this->num_points_diff_ - 1) - (diff_rule_length - 1); // end at num_points_free + 3
	this->duration_diff_ = (this->num_points_diff_ - 1) * this->discretization_;
}

void CHOMPDynamicTrajectyory::calculate_duration_and_points_num_for_full_trajectory() {
	double discretize_length = this->discretization_ * this->quadrotor_speed_;
	int count = 0;
	double duration = 0.0;
	int total_number_of_points = 0;
	Eigen::Vector3d last_point;

	for (auto path_point : this->trajectory_points_) {
		if (count == 0) {
			last_point = path_point;
			count += 1;
			continue;
		}
		double distance = optimized_motion_planner_utils::get_distance(last_point, path_point);
		duration += distance / this->quadrotor_speed_;
		if (distance < discretize_length) {
			total_number_of_points += 1;
			this->group_number_.push_back(1);
			last_point = path_point;
			count += 1;
			continue;
		}
		int number_of_interval_points = static_cast<int>(floor(distance/discretize_length));
		total_number_of_points += number_of_interval_points;
		this->group_number_.push_back(number_of_interval_points); // the number of group_number is vector(trajectory_points_) size -1
		last_point = path_point;
		count += 1;
	}

	this->num_points_ = total_number_of_points + 1;
	this->start_index_origin = 1;
	this->end_index_origin_ = this->num_points_ - 2;
	this->group_number_.back() += 1;
	this->duration_ = duration;
	int total_num_temp = 0;
	for (int num : this->group_number_) {
		total_num_temp += num;
		this->group_number_add_.push_back(total_num_temp);
	}
}

void CHOMPDynamicTrajectyory::fill_trajectory() {
	this->trajectory_.resize(this->num_points_diff_, this->num_joints_);
	for (int i=0; i<this->num_points_; i++) {
		for (int j=0; j<this->num_joints_; j++) {
			this->trajectory_.row(i+this->start_extra_)[j] = this->discretized_points_[i](j);
		}
	}
	for (int i=0; i<this->start_extra_; i++) {
		this->trajectory_.row(i) = this->trajectory_.row(this->start_extra_);
	}
	for (int i=this->num_points_diff_-this->end_extra_; i<this->num_points_diff_; i++) {
		this->trajectory_.row(i) = this->trajectory_.row(this->num_points_diff_-this->end_extra_-1);
	}
}

void CHOMPDynamicTrajectyory::fill_discretized_points() {
	this->discretized_points_.reserve(this->num_points_);
	for (int i=1; i<=this->num_points_; i++) {
		this->discretized_points_.push_back(this->calculate_position_by_index(i));
	}
}

Eigen::Vector3d CHOMPDynamicTrajectyory::calculate_position_by_index(int index) {
	// the index starts from 1
	int interval_index = 0;
	int group_size = static_cast<int>(this->group_number_add_.size());
	for (int i=0; i<group_size; i++) {
		if (index <= this->group_number_add_[i]) {
			interval_index = i;
			break;
		}
	}

	Eigen::Vector3d start_point = this->trajectory_points_[interval_index];
	Eigen::Vector3d end_point = this->trajectory_points_[interval_index+1];
	int start_index, end_index;
	if (interval_index == 0) {
		start_index = 1;
		end_index = this->group_number_add_[interval_index] + 1;
	} else if (interval_index == group_size-1) {
		start_index = this->group_number_add_[interval_index-1] +1;
		end_index = this->group_number_add_[interval_index];
	} else {
		start_index = this->group_number_add_[interval_index-1] + 1;
		end_index = this->group_number_add_[interval_index]+1;
	}

	double ratio = (index - start_index) / static_cast<double>(end_index - start_index);
	double x = (end_point(0) - start_point(0)) * ratio + start_point(0);
	double y = (end_point(1) - start_point(1)) * ratio + start_point(1);
	double z = (end_point(2) - start_point(2)) * ratio + start_point(2);
	Eigen::Vector3d index_point(x,y,z);

	return index_point;
}

void CHOMPDynamicTrajectyory::add_increments_to_trajectory(Eigen::MatrixXd::ColXpr increment, int joint_number, double scale) {
	this->trajectory_.block(start_index_, 0, this->num_points_free_, this->num_joints_).col(joint_number) += scale * increment;
}

}

