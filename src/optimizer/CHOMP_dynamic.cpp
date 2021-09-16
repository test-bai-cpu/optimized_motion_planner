#include "optimizer/CHOMP_dynamic.hpp"

namespace optimized_motion_planner {

CHOMPDynamic::CHOMPDynamic(double collision_threshold, double planning_time_limit, int max_iterations, int max_iterations_after_collision_free,
		 double learning_rate, double obstacle_cost_weight, double dynamic_obstacle_cost_weight, double dynamic_collision_factor,
		 double smoothness_cost_weight, double smoothness_cost_velocity, double smoothness_cost_acceleration, double smoothness_cost_jerk,
		 double ridge_factor, double min_clearence, double joint_update_limit, double quadrotor_radius,
		 const std::shared_ptr<ChompTrajectory>& trajectory, const std::map<std::string, std::shared_ptr<Obstacle>>& obstacle_map,
		 const std::map<int, std::shared_ptr<Human>>& human_map, int chomp_path_file_num
):  collision_threshold_(collision_threshold),
	planning_time_limit_(planning_time_limit),
	max_iterations_(max_iterations),
	max_iterations_after_collision_free_(max_iterations_after_collision_free),
	learning_rate_(learning_rate),
	obstacle_cost_weight_(obstacle_cost_weight),
	dynamic_obstacle_cost_weight_(dynamic_obstacle_cost_weight),
	dynamic_collision_factor_(dynamic_collision_factor),
	smoothness_cost_weight_(smoothness_cost_weight),
	smoothness_cost_velocity_(smoothness_cost_velocity),
	smoothness_cost_acceleration_(smoothness_cost_acceleration),
	smoothness_cost_jerk_(smoothness_cost_jerk),
	ridge_factor_(ridge_factor),
	min_clearence_(min_clearence),
	joint_update_limit_(joint_update_limit),
	quadrotor_radius_(quadrotor_radius),
	chomp_path_file_num_(chomp_path_file_num){
	//ROS_INFO("Start in the chomp");
	this->full_trajectory_ = trajectory;
	this->obstacle_map_ = obstacle_map;
	this->human_map_ = human_map;
	this->initialize();
	bool res = this->optimize();
	if (!res) ROS_WARN("Failed to find a collision free chomp solution");
	this->convert_matrix_to_trajectory_points_vector();
	this->export_potential_data(true);
	this->export_potential_data(false);
	this->export_potential_gradient_data(true);
	this->export_potential_gradient_data(false);
	if (chomp_cost_file.is_open()) {
		chomp_cost_file.close();
	}
}

CHOMPDynamic::~CHOMPDynamic() = default;

void CHOMPDynamic::check_params() {
	std::cout << "#collision_threshold: " << this->collision_threshold_ << std::endl;
	std::cout << "#planning_time_limit: " << this->planning_time_limit_ << std::endl;
	std::cout << "#max_iterations: " << this->max_iterations_ << std::endl;
	std::cout << "#max_iterations_after_collision_free: " << this->max_iterations_after_collision_free_ << std::endl;

	std::cout << "#learning_rate: " << this->learning_rate_ << std::endl;
	std::cout << "#obstacle_cost_weight: " << this->obstacle_cost_weight_ << std::endl;
	std::cout << "#dynamic_obstacle_cost_weight: " << this->dynamic_obstacle_cost_weight_ << std::endl;
	std::cout << "#dynamic_collision_factor: " << this->dynamic_collision_factor_ << std::endl;
	std::cout << "#smoothness_cost_weight: " << this->smoothness_cost_weight_ << std::endl;

	std::cout << "#smoothness_cost_velocity: " << this->smoothness_cost_velocity_ << std::endl;
	std::cout << "#smoothness_cost_acceleration: " << this->smoothness_cost_acceleration_ << std::endl;
	std::cout << "#smoothness_cost_jerk: " << this->smoothness_cost_jerk_ << std::endl;
	std::cout << "#ridge_factor: " << this->ridge_factor_ << std::endl;
	std::cout << "#min_clearence: " << this->min_clearence_ << std::endl;
	std::cout << "#joint_update_limit: " << this->joint_update_limit_ << std::endl;

}

void CHOMPDynamic::initialize() {
	//ROS_INFO("Chomp: Initialize");
	this->num_vars_all_ = this->full_trajectory_->get_num_points_diff(); // actual points + 10
	this->num_vars_free_ = this->full_trajectory_->get_num_points_free(); // actual points - 2
	this->num_vars_origin_ = this->full_trajectory_->get_num_points(); // actual points
	this->free_vars_start_ = this->full_trajectory_->get_start_index();
	this->free_vars_end_ = this->full_trajectory_->get_end_index();

	//ROS_INFO("#######Num of vars, actual points are: ");
	//std::cout <<"##" << this->num_vars_origin_ << std::endl;
	// get joint cost
	std::vector<double> derivative_costs(3);
	double joint_cost = 1.0;
	derivative_costs[0] = joint_cost * this->smoothness_cost_velocity_;
	derivative_costs[1] = joint_cost * this->smoothness_cost_acceleration_;
	derivative_costs[2] = joint_cost * this->smoothness_cost_jerk_;
	this->joint_costs_.reserve(this->num_joints_);
	double max_cost_scale = 0.0;

	for (int i = 0; i < this->num_joints_; i++) {
		this->joint_costs_.push_back(std::make_shared<ChompCost>(this->full_trajectory_, derivative_costs, this->ridge_factor_));
		double cost_scale = this->joint_costs_[i]->getMaxQuadCostInvValue();
		if (max_cost_scale < cost_scale) max_cost_scale = cost_scale;
	}

	for (int i = 0; i < this->num_joints_; i++) {
		this->joint_costs_[i]->scale(max_cost_scale);
	}

	// allocate memory for matrices:
	this->smoothness_increments_ = Eigen::MatrixXd::Zero(this->num_vars_free_, this->num_joints_);
	this->collision_increments_ = Eigen::MatrixXd::Zero(this->num_vars_free_, this->num_joints_);
	this->dynamic_collision_increments_ = Eigen::MatrixXd::Zero(this->num_vars_free_, this->num_joints_);
	this->final_increments_ = Eigen::MatrixXd::Zero(this->num_vars_free_, this->num_joints_);
	this->smoothness_derivative_ = Eigen::VectorXd::Zero(this->num_vars_all_);

	this->optimized_trajectory_.resize(this->num_vars_origin_);
	this->collision_point_pos_.resize(this->num_vars_all_);
	this->collision_point_vel_.resize(this->num_vars_all_);
	this->collision_point_acc_.resize(this->num_vars_all_);
	this->collision_point_vel_mag_.resize(this->num_vars_all_);
	this->collision_point_potential_.resize(this->num_vars_all_);
	this->collision_point_potential_gradient_.resize(this->num_vars_all_);

	this->dynamic_collision_point_potential_.resize(this->num_vars_all_);
	this->dynamic_collision_point_potential_gradient_.resize(this->num_vars_all_);
	//this->jacobian_ = Eigen::MatrixXd::Zero(3, 1);
	//this->jacobian_pseudo_inverse_ = Eigen::MatrixXd::Zero(1, 3);
	//this->jacobian_jacobian_tranpose_ = Eigen::MatrixXd::Zero(3, 3);

	this->collision_free_iteration_ = 0;
	this->last_improvement_iteration_ = -1;
}

bool Chomp::optimize() {
	ros::WallTime start_time = ros::WallTime::now();
	std::string file_name = "chomp_cost_" + std::to_string(this->chomp_path_file_num_) + ".txt";
	this->chomp_cost_file.open(file_name);
	for (this->iteration_=0; this->iteration_ < this->max_iterations_; this->iteration_++) {
		//std::cout << "The iteration number is: " << this->iteration_ << std::endl;
		perform_forward_kinematics();
		double c_cost = this->get_collision_cost();
		double d_cost = 0;
		if (this->human_map_.size() > 0) {
			d_cost = this->get_dynamic_collision_cost();
		}
		double s_cost = this->get_smoothness_cost();
		double cost = c_cost + d_cost + s_cost;
		this->chomp_cost_file << c_cost << " " << d_cost << " " << s_cost << " " << cost << "\n";
		if (this->iteration_ == 0 || cost < this->best_trajectory_cost_) {
			this->best_trajectory_ = this->full_trajectory_->get_trajectory();
			this->best_trajectory_cost_ = cost;
			this->last_improvement_iteration_ = this->iteration_;
		}

		this->calculate_smoothness_increments();
		this->calculate_collision_increments();
		if (this->human_map_.size() > 0) {
			this->calculate_dynamic_collision_increments();
		}
		this->calculate_total_increments();
		this->add_increments_to_trajectory();

		if (!this->filter_mode_ && c_cost < this->collision_threshold_) this->is_collsion_free_ = true;
		if (this->is_collsion_free_) this->collision_free_iteration_ += 1;

		// break conditions
		if (this->collision_free_iteration_ > this->max_iterations_after_collision_free_) break;
		if ((ros::WallTime::now() - start_time).toSec() > this->planning_time_limit_)
		{
			ROS_WARN("Breaking out early due to time limit constraints.");
			break;
		}
	}

	//ROS_INFO("Terminated after %d iterations, using path from iteration %d", this->iteration_, this->last_improvement_iteration_);
	//ROS_INFO("Optimization core finished in %f sec", (ros::WallTime::now() - start_time).toSec());
	//ROS_INFO_STREAM("Time per iteration " << (ros::WallTime::now() - start_time).toSec() / (this->iteration_ * 1.0));

	return is_collsion_free_;
}

void Chomp::get_collision_point_pos() {
	int start_extra = this->full_trajectory_->get_start_extra();
	int end_extra = this->full_trajectory_->get_end_extra();
	for (int i=0; i<this->num_vars_origin_; i++) {
		this->collision_point_pos_[i+start_extra] = this->full_trajectory_->get_position_by_index(i);
	}
	for (int i=0; i<start_extra; i++) {
		this->collision_point_pos_[i] = this->collision_point_pos_[start_extra];
	}
	for (int i=this->num_vars_all_-end_extra; i<this->num_vars_all_; i++) {
		this->collision_point_pos_[i] = this->collision_point_pos_[this->num_vars_all_-end_extra-1];
	}
}

void Chomp::perform_forward_kinematics() {
	double inv_time = 1.0 / this->full_trajectory_->get_discretization();
	double inv_time_sq = inv_time * inv_time;
	this->get_collision_point_pos();
	this->is_collsion_free_ = true;

	int start = this->free_vars_start_;
	int end = this->free_vars_end_;
	if (this->iteration_ == 0) {
		start = 0;
		end = this->num_vars_all_ - 1;
	}

	for (int i = start; i <= end; i++) {
		this->collision_point_potential_[i] = this->get_potential(this->collision_point_pos_[i]);
		this->collision_point_potential_gradient_[i] = this->get_distance_gradient(this->collision_point_pos_[i]);
		this->dynamic_collision_point_potential_[i] = this->get_dynamic_potential(this->collision_point_pos_[i], i);
		this->dynamic_collision_point_potential_gradient_[i] = this->get_dynamic_distance_gradient(this->collision_point_pos_[i], i);
	}

	for (int i = this->free_vars_start_; i <= this->free_vars_end_; i++) {
		this->collision_point_vel_[i] = Eigen::Vector3d(0, 0, 0);
		this->collision_point_acc_[i] = Eigen::Vector3d(0, 0, 0);
		for (int k = -hdi_plan_utils::DIFF_RULE_LENGTH / 2; k <= hdi_plan_utils::DIFF_RULE_LENGTH / 2; k++)
		{
			collision_point_vel_[i] +=
					(inv_time * hdi_plan_utils::DIFF_RULES[0][k + hdi_plan_utils::DIFF_RULE_LENGTH / 2]) *
					this->collision_point_pos_[i + k];
			collision_point_acc_[i] +=
					(inv_time_sq * hdi_plan_utils::DIFF_RULES[1][k + hdi_plan_utils::DIFF_RULE_LENGTH / 2]) *
					this->collision_point_pos_[i + k];
		}
		// get the norm of the velocity:
		this->collision_point_vel_mag_[i] = this->collision_point_vel_[i].norm();
	}
}

Eigen::Vector3d Chomp::get_distance_gradient(const Eigen::Vector3d& point) {
	double resolution = 0.01;
	double inv_twice_resolution = 1.0 / (2.0 * resolution);
	double gradient_x = (this->get_potential_for_gradient(point(0) + resolution, point(1), point(2)) - this->get_potential_for_gradient(point(0) - resolution, point(1), point(2))) * inv_twice_resolution;
	double gradient_y = (this->get_potential_for_gradient(point(0), point(1) + resolution, point(2)) - this->get_potential_for_gradient(point(0), point(1) - resolution, point(2))) * inv_twice_resolution;
	double gradient_z = (this->get_potential_for_gradient(point(0), point(1), point(2) + resolution) - this->get_potential_for_gradient(point(0), point(1), point(2) - resolution)) * inv_twice_resolution;
	Eigen::Vector3d gradient(gradient_x, gradient_y, gradient_z);
	return gradient;
}

Eigen::Vector3d Chomp::get_dynamic_distance_gradient(const Eigen::Vector3d& point, int index) {
	double resolution = 0.01;
	double inv_twice_resolution = 1.0 / (2.0 * resolution);
	double gradient_x = (this->get_dynamic_potential_for_gradient(point(0) + resolution, point(1), point(2), index) - this->get_dynamic_potential_for_gradient(point(0) - resolution, point(1), point(2), index)) * inv_twice_resolution;
	double gradient_y = (this->get_dynamic_potential_for_gradient(point(0), point(1) + resolution, point(2), index) - this->get_dynamic_potential_for_gradient(point(0), point(1) - resolution, point(2), index)) * inv_twice_resolution;
	double gradient_z = (this->get_dynamic_potential_for_gradient(point(0), point(1), point(2) + resolution, index) - this->get_dynamic_potential_for_gradient(point(0), point(1), point(2) - resolution, index)) * inv_twice_resolution;
	Eigen::Vector3d gradient(gradient_x, gradient_y, gradient_z);
	return gradient;
}


double Chomp::get_potential(const Eigen::Vector3d& point) {
	double distance_to_nearest_obstacle = std::numeric_limits<double>::infinity();
	for (auto obstacle : this->obstacle_map_) {
		double distance = hdi_plan_utils::get_distance(point, obstacle.second->get_position()) - this->quadrotor_radius_ - obstacle.second->get_size();
		if (distance < distance_to_nearest_obstacle) distance_to_nearest_obstacle = distance;
	}

	if (distance_to_nearest_obstacle >= this->min_clearence_) {
		return 0.0;
	} else if (distance_to_nearest_obstacle >= 0.0) {
		return 0.5 * pow(this->min_clearence_-distance_to_nearest_obstacle, 2.0) / this->min_clearence_;
	} else {
		this->is_collsion_free_ = false;
		return -distance_to_nearest_obstacle + 0.5 * this->min_clearence_;
	}
}

void Chomp::export_potential_data(bool if_dynamic) {
	std::string file_start_name = "potential_";
	if (if_dynamic) file_start_name = "dynamic_potential_";

	std::string file_name = file_start_name + std::to_string(this->chomp_path_file_num_) + ".txt";
	std::ofstream data_file (file_name);

	if (if_dynamic) {
		for (auto potential : this->dynamic_collision_point_potential_) {
			data_file << potential << "\n";
		}
	} else {
		for (auto potential : this->collision_point_potential_) {
			data_file << potential << "\n";
		}
	}

	if (data_file.is_open()) {
		data_file.close();
	}
}

void Chomp::export_potential_gradient_data(bool if_dynamic) {
	std::string file_start_name = "potential_gradient_";
	if (if_dynamic) file_start_name = "dynamic_potential_gradient_";

	std::string file_name = file_start_name + std::to_string(this->chomp_path_file_num_) + ".txt";
	std::ofstream data_file (file_name);

	if (if_dynamic) {
		for (auto potential : this->dynamic_collision_point_potential_gradient_) {
			data_file << potential(0) << " " << potential(1) << " " << potential(2) << "\n";
		}
	} else {
		for (auto potential : this->collision_point_potential_gradient_) {
			data_file << potential(0) << " " << potential(1) << " " << potential(2) << "\n";
		}
	}

	if (data_file.is_open()) {
		data_file.close();
	}
}

double Chomp::get_potential_for_gradient(double x, double y, double z) {
	double distance_to_nearest_obstacle = std::numeric_limits<double>::infinity();
	Eigen::Vector3d point(x,y,z);
	for (auto obstacle : this->obstacle_map_) {
		double distance = hdi_plan_utils::get_distance(point, obstacle.second->get_position()) -
						  this->quadrotor_radius_ - (obstacle.second->get_size()/2.0);
		if (distance < distance_to_nearest_obstacle) distance_to_nearest_obstacle = distance;
	}

	if (distance_to_nearest_obstacle >= this->min_clearence_) {
		return 0.0;
	} else if (distance_to_nearest_obstacle >= 0.0) {
		return 0.5 * pow(this->min_clearence_-distance_to_nearest_obstacle, 2.0) / this->min_clearence_;
	} else {
		this->is_collsion_free_ = false;
		return -distance_to_nearest_obstacle + 0.5 * this->min_clearence_;
	}
}

double Chomp::get_dynamic_potential(const Eigen::Vector3d& point, int index) {
	double distance_to_nearest_dynamic_obstacle = std::numeric_limits<double>::infinity();
	double point_time = this->full_trajectory_->calculate_time_by_index(index);
	for (auto human : this->human_map_) {
		Eigen::Vector2d predicted_position = human.second->predict_path(point_time);
		Eigen::Vector2d point_position(point(0), point(1));
		double distance = hdi_plan_utils::get_distance_2d(point_position, predicted_position) -
						  this->quadrotor_radius_ - (human.second->get_human_block_distance());
		//std::cout << "the dynamic distance is: " << distance << " . The point time is: " << point_time << std::endl;
		if (distance < distance_to_nearest_dynamic_obstacle) distance_to_nearest_dynamic_obstacle = distance;
	}

	double potential;
	if (distance_to_nearest_dynamic_obstacle >= this->min_clearence_) {
		potential = 0.0;
	} else if (distance_to_nearest_dynamic_obstacle >= 0.0) {
		potential = 0.5 * pow(this->min_clearence_-distance_to_nearest_dynamic_obstacle, 2.0) / this->min_clearence_;
	} else {
		this->is_collsion_free_ = false;
		potential = -distance_to_nearest_dynamic_obstacle + 0.5 * this->min_clearence_;
		//std::cout << "the dynamic potential is: " << potential << ". And final is: " << potential * exp(-this->dynamic_collision_factor_ * point_time) << " And the point time is: " << point_time << std::endl;
	}

	//return potential;

	if (static_cast<double>(index) < static_cast<double>(this->num_vars_all_) * 0.3) {
		return potential * exp(-this->dynamic_collision_factor_ * point_time);
	} else {
		return potential;
	}
}


double Chomp::get_dynamic_potential_for_gradient(double x, double y, double z, int index) {
	double distance_to_nearest_dynamic_obstacle = std::numeric_limits<double>::infinity();
	double point_time = this->full_trajectory_->calculate_time_by_index(index);
	for (auto human : this->human_map_) {
		Eigen::Vector2d predicted_position = human.second->predict_path(point_time);
		Eigen::Vector2d point_position(x, y);
		double distance = hdi_plan_utils::get_distance_2d(point_position, predicted_position) -
						  this->quadrotor_radius_ - (human.second->get_human_block_distance()/2.0);
		if (distance < distance_to_nearest_dynamic_obstacle) distance_to_nearest_dynamic_obstacle = distance;
	}

	double potential;
	if (distance_to_nearest_dynamic_obstacle >= this->min_clearence_) {
		potential = 0.0;
	} else if (distance_to_nearest_dynamic_obstacle >= 0.0) {
		potential = 0.5 * pow(this->min_clearence_-distance_to_nearest_dynamic_obstacle, 2.0) / this->min_clearence_;
	} else {
		this->is_collsion_free_ = false;
		potential = -distance_to_nearest_dynamic_obstacle + 0.5 * this->min_clearence_;
	}

	//return potential;

	if (static_cast<double>(index) < static_cast<double>(this->num_vars_all_) * 0.3) {
		return potential * exp(-this->dynamic_collision_factor_ * point_time);
	} else {
		return potential;
	}
}

double Chomp::get_collision_cost() {
	double collision_cost = 0.0;
	double worst_collision_cost = 0.0;
	this->worst_collision_cost_state_ = -1;

	for (int i=this->free_vars_start_; i<this->free_vars_end_; i++) {
		double state_collision_cost = this->collision_point_potential_[i] * this->collision_point_vel_mag_[i];
		collision_cost += state_collision_cost;
		if (state_collision_cost > worst_collision_cost)
		{
			worst_collision_cost = state_collision_cost;
			this->worst_collision_cost_state_ = i;
		}
	}
	return this->obstacle_cost_weight_ * collision_cost;
}

double Chomp::get_dynamic_collision_cost() {
	double collision_cost = 0.0;
	double worst_collision_cost = 0.0;
	this->worst_dynamic_collision_cost_state_ = -1;

	for (int i=this->free_vars_start_; i<this->free_vars_end_; i++) {
		double state_collision_cost = this->dynamic_collision_point_potential_[i] * this->collision_point_vel_mag_[i];
		collision_cost += state_collision_cost;
		if (state_collision_cost > worst_collision_cost)
		{
			worst_collision_cost = state_collision_cost;
			this->worst_dynamic_collision_cost_state_ = i;
		}
	}
	return this->dynamic_obstacle_cost_weight_ * collision_cost;
}

double Chomp::get_smoothness_cost() {
	double smoothness_cost = 0.0;
	for (int i = 0; i < this->num_joints_; i++) {
		smoothness_cost += this->joint_costs_[i]->getCost(this->full_trajectory_->get_joint_trajectory(i));
	}
	return this->smoothness_cost_weight_ * smoothness_cost;
}

void Chomp::calculate_smoothness_increments() {
	for (int i = 0; i < this->num_joints_; i++) {
		this->smoothness_derivative_ = this->joint_costs_[i]->getDerivative(this->full_trajectory_->get_joint_trajectory(i));
		this->smoothness_increments_.col(i) = -this->smoothness_derivative_.segment(this->full_trajectory_->get_start_index(), this->num_vars_free_);
	}

	/*
	for (int i = 0; i < this->num_vars_free_; i++) {
		std::cout << "The smoothness increments are: " << this->smoothness_increments_(i, 0) << " " << this->smoothness_increments_(i, 1) << " " << this->smoothness_increments_(i, 2) << std::endl;
	}*/
}

void Chomp::calculate_collision_increments() {
	double potential;
	double vel_mag_sq;
	double vel_mag;
	Eigen::Vector3d potential_gradient;
	Eigen::Vector3d normalized_velocity;
	Eigen::Matrix3d orthogonal_projector;
	Eigen::Vector3d curvature_vector;
	Eigen::Vector3d cartesian_gradient;

	this->collision_increments_.setZero(this->num_vars_free_, this->num_joints_);

	int start_point = this->free_vars_start_;
	int end_point = this->free_vars_end_;
	if (this->use_stochastic_descent_) {
		start_point = static_cast<int>(hdi_plan_utils::get_random_double() *
									   (this->free_vars_end_ - this->free_vars_start_) + this->free_vars_start_);
		end_point = start_point;
	}


	for (int i = start_point; i <= end_point; i++) {
		for (int j = 0; j < this->num_joints_; j++) {

		}
		potential = collision_point_potential_[i];
		if (potential < 0.0001)
			continue;
		potential_gradient = -collision_point_potential_gradient_[i];
		vel_mag = collision_point_vel_mag_[i];
		vel_mag_sq = vel_mag * vel_mag;

		normalized_velocity = this->collision_point_vel_[i] / vel_mag;
		orthogonal_projector = Eigen::Matrix3d::Identity() - (normalized_velocity * normalized_velocity.transpose());
		curvature_vector = (orthogonal_projector * this->collision_point_acc_[i]) / vel_mag_sq;
		cartesian_gradient = vel_mag * (orthogonal_projector * potential_gradient - potential * curvature_vector);

		this->collision_increments_.row(i - this->free_vars_start_).transpose() +=
				Eigen::MatrixXd::Identity(3, 3) * cartesian_gradient;
	}

	/*
	for (int i = 0; i < this->num_vars_free_; i++) {
		std::cout << "The collision increments are: " << this->collision_increments_(i, 0) << " " << this->collision_increments_(i, 1) << " " << this->collision_increments_(i, 2) << std::endl;
	}*/
}

void Chomp::calculate_dynamic_collision_increments() {
	double potential;
	double vel_mag_sq;
	double vel_mag;
	Eigen::Vector3d potential_gradient;
	Eigen::Vector3d normalized_velocity;
	Eigen::Matrix3d orthogonal_projector;
	Eigen::Vector3d curvature_vector;
	Eigen::Vector3d cartesian_gradient;

	this->dynamic_collision_increments_.setZero(this->num_vars_free_, this->num_joints_);

	int start_point = this->free_vars_start_;
	int end_point = this->free_vars_end_;
	if (this->use_stochastic_descent_) {
		start_point = static_cast<int>(hdi_plan_utils::get_random_double() *
									   (this->free_vars_end_ - this->free_vars_start_) + this->free_vars_start_);
		end_point = start_point;
	}


	for (int i = start_point; i <= end_point; i++) {
		for (int j = 0; j < this->num_joints_; j++) {

		}
		potential = dynamic_collision_point_potential_[i];
		if (potential < 0.0001)
			continue;
		potential_gradient = -dynamic_collision_point_potential_gradient_[i];
		vel_mag = collision_point_vel_mag_[i];
		vel_mag_sq = vel_mag * vel_mag;

		normalized_velocity = this->collision_point_vel_[i] / vel_mag;
		orthogonal_projector = Eigen::Matrix3d::Identity() - (normalized_velocity * normalized_velocity.transpose());
		curvature_vector = (orthogonal_projector * this->collision_point_acc_[i]) / vel_mag_sq;
		cartesian_gradient = vel_mag * (orthogonal_projector * potential_gradient - potential * curvature_vector);

		this->dynamic_collision_increments_.row(i - this->free_vars_start_).transpose() +=
				Eigen::MatrixXd::Identity(3, 3) * cartesian_gradient;
	}
}

/*
void Chomp::get_jacobian(int trajectory_point, const Eigen::Vector3d &collision_point_pos) {
if (isParent(joint_name, joint_names_[j]))
{
	Eigen::Vector3d column =
	joint_axes_[trajectory_point].cross(collision_point_pos - joint_positions_[trajectory_point]);

	this->jacobian_.col(0)[0] = column.x();
	this->jacobian_.col(0)[1] = column.y();
	this->jacobian_.col(0)[2] = column.z();
}
else
{
	this->jacobian_.col(0)[0] = 0.0;
	this->jacobian_.col(0)[1] = 0.0;
	this->jacobian_.col(0)[2] = 0.0;
}
}*/

void Chomp::calculate_total_increments() {
	for (int i = 0; i < this->num_joints_; i++) {
		this->final_increments_.col(i) = this->learning_rate_ * (
				this->joint_costs_[i]->getQuadraticCostInverse() *
				(this->smoothness_cost_weight_ * this->smoothness_increments_.col(i) +
				 this->obstacle_cost_weight_ * this->collision_increments_.col(i) +
				 this->dynamic_obstacle_cost_weight_ * this->dynamic_collision_increments_.col(i)));
	}
}

void Chomp::add_increments_to_trajectory() {
	for (int i = 0; i < this->num_joints_; i++) {
		double scale = 1.0;
		double max = this->final_increments_.col(i).maxCoeff();
		double min = this->final_increments_.col(i).minCoeff();
		double max_scale = this->joint_update_limit_ / fabs(max);
		double min_scale = this->joint_update_limit_ / fabs(min);
		if (max_scale < scale)
			scale = max_scale;
		if (min_scale < scale)
			scale = min_scale;
		//std::cout << "In chomp scale is : " << scale << std::endl;
		this->full_trajectory_->add_increments_to_trajectory(this->final_increments_.col(i), i, scale);
	}
}

void Chomp::convert_matrix_to_trajectory_points_vector() {
	//ROS_INFO("Chomp: convert to optimized trajectory");
	int start_extra = this->full_trajectory_->get_start_extra();

	for (int i = 0; i < this->num_vars_origin_; i++) {
		Eigen::Vector3d position(this->best_trajectory_(i + start_extra, 0),
								 this->best_trajectory_(i + start_extra, 1),
								 this->best_trajectory_(i + start_extra, 2));
		this->optimized_trajectory_[i] = position;
	}
}

}
