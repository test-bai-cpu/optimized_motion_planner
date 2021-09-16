#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <map>
#include <limits>
#include <vector>
#include <memory>
#include <iostream>
#include <fstream>

#include <ros/ros.h>

#include "generate_dynamic_scene/obstacle.hpp"
#include "dynamic_motion_planner/chomp_trajectory.hpp"
#include "dynamic_motion_planner/chomp_cost.hpp"
#include "generate_dynamic_scene/obstacle.hpp"
#include "generate_dynamic_scene/human.hpp"
#include "utils/utility_functions.hpp"

namespace optimized_motion_planner {
class CHOMPDynamic {
public:
	CHOMPDynamic(double collision_threshold, double planning_time_limit, int max_iterations, int max_iterations_after_collision_free,
		  double learning_rate, double obstacle_cost_weight, double dynamic_obstacle_cost_weight, double dynamic_collision_factor,
		  double smoothness_cost_weight, double smoothness_cost_velocity, double smoothness_cost_acceleration, double smoothness_cost_jerk,
		  double ridge_factor, double min_clearence, double joint_update_limit, double quadrotor_radius,
		  const std::shared_ptr<ChompTrajectory>& trajectory, const std::map<std::string, std::shared_ptr<Obstacle>>& obstacle_map,
		  const std::map<int, std::shared_ptr<Human>>& human_map, int chomp_path_file_num);
	~CHOMPDynamic();

	std::vector<Eigen::Vector3d> get_optimized_trajectory() const {
		return this->optimized_trajectory_;
	}

private:
	// debug
	std::ofstream chomp_cost_file;
	void check_params();

	std::shared_ptr<ChompTrajectory> full_trajectory_;
	std::vector<Eigen::Vector3d> optimized_trajectory_;
	int num_vars_all_;
	int num_vars_free_;
	int num_vars_origin_;
	int free_vars_start_;
	int free_vars_end_;
	int last_improvement_iteration_;
	bool is_collsion_free_{false};
	int iteration_{0};
	int collision_free_iteration_{0};
	bool filter_mode_{false};
	double collision_threshold_{0.07};


	// joint
	int num_joints_{3};
	std::vector<std::shared_ptr<ChompCost>> joint_costs_;

	// trajectory
	Eigen::MatrixXd best_trajectory_;
	double best_trajectory_cost_;

	// matrix
	Eigen::MatrixXd smoothness_increments_;
	Eigen::MatrixXd collision_increments_;
	Eigen::MatrixXd dynamic_collision_increments_;
	Eigen::MatrixXd final_increments_;

	// temporary variables
	Eigen::VectorXd smoothness_derivative_;
	Eigen::MatrixXd jacobian_;
	Eigen::MatrixXd jacobian_pseudo_inverse_;
	Eigen::Matrix3d jacobian_jacobian_tranpose_;

	// smoothness cost
	//std::shared_ptr<ChompCost> joint_cost_;

	// collision cost
	std::map<std::string, std::shared_ptr<Obstacle>> obstacle_map_;
	std::vector<Eigen::Vector3d> collision_point_pos_;
	std::vector<Eigen::Vector3d> collision_point_vel_;
	std::vector<Eigen::Vector3d> collision_point_acc_;
	std::vector<double> collision_point_potential_;
	std::vector<double> collision_point_vel_mag_;
	std::vector<Eigen::Vector3d> collision_point_potential_gradient_;

	double get_potential(const Eigen::Vector3d& point);
	void get_collision_point_pos();
	Eigen::Vector3d get_distance_gradient(const Eigen::Vector3d& point);
	double get_potential_for_gradient(double x, double y, double z);
	void export_potential_data(bool if_dynamic);
	void export_potential_gradient_data(bool if_dynamic);
	//void get_jacobian(int trajectory_point, const Eigen::Vector3d& collision_point_pos);

	// dynamic collision cost
	std::map<int, std::shared_ptr<Human>> human_map_;
	std::vector<double> dynamic_collision_point_potential_;
	std::vector<Eigen::Vector3d> dynamic_collision_point_potential_gradient_;
	double get_dynamic_potential(const Eigen::Vector3d& point, int index);
	Eigen::Vector3d get_dynamic_distance_gradient(const Eigen::Vector3d& point, int index);
	double get_dynamic_potential_for_gradient(double x, double y, double z, int index);

	// optimize process
	void initialize();
	bool optimize();
	void convert_matrix_to_trajectory_points_vector();

	void perform_forward_kinematics();
	double get_collision_cost();
	double get_smoothness_cost();
	double get_dynamic_collision_cost();
	void calculate_smoothness_increments();
	void calculate_collision_increments();
	void calculate_dynamic_collision_increments();
	void calculate_total_increments();
	void add_increments_to_trajectory();

	int worst_collision_cost_state_{-1};
	int worst_dynamic_collision_cost_state_{-1};

	// parameters
	double planning_time_limit_{60.0};
	int max_iterations_{50};
	int max_iterations_after_collision_free_{15};
	double learning_rate_{0.1};
	double obstacle_cost_weight_{0.1};
	double dynamic_obstacle_cost_weight_{0.1};
	double dynamic_collision_factor_{0.1};
	double smoothness_cost_weight_{0.5};
	double smoothness_cost_velocity_{0.0};
	double smoothness_cost_acceleration_{1.0};
	double smoothness_cost_jerk_{0.0};
	double ridge_factor_{0.0};
	double min_clearence_{0.5};
	double joint_update_limit_{0.1};
	double quadrotor_radius_{0.5};
	bool use_stochastic_descent_{false};
	// export data
	int chomp_path_file_num_{0};

};

}