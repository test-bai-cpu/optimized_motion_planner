#pragma once

namespace optimized_motion_planner_utils {
class OptimizerParams {
public:
	OptimizerParams() = default;
	~OptimizerParams() = default;

	double collision_threshold;
	double planning_time_limit;
	int max_iterations;
	int max_iterations_after_collision_free;
	double learning_rate;
	double obstacle_cost_weight;
	double dynamic_obstacle_cost_weight;
	double dynamic_collision_factor;
	double smoothness_cost_weight;
	double smoothness_cost_velocity;
	double smoothness_cost_acceleration;
	double smoothness_cost_jerk;
	double ridge_factor;
	double min_clearence;
	double joint_update_limit;
	double quadrotor_radius;
	double quadrotor_speed;
	double total_planning_time;
};


}