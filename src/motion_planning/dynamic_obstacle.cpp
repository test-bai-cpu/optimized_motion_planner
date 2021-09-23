#include "utils/utility_functions.hpp"

#include "motion_planning/dynamic_obstacle.hpp"


namespace optimized_motion_planner {

Eigen::Vector3d DynamicObstacle::predict_path(double node_time) {
	double move_distance = (node_time - this->start_time_) * this->dynamic_obstacle_velocity_;
	double unit_distance = optimized_motion_planner_utils::get_distance(this->start_position_, this->second_position_);

	Eigen::Vector3d predict_position(this->start_position_(0), this->start_position_(1), this->start_position_(2));

	predict_position(0) = (this->second_position_(0) - this->start_position_(0)) * move_distance / unit_distance + this->start_position_(0);
	predict_position(1) = (this->second_position_(1) - this->start_position_(1)) * move_distance / unit_distance + this->start_position_(1);
	predict_position(2) = (this->second_position_(2) - this->start_position_(2)) * move_distance / unit_distance + this->start_position_(2);

	if (node_time > this->planning_horizon_time_) {
		predict_position(0) = 100;
		predict_position(1) = 100;
		predict_position(2) = 100;
	}

	//std::cout << "The node time is: " << node_time << " .And the predict position is: " << predict_position(0) << " " << predict_position(1) << std::endl;
	return predict_position;
}

bool DynamicObstacle::check_if_node_inside_dynamic_obstacle(const std::shared_ptr<RRTXNode>& node) {
	Eigen::Vector2d node_position(node->get_state()(0), node->get_state()(1));
	double node_time = node->get_time();

	/*if (node_time < this->start_time_) {
		return false;
	}*/

	/*
	if (!this->if_move_) {
		return optimized_motion_planner_utils::get_distance_2d(this->start_position_, node_position)<this->dynamic_obstacle_size_;
	}
	*/

	Eigen::Vector3d predict_position = this->predict_path(node_time);
	//std::cout << "Node time: " << node->get_time() << std::endl;
	//std::cout << "Node position: " << node_position(0) << " " << node_position(1) << " Predict pos: " <<  predict_position(0) << " " << predict_position(1) << " distance: " << optimized_motion_planner_utils::get_distance_2d(predict_position, node_position) -4.5 << std::endl;
	bool res = optimized_motion_planner_utils::get_distance(predict_position, node->get_state()) - this->quadrotor_radius_ < this->dynamic_obstacle_size_;

	if (res) {
		if (node_time > this->planning_horizon_time_) {
			//std::cout << "The node pos is: " << node->get_state().x() << " " << node->get_state().y() << " " << node->get_state().z() << std::endl;
			//std::cout << "The node time is: " << node_time << std::endl;
			//std::cout << "The predict position is: " << predict_position(0) << " " << predict_position(1) << std::endl;
			//std::cout << "The distance to dynamic obstacle is: " << optimized_motion_planner_utils::get_distance_2d(predict_position, node_position) << "The result is: " << res << std::endl;
			ROS_WARN("#####Skip the collided point because the planning_horizon_time");
			res = false;
		}
	}

	return res;
}

}