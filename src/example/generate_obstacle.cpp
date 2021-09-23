#include "example/generate_obstacle.hpp"

namespace optimized_motion_planner {

GenerateObstacle::GenerateObstacle(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
		: nh_(nh),
		  pnh_(pnh) {

	this->obstacle_pub_ = nh_.advertise<optimized_motion_planner::obstacle_info>("optimized_motion_planner/obstacle_info_topic", 1);
	this->dynamic_obstacle_movement_pub_ = nh_.advertise<optimized_motion_planner::obstacle_info>("optimized_motion_planner/dynamic_obstacle_movement", 1);

	ros::Duration(5.0).sleep();
	//this->publish_static_obstacle();
	//ros::Duration(5.0).sleep();
	this->publish_dynamic_obstacle_movement();
}

void GenerateObstacle::publish_static_obstacle() {
	ROS_INFO("Publish static obstacle");
	optimized_motion_planner::obstacle_info obstacle_msg;
	obstacle_msg.name = "sphere_1";
	obstacle_msg.type = optimized_motion_planner_utils::Obstacle_type::sphere;
	obstacle_msg.operation = true;
	obstacle_msg.size = 3;
	obstacle_msg.position.x = 10.0;
	obstacle_msg.position.y = 10.0;
	obstacle_msg.position.z = 2.0;
	this->obstacle_pub_.publish(obstacle_msg);
}

optimized_motion_planner::obstacle_info GenerateObstacle::get_dynamic_obstacle_message(bool operation, int dynamic_obstacle_id, double size, double velocity, const Eigen::Vector3d& position) {
	optimized_motion_planner::obstacle_info obstacle_msg;
	obstacle_msg.name = "movingobstacle_" + std::to_string(dynamic_obstacle_id);
	obstacle_msg.type = optimized_motion_planner_utils::Obstacle_type::dynamic_obstacle;
	obstacle_msg.operation = operation;
	obstacle_msg.size = size;
	obstacle_msg.velocity = velocity;
	obstacle_msg.position.x = position(0);
	obstacle_msg.position.y = position(1);
	obstacle_msg.position.z = position(2);

	return obstacle_msg;
}

void GenerateObstacle::publish_dynamic_obstacle_movement() {
	Eigen::Vector3d start_point(10.0, 23.0, 0.0);
	Eigen::Vector3d goal_point(10.0, 0.0, 0.0);
	Eigen::Vector3d remove_obstacle(100.0, 100.0, 100.0);
	Eigen::Vector3d current_point(start_point(0), start_point(1), start_point(2));
	double distance = optimized_motion_planner_utils::get_distance(start_point, goal_point);
	double velocity = 1;
	double period = 1;
	double size = 4;
	int dynamic_obstacle_id = 1;
	int count = 0;
	ROS_INFO("###Generate Obstacle: Start to spawn a moving obstacle.");
	while (optimized_motion_planner_utils::get_distance(current_point, goal_point) > 0.5 * period * velocity) {
		if (count > 0) this->obstacle_pub_.publish(get_dynamic_obstacle_message(false, dynamic_obstacle_id, size, velocity, remove_obstacle));

		current_point(0) = (goal_point(0) - start_point(0)) * (velocity * count * period) / distance + start_point(0);
		current_point(1) = (goal_point(1) - start_point(1)) * (velocity * count * period) / distance + start_point(1);
		current_point(2) = (goal_point(2) - start_point(2)) * (velocity * count * period) / distance + start_point(2);

		this->obstacle_pub_.publish(get_dynamic_obstacle_message(true, dynamic_obstacle_id, size, velocity, current_point));
		if (count > 0) this->dynamic_obstacle_movement_pub_.publish(get_dynamic_obstacle_message(true, dynamic_obstacle_id, size, velocity, current_point));

		ros::Duration(period).sleep();
		count += 1;
	}
}
}