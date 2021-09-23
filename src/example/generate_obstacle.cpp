#include <ros/ros.h>
#include "utils/types.hpp"
#include <optimized_motion_planner/obstacle_info.h>


void publish_static_obstacle() {
	hdi_plan::obstacle_info obstacle_msg;
	obstacle_msg.name = "sphere_1";
	obstacle_msg.type = optimized_motion_planner_utils::Obstacle_type::sphere;
	obstacle_msg.operation = true;
	obstacle_msg.size = 3;
	obstacle_msg.position.x = 10.0;
	obstacle_msg.position.y = 10.0;
	obstacle_msg.position.z = 2.0;
	obstacle_pub.publish(obstacle_msg);
}

optimized_motion_planner::obstacle_info get_dynamic_obstacle_message(bool operation, int dynamic_obstacle_id, const Eigen::Vector3d& position) {
	optimized_motion_planner::obstacle_info obstacle_msg;
	obstacle_msg.name = "movingobstacle_" + std::to_string(dynamic_obstacle_id);
	obstacle_msg.type = optimized_motion_planner::Obstacle_type::dynamic_obstacle;
	obstacle_msg.operation = operation;
	obstacle_msg.size = 1;
	obstacle_msg.position.x = position(0);
	obstacle_msg.position.y = position(1);
	obstacle_msg.position.z = position(2);

	return obstacle_msg;
}

void publish_dynamic_obstacle_movement() {
	Eigen::Vector3d start_point(10.0, 23.0, 0.0);
	Eigen::Vector3d goal_point(10.0, 0.0, 0.0);
	Eigen::Vector3d remove_obstacle(100.0, 100.0, 100.0);
	Eigen::Vector3d current_point(start_point(0), start_point(1), start_point(2));
	double distance = optimized_motion_planner_utils::get_distance(start_point, goal_point);
	double velocity = 1;
	double period = 1;
	int dynamic_obstacle_id = 1;
	int count = 0;
	ROS_INFO("###Generate Obstacle: Start to spawn a moving obstacle 1.");
	while (optimized_motion_planner_utils::get_distance_2d(current_point, goal_point) > 0.5 * period * velocity) {
		if (count > 0) obstacle_pub.publish(get_obstacle_message(false, dynamic_obstacle_id, remove_obstacle));

		current_point(0) = (goal_point(0) - start_point(0)) * (velocity * count * period) / distance + start_point(0);
		current_point(1) = (goal_point(1) - start_point(1)) * (velocity * count * period) / distance + start_point(1);
		current_point(2) = (goal_point(2) - start_point(2)) * (velocity * count * period) / distance + start_point(2);

		std::cout << "Sending pos: " << current_point(0) << " " << current_point(1) << " " << current_point(2) << std::endl;
		obstacle_pub.publish(get_obstacle_message(true, dynamic_obstacle_id, current_point);
		if (count > 0) dynamic_obstacle_movement_pub.publish(get_obstacle_message(true, dynamic_obstacle_id, current_point));

		ros::Duration(period).sleep();
		count += 1;
	}
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "publish obstacle information");
	ros::NodeHandle nh("");
	ros::Publisher obstacle_pub = nh.advertise<std_msgs::Empty>("optimized_motion_planner/trigger_solve_one_iteration", 1);
	ros::Publisher dynamic_obstacle_movement_pub = nh_.advertise<optimized_motion_planner::obstacle_info>("optimized_motion_planner/dynamic_obstacle_movement", 1);
	ros::Rate loop_rate(10);

	ros::Duration(20.0).sleep();
	this->publish_obstacle();

	return 0;
}