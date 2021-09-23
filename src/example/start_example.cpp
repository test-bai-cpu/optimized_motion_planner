#include <ros/ros.h>
#include <std_msgs/Empty.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "start optimized motion planning example");
	ros::NodeHandle nh("");
	ros::Publisher trigger_solve_pub = nh.advertise<std_msgs::Empty>("optimized_motion_planner/trigger_solve_one_iteration", 1);
	ros::Rate loop_rate(10);

	while (ros::ok()) {
		std_msgs::Empty msg;
		trigger_solve_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}