#pragma once

#include <stack>
#include <queue>
#include <vector>
#include <Eigen/Dense>
#include <memory>
#include <cmath>
#include <limits>
#include <cstdlib>
#include <time.h>
#include <string>
#include <map>
#include <queue>
#include <algorithm>
#include <iostream>
#include <fstream>

//ompl
#include "ompl/datastructures/NearestNeighbors.h"
#include <ompl/datastructures/BinaryHeap.h>
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/datastructures/NearestNeighborsGNAT.h"
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/config.h>


//ros
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

//hdi_plan
#include "dynamic_motion_planner/quadrotor.hpp"
#include "dynamic_motion_planner/RRT_node.hpp"
#include "dynamic_motion_planner/chomp.hpp"
#include "dynamic_motion_planner/chomp_trajectory.hpp"
#include "utils/types.hpp"
#include "utils/utility_functions.hpp"
#include "generate_dynamic_scene/obstacle.hpp"
#include "generate_dynamic_scene/human.hpp"
#include <hdi_plan/obstacle_info.h>
#include <hdi_plan/point_array.h>

namespace optimized_motion_planner {

class RRTX {
public:
	RRTX(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
	~RRTX();

	// debug
	ompl::base::RealVectorBounds get_space_info();
	double get_nn_size();
	void check_params();

private:
	// debug
	int debug{0};
	int iteration_count{0};
	bool if_find_solution_{false};
	std::ofstream solve_time_path_file;
	std::ofstream optimize_time_path_file;
	std::ofstream drone_actual_path_file;
	void visualize_node_tree();
	bool with_chomp_{true};

	double get_distance_check_ob(const std::shared_ptr<RRTNode>& node1, const std::shared_ptr<RRTNode>& node2);

	// drone
	double quadrotor_radius_{0.5};

	// ros nodes
	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;

	// initialize stack for storing points that desire to insert in the future when an obstacle is removed
	std::stack<std::vector<Eigen::Vector3d>> sample_stack;

	// useful rrt nodes
	std::shared_ptr<RRTNode> start_;
	std::shared_ptr<RRTNode> goal_;
	std::shared_ptr<RRTNode> quadrotor_;

	// space configuration
	ompl::base::SpaceInformationPtr si_;
	std::shared_ptr<ompl::base::SE3StateSpace> space_;
	void setup_space();
	int dimension_{3};

	// geometric Near-neighbor Access Tree
	std::shared_ptr<ompl::NearestNeighborsGNAT<std::shared_ptr<RRTNode>>> nearest_neighbors_tree_;

	// setup
	bool load_params();
	void setup();
	double distance_function(const std::shared_ptr<RRTNode>& a, const std::shared_ptr<RRTNode>& b) ;
	void set_to_start_position();
	bool position_ready{false};

	//
	std::shared_ptr<RRTNode> generate_random_node();

	// quadrotor related
	Eigen::Vector3d quadrotor_state_;

	// subscriber
	ros::Subscriber sub_quadrotor_state_;
	void quadrotor_state_callback(const nav_msgs::Odometry::ConstPtr &msg);

	ros::Subscriber sub_obstacle_info_;
	void obstacle_info_callback(const hdi_plan::obstacle_info::ConstPtr &msg);

	ros::Subscriber sub_goal_point_;
	void goal_point_callback(const geometry_msgs::Point::ConstPtr &msg);

	ros::Subscriber sub_human_movement_;
	void human_movement_callback(const hdi_plan::obstacle_info::ConstPtr &msg);
	//void human_movement_callback_region_version(const geometry_msgs::Point::ConstPtr &msg);

	// publisher
	ros::Publisher pub_optimized_path_;
	ros::Publisher pub_go_to_pose_;
	ros::Publisher pub_max_velocity_;
	ros::Publisher pub_get_new_path_;

	ros::Publisher pub_solution_path_;
	ros::Publisher pub_node_pos_;
	ros::Publisher pub_blocked_node_pos_;

	// main loop
	bool solve();

	// RRTX node
	int node_index_{0};
	std::vector<std::shared_ptr<RRTNode>> node_list_;
	void add_node_to_nearest_neighbors_tree(const std::shared_ptr<RRTNode>& node);

	// for calculate the shrinking ball radius
	void calculateRRG();
	double max_distance_{1}; // the maximum length of a motion to be added to a tree
	double r_rrt_{5}; // a constant for r-disc rewiring calculations
	double rrg_r_{3}; // current value of the radius used for the neighbors

	// functions in the main loop
	void saturate(std::shared_ptr<RRTNode> random_node, const std::shared_ptr<RRTNode>& nearest_node, double distance) const;
	bool node_in_free_space_check(const std::shared_ptr<RRTNode>& random_node);
	bool edge_in_free_space_check(const std::shared_ptr<RRTNode>& node1, const std::shared_ptr<RRTNode>& node2);
	bool extend(std::shared_ptr<RRTNode> random_node);

	// extend related
	void update_neighbors_list(std::shared_ptr<RRTNode> random_node);
	bool find_best_parent(std::shared_ptr<RRTNode> random_node);

	// cost related members
	//double compute_cost_human_cost_region_version(const Eigen::Vector3d& state1, const Eigen::Vector3d& state2);
	//double compute_cost_with_weight(const Eigen::Vector3d& state1, const Eigen::Vector3d& state2, double weight=1);
	double combine_cost(double cost1, double cost2);
	bool is_cost_better_than(double cost1, double cost2);

	// rewire
	double epsilon_{0.5};

	// cannot use priority queue since we need to update the key value
	//std::priority_queue<std::shared_ptr<RRTNode>, std::vector<std::shared_ptr<RRTNode>>, node_compare> node_queue1;
	ompl::BinaryHeap<std::shared_ptr<RRTNode>, RRTNode::node_compare> node_queue;
	void rewire_neighbors_v1(std::shared_ptr<RRTNode> random_node);
	void rewire_neighbors_v2(std::shared_ptr<RRTNode> random_node);
	void rewire_neighbors_for_env_update(std::shared_ptr<RRTNode> random_node);
	void cull_neighbors(std::shared_ptr<RRTNode> random_node);
	void verify_queue(std::shared_ptr<RRTNode> node);
	void reduce_inconsistency_v1();
	void reduce_inconsistency_v2();
	void reduce_inconsistency_for_env_update();
	void update_lmc_v1(std::shared_ptr<RRTNode> node);
	void update_lmc_v2(std::shared_ptr<RRTNode> node);
	void update_lmc(std::shared_ptr<RRTNode> node);
	void update_lmc_for_env_update(std::shared_ptr<RRTNode> node);
	// find the solution path
	std::vector<Eigen::Vector3d> solution_path;
	std::vector<Eigen::Vector3d> solution_path_tmp;

	bool update_solution_path();
	bool update_solution_path_tmp();
	void publish_without_chomp();
	void optimize_solution_path();

	// obstacle related
	void initiate_obstacles();
	void update_obstacle();
	std::map<std::string, std::shared_ptr<Obstacle>> obstacle_map;
	std::vector<std::shared_ptr<Obstacle>> obstacle_update_info_list;
	void remove_obstacle(const std::shared_ptr<Obstacle>& obstacle);
	void add_obstacle(const std::shared_ptr<Obstacle>& obstacle);
	void propogate_descendants();
	bool check_if_node_inside_obstacle(const std::shared_ptr<Obstacle>& obstacle, const std::shared_ptr<RRTNode>& node);
	bool check_if_node_inside_obstacle_for_adding_ob(const std::shared_ptr<Obstacle>& obstacle, const std::shared_ptr<RRTNode>& node);
	bool check_if_node_inside_all_obstacles(const std::shared_ptr<RRTNode>& node, bool consider_human);
	std::vector<std::shared_ptr<RRTNode>> orphan_node_list;
	void verify_orphan(std::shared_ptr<RRTNode> node);
	bool check_if_node_in_orphan_list(const std::shared_ptr<RRTNode>& node);
	bool if_environment_change{false};

	// utils
	void make_parent_of(std::shared_ptr<RRTNode> parent_node, std::shared_ptr<RRTNode> node);

	// human part
	std::map<int, std::shared_ptr<Human>> human_map_tmp_;
	std::map<int, std::shared_ptr<Human>> human_map_;
	std::vector<int> human_id_list_;
	int get_human_id(const std::string human_name);
	bool exist_human_{false};
	int human_callback_count{0};
	void add_human_as_obstacle(int human_id);
	bool check_if_edge_collide_human(const std::shared_ptr<RRTNode>& node1, const std::shared_ptr<RRTNode>& node2);
	//void find_nodes_in_human_position(const Eigen::Vector2d& human_position);
	//Eigen::Vector2d human_position_;
	bool if_add_human_{false};
	//bool check_if_node_inside_human(const std::shared_ptr<RRTNode>& node);
	//double check_if_state_near_human(const Eigen::Vector3d& state1);

	// time dimension
	double total_plan_time_{30};
	double generate_random_time(const Eigen::Vector3d& state);
	double quadrotor_speed_{1};
	ros::WallTime start_time_;

	// path write to file name
	int path_file_num_{0};
	int chomp_path_file_num_{0};

	// chomp trajectory
	double discretization_;

	// chomp
	double collision_threshold_;
	double planning_time_limit_;
	int max_iterations_;
	int max_iterations_after_collision_free_;
	double learning_rate_;
	double obstacle_cost_weight_;
	double dynamic_obstacle_cost_weight_;
	double dynamic_collision_factor_;
	double smoothness_cost_weight_;
	double smoothness_cost_velocity_;
	double smoothness_cost_acceleration_;
	double smoothness_cost_jerk_;
	double ridge_factor_;
	double min_clearence_;
	double joint_update_limit_;

	// dynamic obstacle
	double planning_horizon_time_;
};


}