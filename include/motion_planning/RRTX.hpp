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

// ompl
#include "ompl/datastructures/NearestNeighbors.h"
#include <ompl/datastructures/BinaryHeap.h>
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/datastructures/NearestNeighborsGNAT.h"
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/config.h>

// ros
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

// optimized motion planner
#include "motion_planning/RRTX_node.hpp"
#include "motion_planning/dynamic_obstacle.hpp"
#include "optimizer/CHOMP_dynamic.hpp"
#include "optimizer/CHOMP_dynamic_trajectory.hpp"

#include "utils/utility_functions.hpp"
#include "utils/optimizer_params.hpp"
#include "utils/types.hpp"
#include <optimized_motion_planner/obstacle_info.h>
#include <optimized_motion_planner/point_array.h>


namespace optimized_motion_planner {

class RRTX {
public:
	RRTX(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
	~RRTX();

	// debug
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
	double get_distance_check_ob(const std::shared_ptr<RRTXNode>& node1, const std::shared_ptr<RRTXNode>& node2);

	// drone
	double quadrotor_radius_;
	double quadrotor_speed_;

	// time dimension
	double total_planning_time_;
	double generate_random_time(const Eigen::Vector3d& state);
	ros::WallTime start_time_;
	double planning_horizon_time_;

	// ros nodes
	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;

	// rrt nodes
	std::shared_ptr<RRTXNode> start_;
	std::shared_ptr<RRTXNode> goal_;

	// parameter object
	std::shared_ptr<optimized_motion_planner_utils::OptimizerParams> optimizer_params_;
	void set_optimizer_params();

	// geometric Near-neighbor Access Tree
	std::shared_ptr<ompl::NearestNeighborsGNAT<std::shared_ptr<RRTXNode>>> nearest_neighbors_tree_;

	// setup
	bool load_params();
	void setup();
	int dimension_{3};

	// subscriber
	ros::Subscriber sub_solve_trigger_;
	void solve_trigger_callback(const std_msgs::Empty::ConstPtr &msg);
	ros::Subscriber sub_obstacle_info_;
	void obstacle_info_callback(const optimized_motion_planner::obstacle_info::ConstPtr &msg);
	ros::Subscriber sub_dynamic_obstacle_movement_;
	void dynamic_obstacle_movement_callback(const optimized_motion_planner::obstacle_info::ConstPtr &msg);

	// publisher
	ros::Publisher pub_planned_trajectory_;

	// main iteration
	bool solve();
	void saturate(std::shared_ptr<RRTXNode> random_node, const std::shared_ptr<RRTXNode>& nearest_node, double distance) const;
	bool node_in_free_space_check(const std::shared_ptr<RRTXNode>& random_node);
	bool edge_in_free_space_check(const std::shared_ptr<RRTXNode>& node1, const std::shared_ptr<RRTXNode>& node2);
	bool extend(std::shared_ptr<RRTXNode> random_node);

	// RRTX node
	int node_index_{0};
	std::vector<std::shared_ptr<RRTXNode>> node_list_;
	void add_node_to_nearest_neighbors_tree(const std::shared_ptr<RRTXNode>& node);
	std::shared_ptr<RRTXNode> generate_random_node();

	// for calculate the shrinking ball radius
	void calculateRRG();
	double max_distance_{1}; // the maximum length of a motion to be added to a tree
	double r_rrt_{5}; // a constant for r-disc rewiring calculations
	double rrg_r_{3}; // current value of the radius used for the neighbors

	// extend related
	void update_neighbors_list(std::shared_ptr<RRTXNode> random_node);
	bool find_best_parent(std::shared_ptr<RRTXNode> random_node);

	// cost related members
	double combine_cost(double cost1, double cost2);
	bool is_cost_better_than(double cost1, double cost2);

	// rewire
	double epsilon_{0.5};
	ompl::BinaryHeap<std::shared_ptr<RRTXNode>, RRTXNode::node_compare> node_queue;
	void rewire_neighbors_v1(std::shared_ptr<RRTXNode> random_node);
	void rewire_neighbors_v2(std::shared_ptr<RRTXNode> random_node);
	void cull_neighbors(std::shared_ptr<RRTXNode> random_node);
	void verify_queue(std::shared_ptr<RRTXNode> node);
	void reduce_inconsistency_v1();
	void reduce_inconsistency_v2();
	void update_lmc_v1(std::shared_ptr<RRTXNode> node);
	void update_lmc_v2(std::shared_ptr<RRTXNode> node);

	// find the solution path
	std::vector<Eigen::Vector3d> solution_path;
	std::vector<Eigen::Vector3d> solution_path_tmp;
	bool update_solution_path();
	void publish_without_optimizer();
	void publish_with_optimizer();

	// environment update
	void update_obstacle();
	std::map<std::string, std::shared_ptr<Obstacle>> obstacle_map;
	std::vector<std::shared_ptr<Obstacle>> obstacle_update_info_list;
	void remove_obstacle(const std::shared_ptr<Obstacle>& obstacle);
	void add_obstacle(const std::shared_ptr<Obstacle>& obstacle);
	void propogate_descendants();
	bool check_if_node_inside_obstacle(const std::shared_ptr<Obstacle>& obstacle, const std::shared_ptr<RRTXNode>& node);
	bool check_if_node_inside_obstacle_for_adding_ob(const std::shared_ptr<Obstacle>& obstacle, const std::shared_ptr<RRTXNode>& node);
	bool check_if_node_inside_all_obstacles(const std::shared_ptr<RRTXNode>& node, bool consider_dynamic_obstacle);
	std::vector<std::shared_ptr<RRTXNode>> orphan_node_list;
	void verify_orphan(std::shared_ptr<RRTXNode> node);
	bool check_if_node_in_orphan_list(const std::shared_ptr<RRTXNode>& node);
	bool if_environment_change{false};

	// dynamic obstacle
	std::map<int, std::shared_ptr<DynamicObstacle>> dynamic_obstacle_map_tmp_;
	std::map<int, std::shared_ptr<DynamicObstacle>> dynamic_obstacle_map_;
	std::vector<int> dynamic_obstacle_id_list_;
	int get_dynamic_obstacle_id(const std::string& dynamic_obstacle_name);
	bool exist_dynamic_obstacle_{false};
	int dynamic_obstacle_callback_count{0};
	void add_dynamic_obstacle(int dynamic_obstacle_id);
	bool if_add_dynamic_obstacle_{false};

	// utils
	void make_parent_of(std::shared_ptr<RRTXNode> parent_node, std::shared_ptr<RRTXNode> node);
	double discretization_;
	double min_clearence_;

	// path write to file name
	int path_file_num_{0};
	int chomp_path_file_num_{0};
};


}