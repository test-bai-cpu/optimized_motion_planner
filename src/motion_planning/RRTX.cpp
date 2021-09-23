#include "motion_planning/RRTX.hpp"

namespace optimized_motion_planner {

RRTX::RRTX(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
		: nh_(nh),
		  pnh_(pnh) {
	// load parameters
	if (!this->load_params()) {
		ROS_WARN("[%s] Could not load all parameters in optimized motion planner.",
				 this->pnh_.getNamespace().c_str());
	} else {
		ROS_INFO("[%s] Loaded all parameters in optimized motion planner.", this->pnh_.getNamespace().c_str());
	}

	// setup
	this->setup();
	srand((unsigned)time(NULL));
	this->start_time_ = ros::WallTime::now();

	// initialize subscribers
	this->sub_solve_trigger_ = nh_.subscribe("optimized_motion_planner/trigger_solve_one_iteration", 1, &RRTX::solve_trigger_callback, this);
	this->sub_obstacle_info_ = nh_.subscribe("optimized_motion_planner/obstacle_info_topic", 1, &RRTX::obstacle_info_callback, this);
	this->sub_dynamic_obstacle_movement_ = nh_.subscribe("optimized_motion_planner/dynamic_obstacle_movement", 1, &RRTX::dynamic_obstacle_movement_callback, this);
	this->pub_planned_trajectory_ = nh_.advertise<optimized_motion_planner::point_array>("optimized_motion_planner/planned_trajectory", 1);

	// initialize parameter object
	this->set_optimizer_params();

	// save data
	std::string file_name = "solve_time.txt";
	this->solve_time_path_file.open(file_name);
	std::string op_file_name = "optimize_time.txt";
	this->optimize_time_path_file.open(op_file_name);
	std::string actual_path_file_name = "drone_actual_path.txt";
	this->drone_actual_path_file.open(actual_path_file_name);
}

RRTX::~RRTX() = default;

// debug functions
void RRTX::visualize_node_tree() {
	std::vector<std::shared_ptr<RRTXNode>> nodes_list;
	this->nearest_neighbors_tree_->list(nodes_list);
	std::vector<std::shared_ptr<RRTXNode>> subpath;

	for (auto node : nodes_list) {
		std::shared_ptr<RRTXNode> intermediate_node = node;
		subpath.clear();
		while (intermediate_node == this->goal_ || intermediate_node->parent != nullptr) {
			if (std::find(subpath.begin(), subpath.end(), intermediate_node) != subpath.end()) {
				ROS_WARN("Have circle here.");
				break;
			}
			subpath.push_back(intermediate_node);
			if (intermediate_node == this->goal_) {
				break;
			}
			intermediate_node = intermediate_node->parent;
		}
		std::cout << "####### Sub path is: ";
		for (auto path_node : subpath) {
			std::cout << path_node->get_state().x() << " " << path_node->get_state().y() << " " << path_node->get_state().z() << " . g_cost is: " << path_node->get_g_cost() << " . lmc is: " << path_node->get_lmc() << " || ";
		}
		std::cout << " " << "\n" << std::endl;
	}
}

double RRTX::get_nn_size() {
	return static_cast<double>(this->nearest_neighbors_tree_->size());
}

void RRTX::set_optimizer_params() {
	this->optimizer_params_ = std::make_shared<optimized_motion_planner_utils::OptimizerParams>();

	this->pnh_.getParam("collision_threshold", this->optimizer_params_->collision_threshold);
	this->pnh_.getParam("planning_time_limit", this->optimizer_params_->planning_time_limit);
	this->pnh_.getParam("max_iterations", this->optimizer_params_->max_iterations);
	this->pnh_.getParam("max_iterations_after_collision_free", this->optimizer_params_->max_iterations_after_collision_free);
	this->pnh_.getParam("learning_rate", this->optimizer_params_->learning_rate);
	this->pnh_.getParam("obstacle_cost_weight", this->optimizer_params_->obstacle_cost_weight);
	this->pnh_.getParam("dynamic_obstacle_cost_weight", this->optimizer_params_->dynamic_obstacle_cost_weight);
	this->pnh_.getParam("dynamic_collision_factor", this->optimizer_params_->dynamic_collision_factor);
	this->pnh_.getParam("smoothness_cost_weight", this->optimizer_params_->smoothness_cost_weight);
	this->pnh_.getParam("smoothness_cost_velocity", this->optimizer_params_->smoothness_cost_velocity);
	this->pnh_.getParam("smoothness_cost_acceleration", this->optimizer_params_->smoothness_cost_acceleration);
	this->pnh_.getParam("smoothness_cost_jerk", this->optimizer_params_->smoothness_cost_jerk);
	this->pnh_.getParam("ridge_factor", this->optimizer_params_->ridge_factor);
	this->pnh_.getParam("min_clearence", this->optimizer_params_->min_clearence);
	this->pnh_.getParam("joint_update_limit", this->optimizer_params_->joint_update_limit);
	this->pnh_.getParam("quadrotor_radius", this->optimizer_params_->quadrotor_radius);
	this->pnh_.getParam("quadrotor_speed", this->optimizer_params_->quadrotor_speed);
	this->pnh_.getParam("total_planning_time", this->optimizer_params_->total_planning_time);
}

bool RRTX::load_params() {
	this->pnh_.getParam("total_planning_time", this->total_planning_time_);
	this->pnh_.getParam("quadrotor_radius", this->quadrotor_radius_);
	this->pnh_.getParam("quadrotor_speed", this->quadrotor_speed_);
	this->pnh_.getParam("max_distance", this->max_distance_);
	this->pnh_.getParam("r_rrt", this->r_rrt_);
	this->pnh_.getParam("rrg_r", this->rrg_r_);
	this->pnh_.getParam("epsilon", this->epsilon_);
	this->pnh_.getParam("with_chomp", this->with_chomp_);
	this->pnh_.getParam("planning_horizon_time", this->planning_horizon_time_);
	this->pnh_.getParam("discretization", this->discretization_);
	this->pnh_.getParam("min_clearence", this->min_clearence_);
	return true;
}

void RRTX::setup() {
	std::vector<double> start_position_param;
	this->pnh_.getParam("start_position", start_position_param);
	Eigen::Vector3d start_position(start_position_param[0],start_position_param[1],start_position_param[2]);
	std::cout << "The start position is set to: " << start_position(0) << " " << start_position(1) << " " << start_position(2) << std::endl;
	this->start_ = std::make_shared<RRTXNode>(start_position, 0);

	std::vector<double> goal_position_param;
	this->pnh_.getParam("goal_position", goal_position_param);
	Eigen::Vector3d goal_position(goal_position_param[0],goal_position_param[1],goal_position_param[2]);
	std::cout << "The goal position is set to: " << goal_position(0) << " " << goal_position(1) << " " << goal_position(2) << std::endl;
	this->goal_ = std::make_shared<RRTXNode>(goal_position, this->total_planning_time_);
	this->goal_->set_lmc(0);
	this->goal_->set_g_cost(0);

	this->nearest_neighbors_tree_ = std::make_shared<ompl::NearestNeighborsGNAT<std::shared_ptr<RRTXNode>>>();
	this->nearest_neighbors_tree_->setDistanceFunction([this](const std::shared_ptr<RRTXNode>& a, const std::shared_ptr<RRTXNode>& b) { return optimized_motion_planner_utils::get_distance(a->get_state(), b->get_state()); });
	this->add_node_to_nearest_neighbors_tree(this->goal_);
}

void RRTX::add_node_to_nearest_neighbors_tree(const std::shared_ptr<RRTXNode>& node) {
	node->set_unique_id(this->node_index_);

	for (auto node_1 : this->node_list_) {
		double distance = optimized_motion_planner_utils::get_distance(node->get_state(), node_1->get_state());
		node->add_to_neighbor_edge_list(distance);
		node_1->add_to_neighbor_edge_list(distance);
	}

	node->add_to_neighbor_edge_list(0.0);
	this->nearest_neighbors_tree_->add(node);
	this->node_list_.push_back(node);
	this->node_index_ += 1;
}

std::shared_ptr<RRTXNode> RRTX::generate_random_node() {
	std::vector<double> sample_region_x;
	this->pnh_.getParam("sample_region_x", sample_region_x);
	std::vector<double> sample_region_y;
	this->pnh_.getParam("sample_region_y", sample_region_y);
	std::vector<double> sample_region_z;
	this->pnh_.getParam("sample_region_z", sample_region_z);

	bool choose_current_state_region = (rand() % 100) < 5;
	double x, y, z;

	if (choose_current_state_region) {
		double range = 1;
		Eigen::Vector3d start_state = this->start_->get_state();
		x = start_state(0) - range + (rand()/double(RAND_MAX)*2*range);
		y = start_state(1) - range + (rand()/double(RAND_MAX)*2*range);
		z = start_state(2) - range + (rand()/double(RAND_MAX)*2*range);
	} else {
		x = sample_region_x[0] + (rand()/double(RAND_MAX)*(sample_region_x[1] - sample_region_x[0]));
		y = sample_region_y[0] + (rand()/double(RAND_MAX)*(sample_region_y[1] - sample_region_y[0]));
		z = sample_region_z[0] + (rand()/double(RAND_MAX)*(sample_region_z[1] - sample_region_z[0]));
	}

	Eigen::Vector3d random_position(x,y,z);
	auto random_node = std::make_shared<RRTXNode>(random_position);
	return random_node;
}

double RRTX::generate_random_time(const Eigen::Vector3d& state) {
	bool choose_time_by_distance = (rand() % 100) < 90;

	double min_time = optimized_motion_planner_utils::get_distance(state, this->start_->get_state())/this->quadrotor_speed_;
	double max_time = optimized_motion_planner_utils::get_distance(this->goal_->get_state(), this->start_->get_state())/this->quadrotor_speed_;

	double random_time;
	if (choose_time_by_distance) {
		random_time = min_time;
	} else {
		random_time = min_time + (rand()/double(RAND_MAX)*(max_time - min_time));
	}

	return random_time;
}

void RRTX::solve_trigger_callback(const std_msgs::Empty::ConstPtr &msg) {
	this->solve();
}

bool RRTX::solve() {
		ros::Duration(0.1).sleep();

		if (this->iteration_count >= 400 && (this->iteration_count % 50 == 1)) {
			ROS_INFO("TRY to update/find solution path");
			std::cout << "iteration number is: " << this->iteration_count << std::endl;
			//this->visualize_node_tree();
			if (this->update_solution_path()) {
				if (this->with_chomp_) {
					ROS_INFO("Publish with optimizer");
					this->publish_with_optimizer();

				} else {
					ROS_INFO("Publish without optimizer");
					this->publish_without_optimizer();
				}
			}

		}

		ros::WallTime solve_start_time = ros::WallTime::now();
		this->iteration_count += 1;
		//std::cout << "The iteration number is: " << this->iteration_count << "The nodes number is: " << static_cast<double>(this->nearest_neighbors_tree_->size()) << " Minus is: " << this->iteration_count - static_cast<double>(this->nearest_neighbors_tree_->size()) << std::endl;

		this->calculateRRG();

		this->update_obstacle();

		std::shared_ptr<RRTXNode> random_node = generate_random_node();

		// find the closest node in the tree
		std::shared_ptr<RRTXNode> nearest_node = this->nearest_neighbors_tree_->nearest(random_node);

		double distance = optimized_motion_planner_utils::get_distance(random_node->get_state(), nearest_node->get_state());

		if (distance > this->max_distance_) {
			this->saturate(random_node, nearest_node, distance);
		}

		if (!this->extend(random_node)) {
			return false;
		}

		this->rewire_neighbors_v1(random_node);
		this->reduce_inconsistency_v1();

		double solve_time = (ros::WallTime::now() - solve_start_time).toSec();
		this->solve_time_path_file << solve_time << "\n";

		return true;
};

void RRTX::verify_orphan(std::shared_ptr<RRTXNode> node) {
	if (node->handle != nullptr) {
		this->node_queue.remove(node->handle);
		node->handle = nullptr;
	}
	this->orphan_node_list.push_back(node);
}

void RRTX::propogate_descendants() {
	std::vector<std::shared_ptr<RRTXNode>> orphan_node_append_list;
	double inf = std::numeric_limits<double>::infinity();
	for (auto orphan_node : this->orphan_node_list) {
		orphan_node_append_list.insert(orphan_node_append_list.end(), orphan_node->children.begin(), orphan_node->children.end());
	}
	this->orphan_node_list.insert(this->orphan_node_list.end(), orphan_node_append_list.begin(), orphan_node_append_list.end());

	for (auto orphan_node_2 : this->orphan_node_list) {
		std::vector<std::shared_ptr<RRTXNode>> n_out;
		n_out.insert(n_out.end(), orphan_node_2->n0_out.begin(), orphan_node_2->n0_out.end());
		n_out.insert(n_out.end(), orphan_node_2->nr_out.begin(), orphan_node_2->nr_out.end());
		if (orphan_node_2->parent != nullptr) {
			n_out.push_back(orphan_node_2->parent);
		}

		for (auto neighbor : n_out) {
			if (this->check_if_node_in_orphan_list(neighbor)) continue;
			neighbor->set_g_cost(inf, true);
			if (this->check_if_node_inside_all_obstacles(neighbor, true)) continue;
			this->verify_queue(neighbor);
		}
	}

	int orphan_count = 0;
	for (auto orphan_node_3 : this->orphan_node_list) {
		orphan_node_3->set_g_cost(inf, true);
		orphan_node_3->set_lmc(inf, true);
		if (orphan_node_3->parent != nullptr) {
			orphan_node_3->parent->children.erase(std::remove(orphan_node_3->parent->children.begin(), orphan_node_3->parent->children.end(), orphan_node_3), orphan_node_3->parent->children.end());
		}
		orphan_node_3->parent = nullptr;
		//std::cout << "The orphan node position: " << orphan_node_3->get_state().x() << " " << orphan_node_3->get_state().y() << " " << orphan_node_3->get_state().z() << " . g_cost is: " << orphan_node_3->get_g_cost() << " . lmc is: " << orphan_node_3->get_lmc() << std::endl;
		orphan_count += 1;
	}
	//std::cout << "In propogateDescendants, the orphan count is: " << orphan_count << std::endl;
	this->orphan_node_list.clear();
}

bool RRTX::check_if_node_in_orphan_list(const std::shared_ptr<RRTXNode>& node) {
	for (auto orphan_node : this->orphan_node_list) {
		if (orphan_node == node) return true;
	}
	return false;
}

bool RRTX::update_solution_path() {
	ROS_INFO("Try to find the solution path now.");
	std::cout << "The iteration number is: " << this->iteration_count << std::endl;
	bool if_find_solution = false;

	std::vector<std::shared_ptr<RRTXNode>> near_neighbors;
	this->nearest_neighbors_tree_->nearestR(this->start_, this->max_distance_ + 1, near_neighbors);

	for (auto free_neighbor : near_neighbors) {
		this->solution_path_tmp.clear();
		std::shared_ptr<RRTXNode> intermediate_node = free_neighbor;
		while (intermediate_node == this->goal_ || intermediate_node->parent != nullptr) {
			if (std::find(solution_path_tmp.begin(), solution_path_tmp.end(), intermediate_node->get_state()) != solution_path_tmp.end()) {
				break;
			}
			this->solution_path_tmp.push_back(intermediate_node->get_state());
			if (intermediate_node == this->goal_) {
				if_find_solution = true;
				this->if_environment_change = false;
				break;
			}
			intermediate_node = intermediate_node->parent;
		}
		if (if_find_solution) break;
	}

	this->if_find_solution_ = if_find_solution;

	if (!if_find_solution) {
		return if_find_solution;
	}

	this->solution_path.clear();
	this->solution_path = this->solution_path_tmp;
	for (int i = 0; i < static_cast<int>(this->solution_path.size()); i++) {
		Eigen::Vector3d ori_point = this->solution_path.at(i);
		ROS_INFO("Solution trajectory is: x=%.2f, y=%.2f, z=%.2f", ori_point(0), ori_point(1), ori_point(2));
	}
	return if_find_solution;
}

void RRTX::publish_without_optimizer() {
	std::cout << "Now the solution path is found, will publish the path without CHOMP planner. The iteration number is: " << this->iteration_count << "The nodes number is: " << static_cast<double>(this->nearest_neighbors_tree_->size()) << std::endl;
	optimized_motion_planner::point_array trajectory_msg;
	int trajectory_size = static_cast<int>(this->solution_path.size());
	geometry_msgs::Point trajectory_point;
	std::string ori_file_name = "ori_data_" + std::to_string(this->path_file_num_) + ".txt";
	std::ofstream ori_data_file (ori_file_name);
	for (int i = 0; i < trajectory_size; i++) {
		Eigen::Vector3d point = this->solution_path.at(i);
		ori_data_file << point(0) << " " << point(1) << " " << point(2) << "\n";
		trajectory_point.x = point(0);
		trajectory_point.y = point(1);
		trajectory_point.z = point(2);
		trajectory_msg.points.push_back(trajectory_point);
		//ROS_INFO("NO chomp: publish trajectory is: x=%.2f, y=%.2f, z=%.2f", point(0), point(1), point(2));
	}

	if (ori_data_file.is_open()) {
		ori_data_file.close();
	}
	this->path_file_num_ += 1;

	ROS_INFO("No: chomp. Publish the trajectory from motion planner");

	ros::Duration(1.0).sleep();
	this->pub_planned_trajectory_.publish(trajectory_msg);
}

void RRTX::publish_with_optimizer() {
	std::cout << "Now the solution path is found, the optimize part begins. The iteration number is: " << this->iteration_count << "The nodes number is: " << static_cast<double>(this->nearest_neighbors_tree_->size()) << std::endl;
	ros::WallTime chomp_start_time = ros::WallTime::now();
	ROS_INFO("Found the solution path, start to optimize");
	auto chomp_trajectory = std::make_shared<CHOMPDynamicTrajectyory>(this->solution_path, this->total_planning_time_, this->discretization_, this->quadrotor_speed_);
	auto chomp = std::make_shared<CHOMPDynamic>(this->optimizer_params_, chomp_trajectory, this->obstacle_map, this->dynamic_obstacle_map_, this->chomp_path_file_num_);
	this->chomp_path_file_num_ += 1;
	std::vector<Eigen::Vector3d> optimized_trajectory = chomp->get_optimized_trajectory();


	for (int i = 0; i < static_cast<int>(optimized_trajectory.size()); i++) {
		Eigen::Vector3d point = optimized_trajectory.at(i);
		std::cout << "##Optimized path is: " << point(0) << " " << point(1) << " " << point(2) << std::endl;
	}


	double chomp_process_time = (ros::WallTime::now() - chomp_start_time).toSec();
	std::cout << "The optimization time is: " << chomp_process_time << std::endl;
	this->optimize_time_path_file << chomp_process_time << "\n";

	optimized_motion_planner::point_array trajectory_msg;
	int trajectory_size = static_cast<int>(optimized_trajectory.size());
	geometry_msgs::Point trajectory_point;
	std::string file_name = "data_" + std::to_string(this->path_file_num_) + ".txt";
	std::ofstream data_file (file_name);
	for (int i = 0; i < trajectory_size; i++) {
		Eigen::Vector3d point = optimized_trajectory.at(i);
		trajectory_point.x = point(0);
		trajectory_point.y = point(1);
		trajectory_point.z = point(2);
		trajectory_msg.points.push_back(trajectory_point);
		data_file << point(0) << " " << point(1) << " " << point(2) << "\n";
		//ROS_INFO("publish trajectory is: x=%.2f, y=%.2f, z=%.2f", point(0), point(1), point(2));
	}
	if (data_file.is_open()) {
		data_file.close();
	}

	int ori_trajectory_size = this->solution_path.size();
	std::string ori_file_name = "ori_data_" + std::to_string(this->path_file_num_) + ".txt";
	std::ofstream ori_data_file (ori_file_name);
	for (int i = 0; i < ori_trajectory_size; i++) {
		Eigen::Vector3d ori_point = this->solution_path.at(i);
		ori_data_file << ori_point(0) << " " << ori_point(1) << " " << ori_point(2) << "\n";
		//ROS_INFO("Solution trajectory is: x=%.2f, y=%.2f, z=%.2f", ori_point(0), ori_point(1), ori_point(2));
	}
	if (ori_data_file.is_open()) {
		ori_data_file.close();
	}

	this->path_file_num_ += 1;

	ROS_INFO("Publish the trajectory from motion planner");
	ros::Duration(1.0).sleep();
	this->pub_planned_trajectory_.publish(trajectory_msg);
}

void RRTX::rewire_neighbors_v1(std::shared_ptr<RRTXNode> random_node) {
	std::vector<std::shared_ptr<RRTXNode>> n_in;
	n_in.insert(n_in.end(), random_node->n0_in.begin(), random_node->n0_in.end());
	n_in.insert(n_in.end(), random_node->nr_in.begin(), random_node->nr_in.end());
	for (auto neighbor : n_in) {
		if (random_node->parent == neighbor) {
			continue;
		}
		double new_lmc_if_choose_random_node_as_parent = random_node->get_lmc() + neighbor->get_neighbor_edge(random_node->get_unique_id());
		bool change_parent= false;
		if (neighbor->parent != this->goal_ && this->is_cost_better_than(new_lmc_if_choose_random_node_as_parent, neighbor->get_lmc())) {
			neighbor->set_lmc(new_lmc_if_choose_random_node_as_parent);
			this->make_parent_of(random_node, neighbor);
			change_parent = true;
		}
		//Todo: is check all not consistent nodes , or only the node that change its parent node, result in in-consistency.
		if ((neighbor->get_g_cost() - neighbor->get_lmc() > this->epsilon_) && (change_parent)) {
			this->verify_queue(neighbor);
		}
	}
}

void RRTX::rewire_neighbors_v2(std::shared_ptr<RRTXNode> node) {
	this->cull_neighbors(node);
	std::vector<std::shared_ptr<RRTXNode>> n_in;
	n_in.insert(n_in.end(), node->n0_in.begin(), node->n0_in.end());
	n_in.insert(n_in.end(), node->nr_in.begin(), node->nr_in.end());
	for (auto neighbor : n_in) {
		if (node->parent == neighbor) {
			continue;
		}
		double new_lmc_if_choose_node_as_parent = node->get_lmc() + neighbor->get_neighbor_edge(node->get_unique_id());
		bool change_parent= false;
		if (neighbor->parent != this->goal_ && this->is_cost_better_than(new_lmc_if_choose_node_as_parent, neighbor->get_lmc())) {
			neighbor->set_lmc(new_lmc_if_choose_node_as_parent);
			this->make_parent_of(node, neighbor);
			change_parent = true;
		}
		if ((neighbor->get_g_cost() - neighbor->get_lmc() > this->epsilon_) && (change_parent)) {
			this->verify_queue(neighbor);
		}
	}
}

void RRTX::make_parent_of(std::shared_ptr<RRTXNode> parent_node, std::shared_ptr<RRTXNode> node) {
	if (node->parent != nullptr) {
		node->parent->children.erase(std::remove(node->parent->children.begin(), node->parent->children.end(), node), node->parent->children.end());
	}
	node->parent = parent_node;
	parent_node->children.push_back(node);
}

void RRTX::verify_queue(std::shared_ptr<RRTXNode> node) {
	if (node->handle != nullptr) {
		this->node_queue.update(node->handle);
	} else {
		node->handle = this->node_queue.insert(node);
	}
}

void RRTX::reduce_inconsistency_v1() {
	while (!this->node_queue.empty()) {
		std::shared_ptr<RRTXNode> min_node = this->node_queue.top()->data;
		min_node->handle = nullptr;
		this->node_queue.pop();

		if (this->check_if_node_inside_all_obstacles(min_node, true)) {
			//std::cout << "V1: Skip one node Now : " << min_node->get_state().x() << " " << min_node->get_state().y() << " " << min_node->get_state().z() << std::endl;
			continue;
		}
		this->rewire_neighbors_v2(min_node);
		min_node->set_g_cost(min_node->get_lmc());
	}

	//std::cout << "Node queue size is: " << this->node_queue.size() << std::endl;
}

void RRTX::reduce_inconsistency_v2() {
	while (!this->node_queue.empty()) {
		std::shared_ptr<RRTXNode> min_node = this->node_queue.top()->data;
		//std::cout << "When update env, the queue node position: " << min->get_state().x() << " " << min->get_state().y() << " " << min->get_state().z() << std::endl;
		min_node->handle = nullptr;
		this->node_queue.pop();
		//this->update_lmc_v1(min_node);

		if (this->check_if_node_inside_all_obstacles(min_node, true)) {
			//std::cout << "V2: Skip one node Now : " << min_node->get_state().x() << " " << min_node->get_state().y() << " " << min_node->get_state().z() << std::endl;
			continue;
		}
		this->update_lmc_v2(min_node);
		this->rewire_neighbors_v2(min_node);
		min_node->set_g_cost(min_node->get_lmc());
	}

	//std::cout << "Node queue size is: " << this->node_queue.size() << std::endl;
}

// find the best parent in N+(v)
void RRTX::update_lmc_v1(std::shared_ptr<RRTXNode> node) {
	this->cull_neighbors(node);

	std::vector<std::shared_ptr<RRTXNode>> n_out;
	n_out.insert(n_out.end(), node->n0_out.begin(), node->n0_out.end());
	n_out.insert(n_out.end(), node->nr_out.begin(), node->nr_out.end());

	for (auto neighbor : n_out) {
		bool is_orphan = false;
		for (auto orphan : this->orphan_node_list) {
			if (orphan == neighbor) {
				is_orphan = true;
				break;
			}
		}

		if (is_orphan || neighbor->parent == node) {
			continue;
		}

		if (std::isinf(neighbor->get_lmc())) continue;

		double inc_cost = node->get_neighbor_edge(neighbor->get_unique_id());  // d(v,u)
		double cost = this->combine_cost(neighbor->get_lmc(), inc_cost); // d(v,u) + lmc(u)

		// if lmc(v) > d(v,u) + lmc(u)
		//std::cout << "In udpate lmc for update env: the distance is: " << cost << ". The original lmc is: " << node->get_lmc() << std::endl;
		if (node->parent != this->goal_ && this->is_cost_better_than(cost, node->get_lmc())) {
			// update lmc(v)
			//todo: if also set to g_cost here
			node->set_lmc(cost);
			// change parent of v to u
			this->make_parent_of(neighbor, node);
		}
	}
}

void RRTX::update_lmc_v2(std::shared_ptr<RRTXNode> node) {
	std::vector<std::shared_ptr<RRTXNode>> n_out;
	n_out.insert(n_out.end(), node->n0_out.begin(), node->n0_out.end());
	n_out.insert(n_out.end(), node->nr_out.begin(), node->nr_out.end());

	for (auto neighbor : n_out) {
		bool is_orphan = false;
		for (auto orphan : this->orphan_node_list) {
			if (orphan == neighbor) {
				is_orphan = true;
				break;
			}
		}

		if (is_orphan || neighbor->parent == node) {
			continue;
		}

		double inc_cost = node->get_neighbor_edge(neighbor->get_unique_id());  // d(v,u)
		double cost = this->combine_cost(neighbor->get_lmc(), inc_cost); // d(v,u) + lmc(u)
		// if lmc(v) > d(v,u) + lmc(u)
		//std::cout << "In udpate lmc for update env: the distance is: " << cost << ". The original lmc is: " << node->get_lmc() << std::endl;
		if (node->parent != this->goal_ && this->is_cost_better_than(cost, node->get_lmc())) {
			// update lmc(v)
			//todo: if also set to g_cost here
			node->set_lmc(cost);
			// change parent of v to u
			this->make_parent_of(neighbor, node);
		}
	}
}

void RRTX::calculateRRG() {
	auto num_of_nodes_in_tree = this->node_list_.size();
	this->rrg_r_ = std::min(this->max_distance_ + 0.5, this->r_rrt_ * std::pow(log(num_of_nodes_in_tree)/num_of_nodes_in_tree, 1/static_cast<double>(this->dimension_)));

	if (this->rrg_r_ < 1) {
		this->rrg_r_ = this->max_distance_ + 0.5;
	}
	//std::cout << "Tree size is: " << num_of_nodes_in_tree << std::endl;
	//std::cout << "!!!!!!!!!!!!!!!The rrg_r_ is " << this->rrg_r_ << std::endl;
}

void RRTX::cull_neighbors(std::shared_ptr<RRTXNode> random_node) {
	for (auto it = random_node->nr_out.begin(); it != random_node->nr_out.end(); ++it) {
		std::shared_ptr<RRTXNode> neighbor = *it;
		if ((this->rrg_r_ + 2< optimized_motion_planner_utils::get_distance(random_node->get_state(), neighbor->get_state())) && random_node->parent != neighbor) {
			random_node->nr_out.erase(it);
			for (auto u_it = neighbor->nr_in.begin(); u_it != neighbor->nr_in.end(); ++u_it) {
				if (*u_it == random_node) {
					neighbor->nr_in.erase(u_it);
					break;
				}
			}
			break;
		}
	}
}

void RRTX::update_neighbors_list(std::shared_ptr<RRTXNode> random_node) {
	std::vector<std::shared_ptr<RRTXNode>> nbh;
	//todo: find nearest R neighbors, the size of r.
	this->nearest_neighbors_tree_->nearestR(random_node, this->rrg_r_ + 2, nbh);

	random_node->nbh.resize(nbh.size());
	// the default bool value of all added nodes are false, the bool value is the feasibility of edge as been tested
	std::transform(nbh.begin(), nbh.end(), random_node->nbh.begin(), [](std::shared_ptr<RRTXNode> node) { return std::pair<std::shared_ptr<RRTXNode>, bool>(node, false);});
}

void RRTX::saturate(std::shared_ptr<RRTXNode> random_node, const std::shared_ptr<RRTXNode>& nearest_node, double distance) const {
	// move the random node to distance = sigma to nearest node here, using interpolate
	Eigen::Vector3d random_node_state = random_node->get_state();
	Eigen::Vector3d nearest_node_state = nearest_node->get_state();
	double x = (random_node_state(0) - nearest_node_state(0)) * this->max_distance_ / distance + nearest_node_state(0);
	double y = (random_node_state(1) - nearest_node_state(1)) * this->max_distance_ / distance + nearest_node_state(1);
	double z = (random_node_state(2) - nearest_node_state(2)) * this->max_distance_ / distance + nearest_node_state(2);
	//ROS_INFO("Change the position of random node to be closer to nearest node");
	//std::cout << "Saturate, new position is: " << x << " " << y << " " << z << std::endl;
	random_node->set_state_by_value(x,y,z);
}

// inserting a new node
bool RRTX::extend(std::shared_ptr<RRTXNode> random_node) {
	//ROS_INFO("In extend now");

	random_node->set_time(this->generate_random_time(random_node->get_state()));

	if (this->check_if_node_inside_all_obstacles(random_node, true)) {
		return false;
	}
	// v is random_node, u is neighbor
	// 1. find all nodes within shrinking hyperball in the nearest tree, and their distance
	this->update_neighbors_list(random_node);
	if (!this->find_best_parent(random_node)) {
		//ROS_INFO("### Cannot find best parent for this random node.");
		return false;
	}

	this->add_node_to_nearest_neighbors_tree(random_node);
	random_node->parent->children.push_back(random_node);

	// after connecting the parent, now is updating list of v's neighbors
	for (auto it = random_node->nbh.begin(); it != random_node->nbh.end(); ++it) {
		std::shared_ptr<RRTXNode> neighbor = it->first;

		if (this->edge_in_free_space_check(random_node, neighbor)) {
			random_node->n0_out.push_back(neighbor);
			neighbor->nr_in.push_back(random_node);
		}
		if (this->edge_in_free_space_check(neighbor, random_node)) {
			neighbor->nr_out.push_back(random_node);
			random_node->n0_in.push_back(neighbor);
		}
	}
	return true;
}

bool RRTX::find_best_parent(std::shared_ptr<RRTXNode> random_node) {
	//std::cout << "Now is in find best parent." << std::endl;
	bool if_find_best_parent = false;
	for (auto it = random_node->nbh.begin(); it != random_node->nbh.end(); ++it) {
		std::shared_ptr<RRTXNode> neighbor = it->first;
		// compute cost using this neighbor as a parent
		double inc_cost = optimized_motion_planner_utils::get_distance(random_node->get_state(), neighbor->get_state());  // d(v,u)
		double cost = this->combine_cost(neighbor->get_lmc(), inc_cost); // d(v,u) + lmc(u)
		// if lmc(v) > d(v,u) + lmc(u)
		//std::cout << "Now check why always not find the best parent" << std::endl;
		//std::cout << "cost is " << cost << " . get_lmc is " << random_node->get_lmc() << " cost < lmc" << std::endl;
		if (this->is_cost_better_than(cost, random_node->get_lmc()) && this->edge_in_free_space_check(random_node, neighbor)) {
			it->second = true;
			// change parent of v to u
			random_node->parent = neighbor;
			neighbor->children.push_back(random_node);
			// update lmc(v)
			random_node->set_lmc(cost);
			if_find_best_parent = true;
		}
	}
	return if_find_best_parent;
}

double RRTX::combine_cost(double cost1, double cost2) {
	return cost1 + cost2;
}

bool RRTX::is_cost_better_than(double cost1, double cost2) {
	return cost1 < cost2;
}


bool RRTX::node_in_free_space_check(const std::shared_ptr<RRTXNode>& random_node) {
	bool result = !this->check_if_node_inside_all_obstacles(random_node, true);
	//std::cout << "Node in free space check result is " << result << std::endl;
	return result;
}

bool RRTX::edge_in_free_space_check(const std::shared_ptr<RRTXNode>& node1, const std::shared_ptr<RRTXNode>& node2) {
	bool result = (!this->check_if_node_inside_all_obstacles(node1, true)) && (!this->check_if_node_inside_all_obstacles(node2, true));
	//std::cout << "Edge in free space check result is " << result << std::endl;
	return result;
}

int RRTX::get_dynamic_obstacle_id(const std::string& dynamic_obstacle_name) {
	std::string delimiter = "_";
	std::string dynamic_obstacle_id_str = dynamic_obstacle_name.substr(dynamic_obstacle_name.find(delimiter)+1, dynamic_obstacle_name.find(delimiter) + delimiter.length()+1);
	int dynamic_obstacle_id = std::stoi(dynamic_obstacle_id_str);
	return dynamic_obstacle_id;
}

void RRTX::obstacle_info_callback(const optimized_motion_planner::obstacle_info::ConstPtr &msg) {
	optimized_motion_planner_utils::Obstacle_type obstacle_type = static_cast<optimized_motion_planner_utils::Obstacle_type>(msg->type);
	if (obstacle_type == optimized_motion_planner_utils::Obstacle_type::dynamic_obstacle) {
		int dynamic_obstacle_id = this->get_dynamic_obstacle_id(msg->name);
		double dynamic_obstacle_size = static_cast<double>(msg->size);
		double dynamic_obstacle_velocity = static_cast<double>(msg->velocity);
		if (std::find(this->dynamic_obstacle_id_list_.begin(), this->dynamic_obstacle_id_list_.end(),dynamic_obstacle_id)!=this->dynamic_obstacle_id_list_.end()) {
			return;
		}
		std::cout << "Initialize dynamic_obstacle_" << dynamic_obstacle_id << " now." << std::endl;
		this->dynamic_obstacle_id_list_.push_back(dynamic_obstacle_id);
		this->exist_dynamic_obstacle_ = true;
		ros::WallTime dynamic_obstacle_start_time = ros::WallTime::now();
		std::cout << "The dynamic_obstacle_" << dynamic_obstacle_id  << " start time is: " << static_cast<double>((dynamic_obstacle_start_time - this->start_time_).toSec()) << std::endl;
		std::cout << "The dynamic obstacle start position is: " << msg->position.x << " " << msg->position.y << " " << msg->position.z << std::endl;
		Eigen::Vector3d start_position(msg->position.x, msg->position.y, msg->position.z);
		this->dynamic_obstacle_map_tmp_[dynamic_obstacle_id] = std::make_shared<DynamicObstacle>(dynamic_obstacle_id, dynamic_obstacle_size, dynamic_obstacle_velocity, start_position, static_cast<double>((dynamic_obstacle_start_time - this->start_time_).toSec()), this->planning_horizon_time_, this->quadrotor_radius_);

		std::cout << "dynamic obstacle map tmp size is " << this->dynamic_obstacle_map_tmp_.size() << std::endl;
		std::cout << "dynamic obstacle map size is " << this->dynamic_obstacle_map_.size() << std::endl;
		return;
	}

	std::cout << "Motion Planner node: callback" << std::endl;
	std::string obstacle_name = msg->name;
	bool obstacle_operation = msg->operation;
	double obstacle_size = static_cast<double>(msg->size);
	Eigen::Vector3d obstacle_position(msg->position.x, msg->position.y, msg->position.z);
	auto obstacle = std::make_shared<Obstacle>(obstacle_name, obstacle_type, obstacle_operation, obstacle_size, obstacle_position);
	ros::WallTime obstacle_start_time = ros::WallTime::now();
	std::cout << "The obstacle start time is: " << static_cast<double>((obstacle_start_time - this->start_time_).toSec()) << std::endl;
	this->obstacle_update_info_list.push_back(obstacle);
}

void RRTX::dynamic_obstacle_movement_callback(const optimized_motion_planner::obstacle_info::ConstPtr &msg) {
	int dynamic_obstacle_id = this->get_dynamic_obstacle_id(msg->name);
	if (!this->dynamic_obstacle_map_tmp_[dynamic_obstacle_id]->get_if_move()) {
		std::cout << "To get second pos of dynamic_obstacle_" << dynamic_obstacle_id << " ." << std::endl;
		Eigen::Vector2d dynamic_obstacle_position(msg->position.x, msg->position.y);
		this->dynamic_obstacle_map_tmp_[dynamic_obstacle_id]->set_second_position(dynamic_obstacle_position);
		this->dynamic_obstacle_map_tmp_[dynamic_obstacle_id]->set_if_move(true);
		this->dynamic_obstacle_map_[dynamic_obstacle_id] = this->dynamic_obstacle_map_tmp_[dynamic_obstacle_id];
		std::cout << "In movement: Dynamic obstacle map tmp size is " << this->dynamic_obstacle_map_tmp_.size() << std::endl;
		std::cout << "In movement: Dynamic obstacle map size is " << this->dynamic_obstacle_map_.size() << std::endl;
	}
}

void RRTX::update_obstacle() {
	//ROS_INFO("Update obstacle now");
	bool add_obstacle = false;
	bool remove_obstacle = false;

	for (auto obstacle : this->obstacle_update_info_list) {
		if (!obstacle->get_operation()) {
			std::cout << "Add this ob: " << obstacle->get_position() << " " << obstacle->get_size() << " " << obstacle->get_name() << " " << obstacle->get_type() << std::endl;
			this->obstacle_map.erase(obstacle->get_name());
			this->remove_obstacle(obstacle);
			remove_obstacle = true;
		}
	}
	if (remove_obstacle) {
		this->reduce_inconsistency_v2();
		this->if_environment_change = true;
	}

	for (auto obstacle : this->obstacle_update_info_list) {
		if (obstacle->get_operation()) {
			std::cout << "Add this ob: " << obstacle->get_position() << " " << obstacle->get_size() << " " << obstacle->get_name() << " " << obstacle->get_type() << std::endl;
			this->obstacle_map[obstacle->get_name()] = obstacle;
			this->add_obstacle(obstacle);
			add_obstacle = true;
		}
	}

	for (auto dynamic_obstacle : this->dynamic_obstacle_map_) {
		if ((!dynamic_obstacle.second->get_if_add()) && dynamic_obstacle.second->get_if_move()) {
			std::cout << "Adding dynamic_obstacle_" << dynamic_obstacle.first << " as an obstacle." << std::endl;
			this->add_dynamic_obstacle(dynamic_obstacle.first);
			add_obstacle = true;
			dynamic_obstacle.second->set_if_add(true);
		}
	}

	if (add_obstacle) {
		ROS_INFO("Adding obstacle now.");
		this->propogate_descendants();
		this->reduce_inconsistency_v2();
		this->if_environment_change = true;
	}

	this->obstacle_update_info_list.clear();
}

void RRTX::remove_obstacle(const std::shared_ptr<Obstacle>& obstacle) {
	ROS_INFO("Start to removing the ob");
	// need to update this format to all for, and consider const reference
	for (auto node : this->node_list_) {
		if (!check_if_node_inside_obstacle(obstacle, node) || check_if_node_inside_all_obstacles(node, true)) continue;
		for (auto child_node : node->children) {
			//todo: check if only child to parent node, or need to set edge of parent to child also to inf, and also recalculate in remove obstacle step
			child_node->update_neighbor_edge_list(node->get_unique_id(), optimized_motion_planner_utils::get_distance(child_node->get_state(), node->get_state()));
			//todo: des not update lmc here, only one time update lmc in reduce-inconsistency
			//this->update_lmc_v1(child_node);
			this->verify_queue(child_node);
		}
	}
}

bool RRTX::check_if_node_inside_obstacle(const std::shared_ptr<Obstacle>& obstacle, const std::shared_ptr<RRTXNode>& node) {
	double distance = optimized_motion_planner_utils::get_distance(obstacle->get_position(), node->get_state());
	//std::cout << "The distance to obstacle is: " << distance << std::endl;
	if (distance - this->quadrotor_radius_ - this->min_clearence_ < obstacle->get_size()) {
		//Eigen::Vector3d node_state = node->get_state();
		//std::cout << "Random node the node state is: " << node_state(0) << " " << node_state(1) << " " << node_state(2) << std::endl;
		return true;
	}
	return false;
}

bool RRTX::check_if_node_inside_obstacle_for_adding_ob(const std::shared_ptr<Obstacle>& obstacle, const std::shared_ptr<RRTXNode>& node) {
	double distance = optimized_motion_planner_utils::get_distance(obstacle->get_position(), node->get_state());
	//std::cout << "The distance to obstacle is: " << distance << " . And the minus is: " << distance - this->quadrotor_radius_ - obstacle->get_size() << std::endl;
	//std::cout << "obstacle size: " <<  obstacle->get_size() << std::endl;
	if (distance - this->quadrotor_radius_ - this->min_clearence_ < obstacle->get_size()) {
		Eigen::Vector3d node_state = node->get_state();
		//std::cout << "remove node the node state is: " << node_state(0) << " " << node_state(1) << " " << node_state(2) << std::endl;
		return true;
	}
	return false;
}

bool RRTX::check_if_node_inside_all_obstacles(const std::shared_ptr<RRTXNode>& node, bool consider_dynamic_obstacle) {
	//std::cout << "OBSTACLE MAP SIZE is: " << this->obstacle_map.size() << std::endl;
	if (consider_dynamic_obstacle && this->exist_dynamic_obstacle_) {
		for (auto dynamic_obstacle : this->dynamic_obstacle_map_) {
			if (dynamic_obstacle.second->check_if_node_inside_dynamic_obstacle(node)) {
				//Eigen::Vector3d node_state = node->get_state();
				//std::cout << "Check if node inside dynamic obstacle: the node state is: " << node_state(0) << " " << node_state(1) << " " << node_state(2) << std::endl;
				return true;
			}

		}
	}

	bool result = std::any_of(this->obstacle_map.begin(), this->obstacle_map.end(), [this, node](auto obstacle){
		return this->check_if_node_inside_obstacle(obstacle.second, node);
	});

	return result;
}

void RRTX::add_dynamic_obstacle(int dynamic_obstacle_id) {
	double inf = std::numeric_limits<double>::infinity();
	for (auto node : this->node_list_) {
		if (!this->dynamic_obstacle_map_[dynamic_obstacle_id]->check_if_node_inside_dynamic_obstacle(node)) continue;
		std::cout << "The colliding with dynamic obstacle node position is: " << node->get_state().x() << " " << node->get_state().y() << " " << node->get_state().z() << " time:" << node->get_time() << std::endl;
		for (auto child_node : node->children) {
			//todo: check If only child to parent node, or need to set edge of parent to child also to inf, and also recalculate in remove obstacle step
			child_node->update_neighbor_edge_list(node->get_unique_id(), std::numeric_limits<double>::infinity());
			child_node->set_g_cost(inf, true);
			child_node->set_lmc(inf, true);
			child_node->parent = nullptr;
			this->verify_orphan(child_node);
		}
		node->children.clear();
	}
}

void RRTX::add_obstacle(const std::shared_ptr<Obstacle>& obstacle) {
	std::cout << "When adding obstacle, the total nodes list number is: " << this->node_list_.size() << std::endl;
	std::cout << "Obstacle position is: " << obstacle->get_position().x() << " "<< obstacle->get_position().y() << " " << obstacle->get_position().z() << std::endl;
	std::cout << "Obstacle size is: " << obstacle->get_size() << std::endl;
	double inf = std::numeric_limits<double>::infinity();
	for (auto node : this->node_list_) {
		if (!check_if_node_inside_obstacle_for_adding_ob(obstacle, node)) continue;
		for (auto child_node : node->children) {
			//todo: check If only child to parent node, or need to set edge of parent to child also to inf, and also recalculate in remove obstacle step
			child_node->update_neighbor_edge_list(node->get_unique_id(), std::numeric_limits<double>::infinity());
			child_node->set_g_cost(inf, true);
			child_node->set_lmc(inf, true);
			child_node->parent = nullptr;
			this->verify_orphan(child_node);
		}
		node->children.clear();
	}
}

double RRTX::get_distance_check_ob(const std::shared_ptr<RRTXNode>& node1, const std::shared_ptr<RRTXNode>& node2) {
	if (this->check_if_node_inside_all_obstacles(node1, true) || this->check_if_node_inside_all_obstacles(node2, true)) {
		return std::numeric_limits<double>::infinity();
	}
	double inf = std::numeric_limits<double>::infinity();
	double distance1 = inf;
	double distance2 = inf;

	for (auto obstacle : this->obstacle_map) {
		Eigen::Vector3d ob_pos = obstacle.second->get_position();
		double size = obstacle.second->get_size();
		double distance_tmp_1 = optimized_motion_planner_utils::get_distance(node1->get_state(), ob_pos) - size;
		double distance_tmp_2 = optimized_motion_planner_utils::get_distance(node2->get_state(), ob_pos) - size;
		if (distance_tmp_1 < distance1) distance1 = distance_tmp_1;
		if (distance_tmp_2 < distance2) distance2 = distance_tmp_2;
	}
	//std::cout << "The distance is: " << distance1 << " " << distance2 << std::endl;

	if (distance1 < 0 || distance2 < 0) {
		return inf;
	} else {
		return optimized_motion_planner_utils::get_distance(node1->get_state(), node2->get_state());
	}
}
}
