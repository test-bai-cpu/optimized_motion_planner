#pragma once

#include <ros/ros.h>

#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <vector>
#include <limits>
#include <ompl/datastructures/BinaryHeap.h>

namespace optimized_motion_planner {

class RRTXNode {
public:
	RRTXNode() = default;
	RRTXNode(const Eigen::Vector3d& state, double time_from_start = 0);
	~RRTXNode();

	// for nearest neighbor tree
	std::shared_ptr<RRTXNode> parent;
	std::vector<std::shared_ptr<RRTXNode>> children;

	// for extend part
	std::vector<std::pair<std::shared_ptr<RRTXNode>, bool>> nbh;

	// for neighbors, original in is N0-(v), out is N0+(v), running in is Nr-(v), out is Nr+(v)
	std::vector<std::shared_ptr<RRTXNode>> n0_in; // -
	std::vector<std::shared_ptr<RRTXNode>> n0_out; // +
	std::vector<std::shared_ptr<RRTXNode>> nr_in; // -
	std::vector<std::shared_ptr<RRTXNode>> nr_out; // +

	struct node_compare{
		bool operator()(const std::shared_ptr<RRTXNode>& a, const std::shared_ptr<RRTXNode>& b){
			double a_min_cost = std::min(a->get_lmc(), a->get_g_cost());
			double b_min_cost = std::min(b->get_lmc(), b->get_g_cost());

			if (a_min_cost == b_min_cost) return a->get_g_cost() > b->get_g_cost();
			return a_min_cost > b_min_cost;
		}
	};

	// for check if the node is in the priority queue
	ompl::BinaryHeap<std::shared_ptr<RRTXNode>, node_compare>::Element *handle;

	void set_lmc(double lmc, bool if_inf = false) {
		if (if_inf) {
			this->lmc_ = std::numeric_limits<double>::infinity();
		} else {
			this->lmc_ = lmc;
		}
	};

	double get_lmc() const {
		return this->lmc_;
	};

	void set_g_cost(double g_cost, bool if_inf = false) {
		if (if_inf) {
			this->g_cost_ = std::numeric_limits<double>::infinity();
		} else {
			this->g_cost_ = g_cost;
		}
	};

	double get_g_cost() const {
		return this->g_cost_;
	};

	void set_time(double time_from_start) {
		this->time_ = time_from_start;
	};

	double get_time() const {
		return this->time_;
	};

	void set_state_by_vector(const Eigen::Vector3d& new_state) {
		this->state_ = new_state;
	};

	void set_state_by_value(double x, double y, double z) {
		this->state_(0) = x;
		this->state_(1) = y;
		this->state_(2) = z;
	};

	Eigen::Vector3d get_state() const {
		return this->state_;
	};

	void set_unique_id(int unique_id) {
		this->unique_id_ = unique_id;
	}

	int get_unique_id() {
		return this->unique_id_;
	}

	void add_to_neighbor_edge_list(double edge);
	void update_neighbor_edge_list(int neighbor_index, double edge);
	double get_neighbor_edge(int neighbor_index);
private:
	// state of quadrotor
	Eigen::Vector3d state_;

	// cost-to-goal
	double lmc_;
	double g_cost_;

	// time from start
	double time_{0};

	int unique_id_;

	std::vector<double> neighbor_edge_list_;
};


}