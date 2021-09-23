#include "optimizer/CHOMP_dynamic_cost.hpp"

namespace optimized_motion_planner {

CHOMPDynamicCost::CHOMPDynamicCost(const std::shared_ptr<CHOMPDynamicTrajectyory>& trajectory, const std::vector<double> &derivative_costs,
					 double ridge_factor) {
	int num_vars_all = trajectory->get_num_points_diff();
	int num_vars_free = num_vars_all - 2 * (optimized_motion_planner_utils::DIFF_RULE_LENGTH - 1);
	Eigen::MatrixXd diff_matrix = Eigen::MatrixXd::Zero(num_vars_all, num_vars_all);
	this->quad_cost_full_ = Eigen::MatrixXd::Zero(num_vars_all, num_vars_all);

	double multiplier = 1.0;
	for (unsigned int i = 0; i < derivative_costs.size(); i++) {
		multiplier *= trajectory->get_discretization();
		diff_matrix = getDiffMatrix(num_vars_all, &optimized_motion_planner_utils::DIFF_RULES[i][0]);
		Eigen::MatrixXd matrix_product = diff_matrix.transpose() * diff_matrix;
		this->quad_cost_full_ += (derivative_costs[i] * multiplier) * matrix_product;
	}
	this->quad_cost_full_ += Eigen::MatrixXd::Identity(num_vars_all, num_vars_all) * ridge_factor;

	quad_cost_ = quad_cost_full_.block(optimized_motion_planner_utils::DIFF_RULE_LENGTH - 1, optimized_motion_planner_utils::DIFF_RULE_LENGTH - 1, num_vars_free, num_vars_free);
	quad_cost_inv_ = quad_cost_.inverse();
}

Eigen::MatrixXd CHOMPDynamicCost::getDiffMatrix(int size, const double* diff_rule) const
{
	Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(size, size);
	for (int i = 0; i < size; i++)
	{
		for (int j = -optimized_motion_planner_utils::DIFF_RULE_LENGTH / 2; j <= optimized_motion_planner_utils::DIFF_RULE_LENGTH / 2; j++)
		{
			int index = i + j;
			if (index < 0)
				continue;
			if (index >= size)
				continue;
			matrix(i, index) = diff_rule[j + optimized_motion_planner_utils::DIFF_RULE_LENGTH / 2];
		}
	}
	return matrix;
}

void CHOMPDynamicCost::scale(double scale)
{
	double inv_scale = 1.0 / scale;
	this->quad_cost_inv_ *= inv_scale;
	this->quad_cost_ *= scale;
	this->quad_cost_full_ *= scale;
}

double CHOMPDynamicCost::getCost(const Eigen::MatrixXd::ColXpr &joint_trajectory) const {
	return joint_trajectory.dot(this->quad_cost_full_ * joint_trajectory);
}

Eigen::MatrixXd CHOMPDynamicCost::getDerivative(const Eigen::MatrixXd::ColXpr &joint_trajectory) const {
	Eigen::MatrixXd derivative = this->quad_cost_full_ * (2.0 * joint_trajectory);
	return derivative;
}

}