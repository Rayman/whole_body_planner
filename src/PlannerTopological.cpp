#include "whole_body_planner/PlannerTopological.h"

PlannerTopological::PlannerTopological()
{
    current_state_ = "reset";
}

PlannerTopological::~PlannerTopological()
{
    delete wbc_;
}

bool PlannerTopological::computeConstraints(const amigo_whole_body_controller::ArmTaskGoal& goal_constraint, std::vector<amigo_whole_body_controller::ArmTaskGoal>& constraints)
{
    //constraints.resize(1);
    //constraints[0] = goal_constraint;

    /// Update the connectivitygraph according to the goal
    connectivity_graph_.createGraph(goal_constraint);

    /// Get a path from this goal
    constraints = connectivity_graph_.getPlan(current_state_,goal_constraint.goal_type);

    ROS_INFO("Constraints: size = %i",constraints.size());

    /// Update the current state
    current_state_ = goal_constraint.goal_type;
    return true;
}
