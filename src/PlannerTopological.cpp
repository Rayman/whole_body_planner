#include "whole_body_planner/PlannerTopological.h"

PlannerTopological::PlannerTopological()
{
    ROS_INFO("Initializing plannertopological");
    current_state_ = "reset";
    ROS_INFO("Initialized plannertopological");
}

PlannerTopological::~PlannerTopological()
{
}

bool PlannerTopological::computeConstraints(const amigo_whole_body_controller::ArmTaskGoal& goal_constraint, std::vector<amigo_whole_body_controller::ArmTaskGoal>& constraints)
{
    //constraints.resize(1);
    //constraints[0] = goal_constraint;

    /// Update the connectivitygraph according to the goal
    connectivity_graph_.createGraph(goal_constraint);

    /// Get a path from this goal
    constraints = connectivity_graph_.getPlan(current_state_,goal_constraint.goal_type);

    ROS_DEBUG("Constraints: size = %i",(int)constraints.size());

    std::vector<amigo_whole_body_controller::ArmTaskGoal>::iterator iter = constraints.begin();
    constraints.erase(iter);

    ROS_INFO("Constraints after erasing first: size = %i",(int)constraints.size());

    /// Update the current state
    // ToDo: make this more generic
    //current_state_ = goal_constraint.goal_type;
    current_state_ = "reset";
    return true;
}
