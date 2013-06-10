#include "whole_body_planner/PlannerEmpty.h"

PlannerEmpty::PlannerEmpty()
{

}

PlannerEmpty::~PlannerEmpty()
{
    delete wbc_;
}

bool PlannerEmpty::computeConstraints(const amigo_whole_body_controller::ArmTaskGoal& goal_constraint, std::vector<amigo_whole_body_controller::ArmTaskGoal>& constraints)
{
    constraints.resize(1);
    constraints[0] = goal_constraint;
    return true;
}
