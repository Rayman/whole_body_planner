#include "whole_body_planner/Executer2.h"

Executer2::Executer2()
    : current_state_("reset")
{
}

bool Executer2::Execute(const std::vector<amigo_whole_body_controller::ArmTaskGoal>& constraints)
{
    return true;
}

std::string Executer2::getCurrentState()
{
    return current_state_;
}
