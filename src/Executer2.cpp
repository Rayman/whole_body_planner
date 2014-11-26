#include "whole_body_planner/Executer2.h"

Executer2::Executer2()
    : current_state_("reset"), wbc_client("/add_motion_objective")
{
    ROS_INFO("Connecting to the whole body controller...");
    wbc_client.waitForActionServerToStart();
    ROS_INFO("Connected");
}

bool Executer2::Execute(const std::vector<amigo_whole_body_controller::ArmTaskGoal>& constraints)
{
    for (std::vector<amigo_whole_body_controller::ArmTaskGoal>::const_iterator it = constraints.begin();
            it != constraints.end(); ++it) {
        const amigo_whole_body_controller::ArmTaskGoal goal = *it;

        wbc_client.sendGoal(goal,
                            boost::bind(&Executer2::transition_cb, this, _1),
                            boost::bind(&Executer2::feedback_cb,   this, _1, _2));
    }
    return true;
}

void Executer2::feedback_cb(ArmTaskClient::GoalHandle goal_handle, const amigo_whole_body_controller::ArmTaskFeedbackConstPtr &feedback)
{

}

void Executer2::transition_cb(ArmTaskClient::GoalHandle goal_handle)
{

}

std::string Executer2::getCurrentState()
{
    return current_state_;
}
