#include "whole_body_planner/Executer.h"

Executer::Executer()
{

    ros::NodeHandle nh_private("~");

    action_client_ = new actionlib::SimpleActionClient<amigo_whole_body_controller::ArmTaskAction>("/add_motion_objective", true);
    ROS_INFO("Waiting for whole body controller");
    action_client_->waitForServer();
    ROS_INFO("Connected to server, executer initialized");

}

Executer::~Executer()
{

    delete action_client_;

}

bool Executer::Execute(const std::vector<amigo_whole_body_controller::ArmTaskGoal> &constraints)
{
    ROS_WARN("No timeouts defined yet");

    /// Send goals to the whole body controller
    for (unsigned int i = 0; i < constraints.size(); i++)
    {
        //ROS_WARN("Tip frame = %s",constraints[i].position_constraint.link_name.c_str());
        action_client_->sendGoal(constraints[i]);
        action_client_->waitForResult();
    }

    /// Check state
    if (action_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Failed to meet all goal constraints");
        return false;
    }
    else
    {
        return true;
    }
}
