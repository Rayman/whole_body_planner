#include "whole_body_planner/Executer.h"

Executer::Executer()
{

	ros::NodeHandle nh_private("~");
    action_client_ = new actionlib::SimpleActionClient<amigo_whole_body_controller::ArmTaskAction>("/add_motion_objective", true);
    ROS_INFO("Waiting for whole body controller");
    action_client_->waitForServer();

    current_state_ = "reset";

    ROS_INFO("Connected to server, executer initialized");
}

Executer::~Executer()
{

    delete action_client_;
}

bool Executer::Execute(const std::vector<amigo_whole_body_controller::ArmTaskGoal> &constraints)
{
    //ROS_WARN("No timeouts defined yet");


    /// Send goals to the whole body controller
    for (unsigned int i = 0; i < constraints.size(); i++)
    {
        ros::Time start_time = ros::Time::now();
        action_client_->sendGoalAndWait(constraints[i],ros::Duration(40.0));

        if (action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Constraint %d is valid", i);
            current_state_ = constraints[i].goal_type;
        }
        else
        {
            std::cout<<"Execution of constraint took: "<<ros::Time::now() - start_time<<std::endl;
            break;
        }

    }

    /// Check state
    if (action_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Failed to meet all goal constraints");
        return false;
    }
    else
    {
        ROS_INFO("Arrived at goal!");
        return true;
    }

    return true;
}

std::string Executer::getCurrentState() {
    return current_state_;
}
