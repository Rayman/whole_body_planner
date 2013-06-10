#include "whole_body_planner/WholeBodyPlanner.h"

WholeBodyPlanner::WholeBodyPlanner()
{

    ros::NodeHandle nh_private("~");

    /// Action servers
    action_server_ = new actionlib::SimpleActionServer<amigo_whole_body_controller::ArmTaskAction>(nh_private, "motion_constraint", false);
    action_server_->registerGoalCallback(boost::bind(&WholeBodyPlanner::goalCB, this));
    action_server_->start();

    // ToDo: depend on parameter (e.g. in launchfile)
    planner_ = 0;

}

WholeBodyPlanner::~WholeBodyPlanner()
{

    delete action_server_;

}

void WholeBodyPlanner::goalCB()
{
    const amigo_whole_body_controller::ArmTaskGoal& goal = *action_server_->acceptNewGoal();

    /// Get initial positions from robot interface
    std::map<std::string, double> joint_position_map = robot_state_interface_.getJointPositions();

    /// Compute constraints
    bool plan_result = false;
    if (planner_ == 0)
    {
        planner_empty_.setInitialJointPositions(joint_position_map);
        plan_result = planner_empty_.computeConstraints(goal, constraints_);
    }

    /// If succeeded, send to whole body controller
    bool execute_result = false;
    if (plan_result)
    {
        execute_result = executer_.Execute(constraints_);
    }

    /// If succeeded, set server succeeded
    if (execute_result)
    {
        action_server_->setSucceeded();
    }
    else
    {
        action_server_->setAborted();
    }

}
