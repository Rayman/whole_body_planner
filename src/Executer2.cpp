#include "whole_body_planner/Executer2.h"

Executer2::Executer2()
    : current_state_("reset"), wbc_client("/add_motion_objective"), rate(50), is_done_(false)
{
}

bool Executer2::Execute(const std::vector<amigo_whole_body_controller::ArmTaskGoal>& constraints)
{
    ROS_INFO("Connecting to the whole body controller...");
    // unfortunately this cannot be done in the contructor because then this will hang... (ros bug)
    wbc_client.waitForActionServerToStart();

    int i = 0;
    for (std::vector<amigo_whole_body_controller::ArmTaskGoal>::const_iterator it = constraints.begin();
            it != constraints.end(); ++it) {
        const amigo_whole_body_controller::ArmTaskGoal goal = *it;

        ros::Time start_time = ros::Time::now();
        ROS_INFO("Sending goal...");

        ArmTaskClient::GoalHandle handle = wbc_client.sendGoal(goal,
                            boost::bind(&Executer2::transition_cb, this, _1),
                            boost::bind(&Executer2::feedback_cb,   this, _1, _2));

        is_done_ = false;
        while (ros::ok() && !is_done_ && (ros::Time::now() - start_time) < ros::Duration(40.0)) {
            ros::spinOnce();
            rate.sleep();
        }

        if (is_done_) {
            handle.cancel();
            ROS_INFO("Constraint %d is valid", ++i);
            current_state_ = goal.goal_type;
        } else {
            ROS_WARN("Constraint %d is NOT valid, stopping", ++i);
            handle.cancel();
            return false;
        }

        ROS_INFO_STREAM("Execution of constraint took: " << ros::Time::now() - start_time);
    }

    return true;
}

void Executer2::feedback_cb(ArmTaskClient::GoalHandle goal_handle, const amigo_whole_body_controller::ArmTaskFeedbackConstPtr &feedback)
{
    const amigo_whole_body_controller::WholeBodyControllerStatus &code = feedback->status_code;

    switch (code.status) {
    case wbc_codes::AT_GOAL_POSE:
        is_done_ = true;
        break;
    case wbc_codes::MOVING_TO_GOAL_POSE:
        break;
    default:
        ROS_WARN("unknown feedback status code: %i", code.status);
        break;
    }

}

void Executer2::transition_cb(ArmTaskClient::GoalHandle goal_handle)
{
    ROS_INFO("transition_cb: %s", goal_handle.getCommState().toString().c_str());
}

std::string Executer2::getCurrentState()
{
    return current_state_;
}
