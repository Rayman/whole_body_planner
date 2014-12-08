#include "whole_body_planner/Executer2.h"

Executer2::Executer2()
    : current_state_("reset"), wbc_client("/add_motion_objective"), rate(150), is_done_(false)
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

        std::string frame_id = goal.position_constraint.header.frame_id;
        std::string link_name = goal.position_constraint.link_name;
        goal_key key = make_key(frame_id, link_name);

        ArmTaskClient::GoalHandle old_handle = goal_map[key];

        if (!old_handle.isExpired()) {
            ROS_INFO("cancelling old handle (%s,%s)", frame_id.c_str(), link_name.c_str());
            old_handle.cancel(); // cancel if there was a previous goal
        }

        ROS_WARN("sending new goal: (%s,%s)", frame_id.c_str(), link_name.c_str());
        active_goal = wbc_client.sendGoal(goal,
                            boost::bind(&Executer2::transition_cb, this, _1),
                            boost::bind(&Executer2::feedback_cb,   this, _1, _2));
        goal_map[key] = active_goal;

        for (std::map<goal_key, ArmTaskClient::GoalHandle>::const_iterator hit = goal_map.begin(); hit != goal_map.end(); ++hit) {
            const goal_key k = hit->first;
            const ArmTaskClient::GoalHandle gh = hit->second;
            ROS_INFO("\thandle (%s %s): %s", k.first.c_str(), k.second.c_str(), gh.isExpired() ? "expired" : "active");
        }

        is_done_ = false;
        while (ros::ok() && !is_done_ && (ros::Time::now() - start_time) < ros::Duration(40.0)) {
            ros::spinOnce();
            rate.sleep();
        }

        if (is_done_) {
            ROS_INFO("the finished goal has state %s", active_goal.getCommState().toString().c_str());

            /**
             * Trough feedback we determine if the goal converged. This means that the goal will
             * still be active when is_done_. When a goal has been cancelled, the state == DONE
             * and we check the result.
             */

            if (active_goal.getCommState() == actionlib::CommState::DONE
                && active_goal.getResult()->status_code.status != wbc_codes::AT_GOAL_POSE) {
                // something went wrong, we did not converge
                ROS_WARN("Constraint %d was cancelled, stopping", ++i);
                return false;
            }

            // only for the last constraint, don't cancel
            if ((it+1) != constraints.end()) {
                ROS_INFO("cancelling intermediate goal (%s,%s)", frame_id.c_str(), link_name.c_str());
                active_goal.cancel();
            }

            ROS_INFO("Constraint %d is valid", ++i);
            current_state_ = goal.goal_type;
        } else {
            ROS_WARN("Constraint %d did not converge, stopping", ++i);
            active_goal.cancel();
            return false;
        }

        ROS_INFO_STREAM("Execution of constraint took: " << ros::Time::now() - start_time);
    }

    return true;
}

void Executer2::feedback_cb(ArmTaskClient::GoalHandle goal_handle, const amigo_whole_body_controller::ArmTaskFeedbackConstPtr &feedback)
{
    const amigo_whole_body_controller::WholeBodyControllerStatus &code = feedback->status_code;
    ROS_DEBUG("status: %i", code.status);

    if (goal_handle != active_goal ) {
        // feedback is from a different goal, so just ignore it
        return;
    }

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
    std::string state = goal_handle.getCommState().toString();

    ROS_DEBUG("transition_cb: %s", state.c_str());

    if (goal_handle == active_goal) {
        switch (goal_handle.getCommState().state_) {
        case actionlib::CommState::WAITING_FOR_GOAL_ACK:
        case actionlib::CommState::ACTIVE:
            // good
            break;
        default:
            // every other CommState will result in the goal being NOT active,
            // so we should stop the execution
            if (!is_done_) {
                ROS_INFO("current goal is not active (%s) -> stopping execution", state.c_str());
                is_done_ = true;
            }
            break;
        }
    }

    // cleanup old goal handles
    if (goal_handle.getCommState() == actionlib::CommState::DONE) {
        // we don't have the goal here, instead just search for the handle
        for (std::map<goal_key, ArmTaskClient::GoalHandle>::iterator it = goal_map.begin(); it != goal_map.end(); ) { // no it++ here
            ArmTaskClient::GoalHandle &cur = it->second;
            if (cur == goal_handle) {
                ROS_DEBUG("removed old goal from the goal_map");
                goal_map.erase(it++);
            } else {
                ++it;
            }
        }
    }
}

std::string Executer2::getCurrentState()
{
    return current_state_;
}
