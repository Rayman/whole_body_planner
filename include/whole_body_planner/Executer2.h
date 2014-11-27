#ifndef EXECUTER2_H
#define EXECUTER2_H

#include <amigo_whole_body_controller/ArmTaskAction.h>
#include <actionlib/client/action_client.h>

namespace wbc_codes {
    enum {
        AT_GOAL_POSE = amigo_whole_body_controller::WholeBodyControllerStatus::AT_GOAL_POSE,
        MOVING_TO_GOAL_POSE = amigo_whole_body_controller::WholeBodyControllerStatus::MOVING_TO_GOAL_POSE
    };
}

class Executer2
{
public:
    Executer2();

    /**
      * Execute
      * @param constraints: vector of constraints that is sent to the whole body controller
      */
    bool Execute(const std::vector<amigo_whole_body_controller::ArmTaskGoal>& constraints);

    /** Returns current state
      * @return Current state */
    std::string getCurrentState();

protected:
    std::string current_state_;

    typedef actionlib::ActionClient<amigo_whole_body_controller::ArmTaskAction> ArmTaskClient;
    ArmTaskClient wbc_client;

    void feedback_cb(ArmTaskClient::GoalHandle goal_handle, const amigo_whole_body_controller::ArmTaskFeedbackConstPtr &feedback);

    void transition_cb(ArmTaskClient::GoalHandle goal_handle);

    ros::Rate rate;
    bool is_done_;
};

#endif // EXECUTER2_H
