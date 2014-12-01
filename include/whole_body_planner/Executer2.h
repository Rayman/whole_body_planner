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

    /**
     * Keep a map from /frame+/link_name to the corresponding goal handle.
     * This will be used to cancel previous goals with the same key
     */
    typedef std::pair<std::string, std::string> goal_key;
    goal_key make_key(std::string frame_id, std::string link_name) { return std::make_pair(frame_id, link_name); }
    std::map<goal_key, ArmTaskClient::GoalHandle> goal_map;

    ros::Rate rate;
    bool is_done_;
};

#endif // EXECUTER2_H
