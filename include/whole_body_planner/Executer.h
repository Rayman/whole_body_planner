/*!
 * \author Janno Lunenburg
 * \date June 2013
 * \version 0.1
 */

#ifndef EXECUTER_H_
#define EXECUTER_H_

#include <amigo_whole_body_controller/ArmTaskAction.h>
#include <actionlib/client/simple_action_client.h>

class Executer
{

public:

    /**
      * Constructor
      */
    Executer();

    /**
      * Deconstructor
      */
    virtual ~Executer();

    /**
      * Execute
      * @param constraints: vector of constraints that is sent to the whole body controller
      */
    //bool Execute(const std::vector<amigo_whole_body_controller::ArmTaskGoal*> constraints);
    bool Execute(const std::vector<amigo_whole_body_controller::ArmTaskGoal>& constraints);

    /** Returns current state
      * @return Current state */
    std::string getCurrentState();


protected:

private:

    actionlib::SimpleActionClient<amigo_whole_body_controller::ArmTaskAction>* action_client_;

    /** Keeps track of the current state of the robot */
    std::string current_state_;

};

#endif

