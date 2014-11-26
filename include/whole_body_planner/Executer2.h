#ifndef EXECUTER2_H
#define EXECUTER2_H

#include <amigo_whole_body_controller/ArmTaskAction.h>

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

};

#endif // EXECUTER2_H
