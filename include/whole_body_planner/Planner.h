/*!
 * \author Janno Lunenburg
 * \date June 2013
 * \version 0.1
 */

#ifndef PLANNER_H_
#define PLANNER_H_

#include <WholeBodyController.h>
#include <amigo_whole_body_controller/ArmTaskAction.h>

/**
  * \brief Abstract class for planners
  */
class Planner
{

public:

    /**
      * Constructor
      */
    Planner()
    {
        // ToDo: don't hardcode
        wbc_ = new WholeBodyController(0.05);
    }

    /**
      * Deconstructor
      */
    virtual ~Planner(){};

    /**
      * Computes series of constraints
      * @param goal_constraint: goal constraint
      * @param constraints: vector containing pointers to goal constraints
      */
    virtual bool computeConstraints(const amigo_whole_body_controller::ArmTaskGoal& goal_constraint, std::vector<amigo_whole_body_controller::ArmTaskGoal>& constraints)=0;

    /**
      * Sets initial conditions of the whole body controller
      * @param map containing joint names and corresponding joint positions
      */
    // ToDo: make const (need to change wbc as well
    bool setInitialJointPositions(std::map<std::string, double>& joint_position_map)
    {
        for (std::map<std::string, double>::iterator it = joint_position_map.begin(); it != joint_position_map.end(); ++it)
        {
            wbc_->setMeasuredJointPosition(it->first, it->second);
        }
        return true;
    }

protected:

    /**
      * Whole body controller object for forward simulation
      */
    WholeBodyController* wbc_;

};

#endif
