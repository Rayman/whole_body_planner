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
      * Deconstructor
      */
    virtual ~Planner(){};

    /**
      * Computes series of constraints
      * @param goal_constraint: goal constraint
      * @param constraints: vector containing pointers to goal constraints
      */
    virtual bool ComputeConstraints(const amigo_whole_body_controller::ArmTaskGoal& goal_constraint, std::vector<amigo_whole_body_controller::ArmTaskGoal>& constraints)=0;

protected:

    /**
      * Whole body controller object for forward simulation
      */
    WholeBodyController* wbc_;

};

#endif
