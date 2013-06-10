/*!
 * \author Janno Lunenburg
 * \date June 2013
 * \version 0.1
 */

#ifndef PLANNEREMPTY_H_
#define PLANNEREMPTY_H_

#include "whole_body_planner/Planner.h"

class PlannerEmpty : public Planner
{
public:

    /**
      * Constructor
      */
    PlannerEmpty();

    /**
      * Deconstructor
      */
    ~PlannerEmpty();

    /**
      * Computes series of constraints
      * @param goal_constraint: goal constraint
      * @param constraints: vector containing pointers to goal constraints
      */
    bool computeConstraints(const amigo_whole_body_controller::ArmTaskGoal& goal_constraint, std::vector<amigo_whole_body_controller::ArmTaskGoal>& constraints);

};

#endif
