/*!
 * \author Janno Lunenburg
 * \date June 2013
 * \version 0.1
 */

#ifndef PLANNERTOPOLOGICAL_H_
#define PLANNERTOPOLOGICAL_H_

#include "whole_body_planner/Planner.h"
#include "whole_body_planner/ConnectivityGraph.h"

class PlannerTopological : public Planner
{
public:

    /**
      * Constructor
      */
    PlannerTopological();

    /**
      * Deconstructor
      */
    ~PlannerTopological();

    /**
      * Computes series of constraints
      * @param goal_constraint: goal constraint
      * @param constraints: vector containing pointers to goal constraints
      */
    bool computeConstraints(const amigo_whole_body_controller::ArmTaskGoal& goal_constraint, std::vector<amigo_whole_body_controller::ArmTaskGoal>& constraints);

protected:

    /**
      * Connectivitygraph with possible controlpoints
      */
    Graph connectivity_graph_;

    /**
      * Contains the current state of the planner
      */
    std::string current_state_;

};

#endif
