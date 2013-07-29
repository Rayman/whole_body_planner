/*!
 * \author Teun Derksen
 * \date July 2013
 * \version 0.1
 */

#ifndef PLANNERGLOBAL_H_
#define PLANNERGLOBAL_H_

#include "whole_body_planner/Planner.h"
#include "whole_body_planner/TaskSpaceRoadmap.h"
#include "whole_body_planner/PlanningVisualizer.h"

class PlannerGlobal : public Planner
{
public:

    /**
      * Constructor
      */
    PlannerGlobal();

    /**
      * Deconstructor
      */
    ~PlannerGlobal();

    /**
      * Computes series of constraints
      * @param goal_constraint: goal constraint
      * @param constraints: vector containing pointers to goal constraints
      */
    bool computeConstraints(const amigo_whole_body_controller::ArmTaskGoal& goal_constraint, std::vector<amigo_whole_body_controller::ArmTaskGoal>& constraints);

protected:

    /**
      * Global Planner
      */
    TaskSpaceRoadmap* task_space_roadmap_;

    /**
      * RViz Visualization
      */
    PlanningVisualizer* visualizer_;

};

#endif
