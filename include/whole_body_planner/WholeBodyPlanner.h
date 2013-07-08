/*!
 * \author Janno Lunenburg
 * \date June 2013
 * \version 0.1
 */

#ifndef WHOLEBODYPLANNER_H_
#define WHOLEBODYPLANNER_H_

#include <amigo_whole_body_controller/ArmTaskAction.h>
#include <actionlib/server/simple_action_server.h>

/// Planners
#include "PlannerEmpty.h"
#include "PlannerTopological.h"

#include "Executer.h"
#include "RobotStateInterface.h"

class WholeBodyPlanner
{

public:

    /**
      * Constructor
      */
    WholeBodyPlanner();

    /**
      * Deconstructor
      */
    virtual ~WholeBodyPlanner();

protected:

    /**
      * Vector containing the list of constraints that is computed by the planner and is sent to the robot
      */
    //std::vector<amigo_whole_body_controller::ArmTaskGoal*> constraints_;
    std::vector<amigo_whole_body_controller::ArmTaskGoal> constraints_;

    /**
      * Class to send constraints to whole body controller
      */
    Executer executer_;

    /// Receiving goals
    /**
      * Action server
      */
    actionlib::SimpleActionServer<amigo_whole_body_controller::ArmTaskAction>* action_server_;

    /**
      * Callback function
      */
    void goalCB();

    /// Planners
    /**
      * Desired planner
      * Currently defaults to 0, meaning the goal is simply forwarded to the controller
      */
    unsigned int planner_;

    /**
      * Empty planner
      */
    PlannerEmpty planner_empty_;

    /**
      * Topological planner
      */
    PlannerTopological planner_topological_;

    /**
      * Interface to robot hardware
      * Receives joint positions
      */
    RobotStateInterface robot_state_interface_;

    /// Marker publisher
    /**
      * Publisher
      */
    ros::Publisher marker_array_pub_ ;

    /**
      * Function publishes constraints as marker array
      */
    void PublishMarkers();

};

#endif
