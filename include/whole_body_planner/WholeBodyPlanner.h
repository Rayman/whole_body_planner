/*!
 * \author Janno Lunenburg
 * \date June 2013
 * \version 0.1
 */

#ifndef WHOLEBODYPLANNER_H_
#define WHOLEBODYPLANNER_H_

#include <amigo_whole_body_controller/ArmTaskAction.h>
#include <actionlib/server/simple_action_server.h>
#include <amigo_arm_navigation/grasp_precomputeAction.h>

/// Messages
#include <nav_msgs/Path.h>

/// Planners
#include "PlannerEmpty.h"
#include "PlannerTopological.h"
#include "PlannerGlobal.h"

/// Auxiliary stuff
#include "Executer.h"
#include "RobotStateInterface.h"
#include "Simulator.h"

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
    std::vector<amigo_whole_body_controller::ArmTaskGoal> constraints_;

    /**
      * Object to simulate constraints to check if a plan is feasible
      */
    Simulator simulator_;

    /**
      * Object to send constraints to whole body controller
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

    /**
      * Calls planners, simulator and executer
      * @param goal: goal definition
      */
    bool planSimExecute(const amigo_whole_body_controller::ArmTaskGoal &goal);

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
      * Global planner
      */
    PlannerGlobal planner_global_;

    /**
      * Interface to robot hardware
      * Receives joint positions
      */
    RobotStateInterface* robot_state_interface_;

    /// Marker + trajectory publisher
    /**
      * Marker Publisher
      */
    ros::Publisher marker_array_pub_ ;

    /**
      * Trajectory publisher
      */
    ros::Publisher trajectory_pub_;

    /**
      * Function publishes constraints as marker array
      */
    void PublishMarkers();

    /**
      * Assigns cartesian impedance stiffnesses and constraint region parameters
      * @param goal: goal definition
      */
    void assignImpedance(const amigo_whole_body_controller::ArmTaskGoal &goal);

    /**
      * Functions publishes end-effector trajectory
      */
    void PublishTrajectory(nav_msgs::Path& trajectory);

    /// Required for old interface
    /**
      * Action server for old interface (left)
      */
    actionlib::SimpleActionServer<amigo_arm_navigation::grasp_precomputeAction>* action_server_old_left_;

    /**
      * Callback function for old interface (left)
      */
    void goalCBOldLeft();

    /**
      * Action server for old interface (right)
      */
    actionlib::SimpleActionServer<amigo_arm_navigation::grasp_precomputeAction>* action_server_old_right_;

    /**
      * Callback function for old interface (left)
      */
    void goalCBOldRight();

    /**
      * Converts grasp-precompute goal to motion constraint
      */
    bool convertGoalType(const amigo_arm_navigation::grasp_precomputeGoal &grasp_goal, amigo_whole_body_controller::ArmTaskGoal &goal);

    /**
      * tf listener (required for delta goals)
      */
    tf::TransformListener listener_;

    /**
      * Maximum iterations for joint-space validation
      */
    int max_iterations_;

    /**
      * Default constraint specification
      */
    amigo_whole_body_controller::ArmTaskGoal default_constraint_;

    /**
      * Intermediate constraint specification
      */
    amigo_whole_body_controller::ArmTaskGoal intermediate_constraint_;

    void loadConstraint(XmlRpc::XmlRpcValue, amigo_whole_body_controller::ArmTaskGoal &constraint);

};

#endif
