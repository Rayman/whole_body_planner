/*!
 * \author Janno Lunenburg
 * \date June 2013
 * \version 0.1
 */

#ifndef WHOLEBODYPLANNER_H_
#define WHOLEBODYPLANNER_H_

#include <amigo_whole_body_controller/ArmTaskAction.h>
#include <actionlib/server/simple_action_server.h>
#include <tue_manipulation/GraspPrecomputeAction.h>

/// Messages
#include <nav_msgs/Path.h>

/// Planners
#include "PlannerEmpty.h"
#include "PlannerTopological.h"
#include "PlannerGlobal.h"

/// Auxiliary stuff
#include "Executer2.h"
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

    /** \brief Callback function for octomap */
#if ROS_VERSION_MINIMUM(1,9,0)
    // Groovy
    void octoMapCallback(const octomap_msgs::Octomap::ConstPtr& msg);
#elif ROS_VERSION_MINIMUM(1,8,0)
    // Fuerte
    void octoMapCallback(const octomap_msgs::OctomapBinary::ConstPtr& msg);
#endif

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
    Executer2 executer_;

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
      * Functions publishes end-effector trajectory
      */
    void PublishTrajectory(nav_msgs::Path& trajectory);

    /// Required for old interface
    /**
      * Action server for old interface (left)
      */
    actionlib::SimpleActionServer<tue_manipulation::GraspPrecomputeAction>* action_server_old_left_;

    /**
      * Callback function for old interface (left)
      */
    void goalCBOldLeft();

    /**
      * Action server for old interface (right)
      */
    actionlib::SimpleActionServer<tue_manipulation::GraspPrecomputeAction>* action_server_old_right_;

    /**
      * Callback function for old interface (left)
      */
    void goalCBOldRight();

    /**
      * Converts grasp-precompute goal to motion constraint
      */
    bool convertGoalType(const tue_manipulation::GraspPrecomputeGoal &grasp_goal, amigo_whole_body_controller::ArmTaskGoal &goal);

    /**
      * tf listener (required for delta goals)
      */
    tf::TransformListener listener_;

    /**
      * Maximum iterations for joint-space validation
      */
    int max_iterations_;

    ros::Subscriber octomap_sub;



};

#endif
