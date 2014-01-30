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
      * @param constraints: vector containing constraints describing the path
      */
    bool computeConstraints(const amigo_whole_body_controller::ArmTaskGoal& goal_constraint, std::vector<amigo_whole_body_controller::ArmTaskGoal>& constraints);
    bool reComputeConstraints(std::vector<amigo_whole_body_controller::ArmTaskGoal>& constraints);

    /**
      *
      */
    void setStartPose(KDL::Frame startPose);

    void setBasePose(KDL::Frame base_pose);

    /**
      *
      */
    void interpolateConstraints(std::vector<amigo_whole_body_controller::ArmTaskGoal>& constraints);

    /**
      * Global Planner
      */
    TaskSpaceRoadmap* task_space_roadmap_;

protected:

    /** \brief A shared private node handle */
    ros::NodeHandle nh_private;

    void publishMarkers();

    void setOrientation(std::vector<amigo_whole_body_controller::ArmTaskGoal>& constraints);

    /**
      * RViz Visualization
      */
    PlanningVisualizer* visualizer_;

    /**
      *
      */
    KDL::Frame start_pose_;
    KDL::Frame base_pose_;

    amigo_whole_body_controller::ArmTaskGoal goal_constraint_;


    /**
      * Default constraint specification
      */
    amigo_whole_body_controller::ArmTaskGoal default_constraint_;

    /**
      * Intermediate constraint specification
      */
    amigo_whole_body_controller::ArmTaskGoal intermediate_constraint_;

    void loadConstraint(XmlRpc::XmlRpcValue, amigo_whole_body_controller::ArmTaskGoal &constraint);

    /**
      * Assigns cartesian impedance stiffnesses and constraint region parameters
      * @param goal: goal definition
      */
    void assignImpedance(std::vector<amigo_whole_body_controller::ArmTaskGoal>& constraints_);

};

#endif
