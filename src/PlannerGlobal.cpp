#include "whole_body_planner/PlannerGlobal.h"
PlannerGlobal::PlannerGlobal()
{
    // Setup planning object
    task_space_roadmap_ = new TaskSpaceRoadmap();

    // Setup visualization object
    visualizer_ = new PlanningVisualizer();
}

PlannerGlobal::~PlannerGlobal()
{
    delete task_space_roadmap_;
    delete visualizer_;
}

void PlannerGlobal::setStartPose(KDL::Frame start_pose)
{
    start_pose_ = start_pose;
}

void PlannerGlobal::setBasePose(KDL::Frame base_pose)
{
    base_pose_ = base_pose;
}

bool PlannerGlobal::computeConstraints(const amigo_whole_body_controller::ArmTaskGoal& goal_constraint, std::vector<amigo_whole_body_controller::ArmTaskGoal>& constraints)
{
    /// Create roadmap and find global path
    if(!task_space_roadmap_->plan(goal_constraint, start_pose_, base_pose_)){
        ROS_INFO("NO PLAN!");
        return false;
    }

    /// Visualize in RViz
    publishMarkers();

    /// Get the path
    constraints = task_space_roadmap_->getPlan();
    //constraints = task_space_roadmap_->simplifyPlan();

    ROS_INFO("Constraints: size = %i",(int)constraints.size());

    /// At this point only 3D constraints are computed by PlannerGlobal, give goal orientation to the rest of constraints
    setOrientation(goal_constraint, constraints);
    return true;
}

bool PlannerGlobal::reComputeConstraints(const amigo_whole_body_controller::ArmTaskGoal& goal_constraint, std::vector<amigo_whole_body_controller::ArmTaskGoal>& constraints)
{
    /// Create roadmap and find global path
    std::cout<<start_pose_.p.x()<<std::endl;
    if(!task_space_roadmap_->replan(start_pose_))
    {
        ROS_INFO("REPLANNING FAILED!");
        return false;
    }

    ros::Duration(2.0).sleep();

    /// Visualize in RViz
    publishMarkers();

    /// Get the path
    constraints = task_space_roadmap_->getPlan();
    //constraints = task_space_roadmap_->simplifyPlan();

    ROS_INFO("Constraints: size = %i",(int)constraints.size());

    setOrientation(goal_constraint, constraints);
    return true;
}

void PlannerGlobal::interpolateConstraints(std::vector<amigo_whole_body_controller::ArmTaskGoal>& constraints)
{
    constraints = task_space_roadmap_->interpolatePlan();
}

void PlannerGlobal::publishMarkers()
{
    /// Visualize roadmap and plan
    std::vector<std::vector<double> > coordinates = task_space_roadmap_->convertSolutionToVector();
    visualizer_->displaySamples(task_space_roadmap_->getPlanData());
    visualizer_->displayGraph(task_space_roadmap_->getPlanData());
    visualizer_->displayPath(coordinates,1);

}

void PlannerGlobal::setOrientation(const amigo_whole_body_controller::ArmTaskGoal& goal_constraint, std::vector<amigo_whole_body_controller::ArmTaskGoal>& constraints)
{
    for( int constraint_id = 0; constraint_id  < int ( constraints.size() ); ++constraint_id )
    {
        /// Starting pose
        if (constraint_id  == 0){
            start_pose_.M.GetQuaternion(constraints[constraint_id].orientation_constraint.orientation.x,
                                        constraints[constraint_id].orientation_constraint.orientation.y,
                                        constraints[constraint_id].orientation_constraint.orientation.z,
                                        constraints[constraint_id].orientation_constraint.orientation.w);
        }
        /// Constraints
        else{
            constraints[constraint_id].orientation_constraint.orientation = goal_constraint.orientation_constraint.orientation;
        }
    }
}

