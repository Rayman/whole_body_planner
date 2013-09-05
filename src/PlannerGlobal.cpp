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

bool PlannerGlobal::computeConstraints(const amigo_whole_body_controller::ArmTaskGoal& goal_constraint, std::vector<amigo_whole_body_controller::ArmTaskGoal>& constraints)
{
    /// Get Start Position
    //std::cout<<"Start: x= "<<start_pose_.p.x()<<" y= "<<start_pose_.p.y()<<" z= "<< start_pose_.p.z()<<"\n"<<std::cout;

    /// Create roadmap and find global path
    if(!task_space_roadmap_->plan(goal_constraint, start_pose_)){
        ROS_INFO("NO PLAN!");
    }

    /// Visualize roadmap and plan
    std::vector<std::vector<double> > coordinates = task_space_roadmap_->convertSolutionToVector();
    visualizer_->displaySamples(task_space_roadmap_->getPlanData());
    visualizer_->displayGraph(task_space_roadmap_->getPlanData());
    visualizer_->displayPath(coordinates,1);

    /// Get the path
    constraints = task_space_roadmap_->getPlan();
    ROS_INFO("Constraints: size = %i",(int)constraints.size());

    /// At this point only 3D constraints are computed by PlannerGlobal
    for( int constraint_id = 0; constraint_id  < int ( constraints.size() ); ++constraint_id )
    {
        if (constraint_id  == 0)
        {
            start_pose_.M.GetQuaternion(constraints[constraint_id].orientation_constraint.orientation.x,
                                        constraints[constraint_id].orientation_constraint.orientation.y,
                                        constraints[constraint_id].orientation_constraint.orientation.z,
                                        constraints[constraint_id].orientation_constraint.orientation.w);
        }
        else{
            constraints[constraint_id].orientation_constraint.orientation = goal_constraint.orientation_constraint.orientation;

        }
    }

    return true;
}
