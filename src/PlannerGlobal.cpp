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
    delete wbc_;
    delete task_space_roadmap_;
    delete visualizer_;
}

bool PlannerGlobal::computeConstraints(const amigo_whole_body_controller::ArmTaskGoal& goal_constraint, std::vector<amigo_whole_body_controller::ArmTaskGoal>& constraints)
{
    // Get Start Position
    //std::map<std::string, geometry_msgs::PoseStamped>::iterator itrFK = wbc_->robot_state_.fk_poses_.find(goal_constraint.position_constraint.link_name);
    //std::cout<<wbc_->robot_state_.fk_poses_.size()<<std::endl;
    //geometry_msgs::PoseStamped end_effector_pose_ = itrFK->second;

    /// Start the planning
    if(!task_space_roadmap_->plan(goal_constraint))
    {
        ROS_INFO("NO PLAN!");
    }

    // Visualize plan
    std::vector<std::vector<double> > coordinates = task_space_roadmap_->convertSolutionToVector();
    visualizer_->displaySamples(task_space_roadmap_->getPlanData());
    visualizer_->displayGraph(task_space_roadmap_->getPlanData());
    visualizer_->displayPath(coordinates,1);

    /// Get the path
    constraints = task_space_roadmap_->getPlan();
    ROS_INFO("Constraints: size = %i",constraints.size());

    return true;
}
