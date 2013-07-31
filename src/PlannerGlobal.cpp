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
    /// Get Start Position
    geometry_msgs::PoseStamped& start_pose_ =  wbc_->robot_state_.fk_poses_.find(goal_constraint.position_constraint.link_name)->second;

    /// Create roadmap and find global path
    if(!task_space_roadmap_->plan(goal_constraint,start_pose_))
    {
        ROS_INFO("NO PLAN!");
    }

    /// Visualize roadmap and plan
    std::vector<std::vector<double> > coordinates = task_space_roadmap_->convertSolutionToVector();
    visualizer_->displaySamples(task_space_roadmap_->getPlanData());
    visualizer_->displayGraph(task_space_roadmap_->getPlanData());
    visualizer_->displayPath(coordinates,1);

    /// Get the path
    constraints = task_space_roadmap_->getPlan();
    ROS_INFO("Constraints: size = %i",constraints.size());

    return true;
}
