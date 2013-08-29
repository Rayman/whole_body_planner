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
    /// Get Start Position and convert from KDL to PoseStamped message
    std::cout<<"Start: x= "<<start_pose_.p.x()<<" y= "<<start_pose_.p.y()<<" z= "<< start_pose_.p.z()<<"\n"<<std::cout;
    geometry_msgs::PoseStamped msg_start_pose;
    msg_start_pose.header.frame_id = "/map";
    msg_start_pose.pose.position.x = start_pose_.p.x();
    msg_start_pose.pose.position.y = start_pose_.p.y();
    msg_start_pose.pose.position.z = start_pose_.p.z();
    start_pose_.M.GetQuaternion(msg_start_pose.pose.orientation.x,
                                   msg_start_pose.pose.orientation.y,
                                   msg_start_pose.pose.orientation.z,
                                   msg_start_pose.pose.orientation.w);

    /// Create roadmap and find global path
    if(!task_space_roadmap_->plan(goal_constraint,msg_start_pose))
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
    ROS_INFO("Constraints: size = %i",(int)constraints.size());

    return true;
}
