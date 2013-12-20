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
	std::cout<<goal_constraint.goal_type<<std::endl;
	if (goal_constraint.goal_type.compare("grasp")==0){
        ROS_INFO("Received grasp goal, removing object pose from STATIC octomap, this should be generalized!");
        //removeOctomapBBX(goal_constraint.position_constraint.position, goal_constraint.position_constraint.header.frame_id);
    }
	
	
    /// Create roadmap and find global path
    if(!task_space_roadmap_->plan(goal_constraint, start_pose_, base_pose_)){
        ROS_INFO("NO PLAN!");
        return false;
    }

    /// Visualize in RViz
    publishMarkers();

    /// Get the path
    //constraints = task_space_roadmap_->getPlan();
    constraints = task_space_roadmap_->convertSolutionToArmTaskGoal();

    /// Try to shortcut
    constraints = task_space_roadmap_->shortCutPlan();

    ROS_INFO("Constraints: size = %i",(int)constraints.size());

    /// At this point only 3D constraints are computed by PlannerGlobal, give goal orientation to the rest of constraints
    setOrientation(goal_constraint, constraints);
    return true;
}

bool PlannerGlobal::reComputeConstraints(const amigo_whole_body_controller::ArmTaskGoal& goal_constraint, std::vector<amigo_whole_body_controller::ArmTaskGoal>& constraints)
{
    /// Create roadmap and find global path
    if(!task_space_roadmap_->replan(start_pose_))
    {
        ROS_INFO("REPLANNING FAILED!");
        return false;
    }

    /// Visualize in RViz
    publishMarkers();

    /// Get the path
    constraints = task_space_roadmap_->convertSolutionToArmTaskGoal();

    /// Simplify the path
    constraints = task_space_roadmap_->simplifyPlan();

    ROS_INFO("Constraints: size = %i",(int)constraints.size());


    /// At this point only 3D constraints are computed by PlannerGlobal, give goal orientation to the rest of constraints
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
    //std::vector<std::vector<double> > coordinates = task_space_roadmap_->convertSolutionToVector();
    std::vector<std::vector<double> > coordinates = task_space_roadmap_->shortCutPlanToVector();
    visualizer_->displaySamples(task_space_roadmap_->getPlanData());
    ros::Duration(0.1).sleep();
    visualizer_->displayGraph(task_space_roadmap_->getPlanData());
    ros::Duration(0.1).sleep();
    visualizer_->displayPath(coordinates,1);

    /*
    /// Shortcut

    task_space_roadmap_->shortCutPlanToVector();
    coordinates = task_space_roadmap_->convertSolutionToVector();
    visualizer_->displayPath(coordinates,2);

    /// Smooth
    task_space_roadmap_->smoothPlanToVector();
    coordinates = task_space_roadmap_->convertSolutionToVector();
    visualizer_->displayPath(coordinates,3);

    */

}

void PlannerGlobal::setOrientation(const amigo_whole_body_controller::ArmTaskGoal& goal_constraint, std::vector<amigo_whole_body_controller::ArmTaskGoal>& constraints)
{
    for( int constraint_id = 0; constraint_id  < int ( constraints.size() ); ++constraint_id )
    {
         constraints[constraint_id].orientation_constraint.orientation = goal_constraint.orientation_constraint.orientation;
    }
}

/*
void PlannerGlobal::removeOctomapBBX(const geometry_msgs::Point& goal, const std::string& root){

    if (task_space_roadmap_->octomap_){
        if (task_space_roadmap_->octomap_->size() > 0){

            KDL::Frame Frame_map_root = base_pose_;
            KDL::Frame Frame_root_goal;
            Frame_root_goal.p.x(goal.x);
            Frame_root_goal.p.y(goal.y);
            Frame_root_goal.p.z(goal.z);

            // Convert goal pose to map
            KDL::Frame Frame_map_goal = Frame_map_root*Frame_root_goal;
            ROS_INFO("Collision Avoidance: Bounding box removed from octomap in /map (%f,%f,%f)",Frame_map_goal.p.x(),Frame_map_goal.p.y(),Frame_map_goal.p.z());

            // Set up bounding box dimension
            geometry_msgs::Point bbx_min;
            geometry_msgs::Point bbx_max;

            bbx_max.x = Frame_map_goal.p.x() + 0.05;
            bbx_max.y = Frame_map_goal.p.y() + 0.051;
            bbx_max.z = Frame_map_goal.p.z() + 0.05;
            bbx_min.x = Frame_map_goal.p.x() - 0.05;
            bbx_min.y = Frame_map_goal.p.y() - 0.051;
            bbx_min.z = Frame_map_goal.p.z() - 0.015;

            ROS_INFO("Bounding box max: (%f %f %f), min (%f %f %f)",bbx_max.x,bbx_max.y,bbx_max.z,bbx_min.x, bbx_min.y, bbx_min.z );
            octomath::Vector3 min = octomap::pointMsgToOctomap(bbx_min);
            octomath::Vector3 max = octomap::pointMsgToOctomap(bbx_max);

            for(octomap::OcTreeStamped::leaf_bbx_iterator it = task_space_roadmap_->octomap_->begin_leafs_bbx(min,max),
                end=task_space_roadmap_->octomap_->end_leafs_bbx(); it!= end; ++it){

                it->setLogOdds(octomap::logodds(0.0));
            }
            task_space_roadmap_->octomap_->updateInnerOccupancy();
        }
    }
}
*/
