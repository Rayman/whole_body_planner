#include "whole_body_planner/PlannerGlobal.h"

#include <octomap_ros/conversions.h>

PlannerGlobal::PlannerGlobal()
{
    /// Setup planning object
    task_space_roadmap_ = new TaskSpaceRoadmap();

    /// Setup visualization object
    visualizer_ = new PlanningVisualizer();

    /// Load intermediate (path) and default goal constraint
    XmlRpc::XmlRpcValue default_constraint, intermediate_constraint;
    nh_private.getParam("/whole_body_planner/default_goal_constraint", default_constraint);
    nh_private.getParam("/whole_body_planner/intermediate_path_constraint", intermediate_constraint);

    loadConstraint(default_constraint, default_constraint_);
    loadConstraint(intermediate_constraint, intermediate_constraint_);

}

PlannerGlobal::~PlannerGlobal()
{
    delete task_space_roadmap_;
    delete visualizer_;
}



bool PlannerGlobal::computeConstraints(const amigo_whole_body_controller::ArmTaskGoal& goal_constraint, std::vector<amigo_whole_body_controller::ArmTaskGoal>& constraints)
{

    goal_constraint_ = goal_constraint;

	if (goal_constraint.goal_type.compare("grasp")==0){
        ROS_INFO("Received grasp goal, removing object pose from STATIC octomap, this should be generalized!");
        removeOctomapBBX(goal_constraint.position_constraint.position, goal_constraint.position_constraint.header.frame_id);
    }
		
    /// Create roadmap and find global path
    if(!task_space_roadmap_->plan(goal_constraint_, start_pose_, base_pose_)){

        ROS_INFO("Global planning could not find a path!");
        return false;
    }

    /// Visualize in RViz
    publishMarkers();

    /// Get the path
    constraints = task_space_roadmap_->convertSolutionToArmTaskGoal();

    /// Try to shortcut
    constraints = task_space_roadmap_->simplifyPlan();
    //constraints = task_space_roadmap_->smoothPlan();
    ROS_INFO("Constraints: size = %i",(int)constraints.size());

    /// At this point only 3D constraints are computed by PlannerGlobal, give goal orientation to the rest of constraints
    setOrientation(constraints);
    assignImpedance(constraints);

    return true;
}

bool PlannerGlobal::reComputeConstraints(std::vector<amigo_whole_body_controller::ArmTaskGoal>& constraints)
{
    /// Create roadmap and find global path
    if(!task_space_roadmap_->replan(start_pose_))
    {
        ROS_INFO("Global planning could not find a path!");
        return false;
    }

    /// Visualize in RViz
    publishMarkers();

    /// Get the path
    constraints = task_space_roadmap_->convertSolutionToArmTaskGoal();

    /// Simplify the path
    constraints = task_space_roadmap_->shortCutPlan();

    ROS_INFO("Constraints: size = %i",(int)constraints.size());


    /// At this point only 3D constraints are computed by PlannerGlobal, give goal orientation to the rest of constraints
    setOrientation(constraints);
    assignImpedance(constraints);
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
    std::vector<std::vector<double> > coordinates = task_space_roadmap_->convertSolutionToVector();
    //visualizer_->displaySamples(task_space_roadmap_->getPlanData());
    //visualizer_->displayGraph(task_space_roadmap_->getPlanData());
    visualizer_->displayPath(coordinates,1);

    /// Shortcut
    task_space_roadmap_->shortCutPlanToVector();
    coordinates = task_space_roadmap_->convertSolutionToVector();
    visualizer_->displayPath(coordinates,2);

    /// Smooth
    task_space_roadmap_->smoothPlanToVector();
    coordinates = task_space_roadmap_->convertSolutionToVector();
    visualizer_->displayPath(coordinates,3);

}

void PlannerGlobal::setOrientation( std::vector<amigo_whole_body_controller::ArmTaskGoal>& constraints)
{
    for( int constraint_id = 0; constraint_id  < int ( constraints.size() ); ++constraint_id )
    {
         constraints[constraint_id].orientation_constraint.orientation = goal_constraint_.orientation_constraint.orientation;
    }
}

void PlannerGlobal::loadConstraint(XmlRpc::XmlRpcValue param_constraint, amigo_whole_body_controller::ArmTaskGoal &constraint){

    XmlRpc::XmlRpcValue position_stiffness = param_constraint["position_stiffness"];
    XmlRpc::XmlRpcValue position_dimension = param_constraint["position_dimension"];
    XmlRpc::XmlRpcValue orientation_stiffness = param_constraint["orientation_stiffness"];
    XmlRpc::XmlRpcValue orientation_tolerance = param_constraint["orientation_tolerance"];

    if (position_stiffness.hasMember("x")){
        constraint.stiffness.force.x = position_stiffness["x"];
    }
    if (position_stiffness.hasMember("y")){
        constraint.stiffness.force.y = position_stiffness["y"];
    }
    if (position_stiffness.hasMember("z")){
        constraint.stiffness.force.z = position_stiffness["z"];
    }
    if (orientation_stiffness.hasMember("r")){
        constraint.stiffness.torque.x = orientation_stiffness["r"];
    }
    if (orientation_stiffness.hasMember("p")){
        constraint.stiffness.torque.y = orientation_stiffness["p"];
    }
    if (orientation_stiffness.hasMember("y")){
        constraint.stiffness.torque.z = orientation_stiffness["y"];
    }
    if (orientation_tolerance.hasMember("r")){
        constraint.orientation_constraint.absolute_roll_tolerance = orientation_tolerance["r"];
    }
    if (orientation_tolerance.hasMember("p")){
        constraint.orientation_constraint.absolute_pitch_tolerance = orientation_tolerance["p"];
    }
    if (orientation_tolerance.hasMember("y")){
        constraint.orientation_constraint.absolute_yaw_tolerance  = orientation_tolerance["y"];
    }
    constraint.position_constraint.constraint_region_shape.dimensions.push_back(position_dimension);
}

void PlannerGlobal::assignImpedance( std::vector<amigo_whole_body_controller::ArmTaskGoal>& constraints_)
{
    for (unsigned int i = 0; i < constraints_.size(); i++)
    {
        if (i != constraints_.size()-1)
        {
            constraints_[i].position_constraint.constraint_region_shape.type = intermediate_constraint_.position_constraint.constraint_region_shape.SPHERE;
            constraints_[i].position_constraint.constraint_region_shape.dimensions.push_back(intermediate_constraint_.position_constraint.constraint_region_shape.dimensions[0]);
            constraints_[i].orientation_constraint = intermediate_constraint_.orientation_constraint;
            constraints_[i].stiffness = intermediate_constraint_.stiffness;
            //constraints_[i].goal_type = 'reset';
        }

        /// Final constraint!
        else {
            // Assign Stiffness
            constraints_[i].stiffness = goal_constraint_.stiffness;

            // Position constraints
            if (!goal_constraint_.position_constraint.constraint_region_shape.dimensions.empty())
            {
                constraints_[i].position_constraint.constraint_region_shape = goal_constraint_.position_constraint.constraint_region_shape;
            }
            else {
                constraints_[i].position_constraint.constraint_region_shape.type = constraints_[i].position_constraint.constraint_region_shape.SPHERE;
                constraints_[i].position_constraint.constraint_region_shape.dimensions.push_back(default_constraint_.position_constraint.constraint_region_shape.dimensions[0]);
            }

            // Orientation constraints
            if (goal_constraint_.orientation_constraint.absolute_roll_tolerance == 0)
                constraints_[i].orientation_constraint.absolute_roll_tolerance = default_constraint_.orientation_constraint.absolute_roll_tolerance;
            else
                constraints_[i].orientation_constraint.absolute_roll_tolerance = goal_constraint_.orientation_constraint.absolute_roll_tolerance;

            if (goal_constraint_.orientation_constraint.absolute_pitch_tolerance == 0)
                constraints_[i].orientation_constraint.absolute_pitch_tolerance = default_constraint_.orientation_constraint.absolute_pitch_tolerance;
            else
                constraints_[i].orientation_constraint.absolute_pitch_tolerance = goal_constraint_.orientation_constraint.absolute_pitch_tolerance;

            if (goal_constraint_.orientation_constraint.absolute_yaw_tolerance == 0)
                constraints_[i].orientation_constraint.absolute_yaw_tolerance = default_constraint_.orientation_constraint.absolute_yaw_tolerance;
            else
                constraints_[i].orientation_constraint.absolute_yaw_tolerance = goal_constraint_.orientation_constraint.absolute_yaw_tolerance;
        }
    }


}

void PlannerGlobal::setStartPose(KDL::Frame start_pose)
{
    start_pose_ = start_pose;
}

void PlannerGlobal::setBasePose(KDL::Frame base_pose)
{
    base_pose_ = base_pose;
}


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
                   

            bbx_max.x = goal.x + 0.051;
            bbx_max.y = goal.y + 0.051;
            bbx_max.z = goal.z + 0.051;
            bbx_min.x = goal.x - 0.051;
            bbx_min.y = goal.y - 0.051;
            bbx_min.z = goal.z - 0.05;

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

