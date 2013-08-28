#include "whole_body_planner/WholeBodyPlanner.h"

WholeBodyPlanner::WholeBodyPlanner()
{

    ROS_INFO("Initializing whole body planner");
    ros::NodeHandle nh_private("~");

    /// Action servers
    action_server_ = new actionlib::SimpleActionServer<amigo_whole_body_controller::ArmTaskAction>(nh_private, "motion_constraint", false);
    action_server_->registerGoalCallback(boost::bind(&WholeBodyPlanner::goalCB, this));
    action_server_->start();

    action_server_old_left_ = new actionlib::SimpleActionServer<amigo_arm_navigation::grasp_precomputeAction>(nh_private, "/grasp_precompute_left", false);
    action_server_old_left_->registerGoalCallback(boost::bind(&WholeBodyPlanner::goalCBOldLeft, this));
    action_server_old_left_->start();

    action_server_old_right_ = new actionlib::SimpleActionServer<amigo_arm_navigation::grasp_precomputeAction>(nh_private, "/grasp_precompute_right", false);
    action_server_old_right_->registerGoalCallback(boost::bind(&WholeBodyPlanner::goalCBOldRight, this));
    action_server_old_right_->start();

    /// Publishers
    marker_array_pub_ = nh_private.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1);
    trajectory_pub_   = nh_private.advertise<nav_msgs::Path>("/whole_body_planner/trajectory", 1);

    int planner;
    nh_private.param("planner_type", planner, 0);
    planner_ = planner;
    ROS_INFO("Planner type = %i",planner_);

    // ToDo: don't hardcode
    simulator_.initialize(0.02);

}

WholeBodyPlanner::~WholeBodyPlanner()
{

    delete action_server_;
    action_server_ = NULL;
    delete action_server_old_left_;
    action_server_old_left_ = NULL;
    delete action_server_old_right_;
    action_server_old_right_ = NULL;

}

bool WholeBodyPlanner::planSimExecute(const amigo_whole_body_controller::ArmTaskGoal &goal)
{
    /// Get initial positions from robot interface
    std::map<std::string, double> joint_position_map = robot_state_interface_.getJointPositions();

    /// Compute constraints
    bool plan_result = false;
    if (planner_ == 0)
    {
        planner_empty_.setInitialJointPositions(joint_position_map);
        plan_result = planner_empty_.computeConstraints(goal, constraints_);
    }
    else if (planner_ == 1)
    {
        planner_topological_.setInitialJointPositions(joint_position_map);
        plan_result = planner_topological_.computeConstraints(goal, constraints_);
    }
    else if (planner_ == 2)
    {
        planner_global_.setInitialJointPositions(joint_position_map);
        plan_result = planner_global_.computeConstraints(goal, constraints_);
    }
    ROS_INFO("Computed plan, result = %d",plan_result);

    /// Check whether constraints are feasible in joint space as well
    bool plan_feasible = false;
    if (plan_result)
    {
        /// Publish markers
        PublishMarkers();
        /// Set initial state simulator (setInitialJointConfiguration)
        simulator_.setInitialJointConfiguration(robot_state_interface_.getJointPositions(), robot_state_interface_.getAmclPose());
        /// Check if plan is feasible (checkFeasibility)
        int error_index = 0;
        // ToDo: Don't hardcode max_iter
        plan_feasible = simulator_.checkFeasibility(constraints_, 100, error_index);
        ROS_INFO("Checked feasibility, error_index = %i", error_index);

        /// Publish computed trajectory
        nav_msgs::Path path = simulator_.getPath();
        PublishTrajectory(path);
    }

    /// If succeeded, send to whole body controller
    bool execute_result = false;
    if (plan_feasible)
    {
        //execute_result = executer_.Execute(constraints_);
        execute_result = true;
        ROS_WARN("Execution disabled!!!");
    }
    return execute_result;
}

void WholeBodyPlanner::goalCB()
{
    const amigo_whole_body_controller::ArmTaskGoal& goal = *action_server_->acceptNewGoal();

    /// Plan, simulate and execute
    bool result = planSimExecute(goal);

    /// If succeeded, set server succeeded
    if (result)
    {
        action_server_->setSucceeded();
    }
    else
    {
        action_server_->setAborted();
    }

}

void WholeBodyPlanner::goalCBOldLeft()
{
    ROS_WARN("This action will become deprecated, please convert to the new interface");

    /// Messages
    const amigo_arm_navigation::grasp_precomputeGoal& grasp_goal = *action_server_old_left_->acceptNewGoal();
    amigo_whole_body_controller::ArmTaskGoal goal;

    /// Convert goal
    convertGoalType(grasp_goal, goal);

    /// Set link names
    ROS_INFO("Setting link names");
    goal.position_constraint.link_name = "/grippoint_left";
    goal.orientation_constraint.link_name = "/grippoint_left";

    /// Plan, simulate and execute
    ROS_INFO("Plan, simulate, execute");
    bool result = planSimExecute(goal);

    /// If succeeded, set server succeeded
    if (result)
    {
        action_server_old_left_->setSucceeded();
    }
    else
    {
        action_server_old_left_->setAborted();
    }
}

void WholeBodyPlanner::goalCBOldRight()
{
    ROS_WARN("This action will become deprecated, please convert to the new interface");

    /// Messages
    const amigo_arm_navigation::grasp_precomputeGoal& grasp_goal = *action_server_old_right_->acceptNewGoal();
    amigo_whole_body_controller::ArmTaskGoal goal;

    /// Convert goal
    convertGoalType(grasp_goal, goal);

    /// Set link names
    goal.position_constraint.link_name = "/grippoint_right";
    goal.orientation_constraint.link_name = "/grippoint_right";
}

void WholeBodyPlanner::convertGoalType(const amigo_arm_navigation::grasp_precomputeGoal& grasp_goal, amigo_whole_body_controller::ArmTaskGoal &goal)
{
    /// Position constraint
    ROS_INFO("Position constraint: position");
    ROS_INFO("Position constraint: x: %f, y: %f, z: %f",grasp_goal.goal.x, grasp_goal.goal.y, grasp_goal.goal.z);
    goal.position_constraint.header = grasp_goal.goal.header;
    goal.position_constraint.position.x = grasp_goal.goal.x;
    goal.position_constraint.position.y = grasp_goal.goal.y;
    goal.position_constraint.position.z = grasp_goal.goal.z;

    /// Default: sphere with radius 2 cm
    ROS_INFO("Position constriant: constraint region shape");
    goal.position_constraint.constraint_region_shape.type = goal.position_constraint.constraint_region_shape.SPHERE;
    goal.position_constraint.constraint_region_shape.dimensions.push_back(0.02);

    /// Orientation constraint (static)
    ROS_INFO("Orientation constraint: roll: %f, pitch: %f, yaw: %f",grasp_goal.goal.roll, grasp_goal.goal.pitch, grasp_goal.goal.yaw);
    goal.orientation_constraint.orientation = tf::createQuaternionMsgFromRollPitchYaw(grasp_goal.goal.roll, grasp_goal.goal.pitch, grasp_goal.goal.yaw);
    ROS_INFO("Orientation constraint: tolerances");
    goal.orientation_constraint.absolute_roll_tolerance = 0.3;
    goal.orientation_constraint.absolute_pitch_tolerance = 0.3;
    goal.orientation_constraint.absolute_yaw_tolerance = 0.3;

    /// Stiffness
    ROS_INFO("Stiffness");
    goal.stiffness.force.x = 100;
    goal.stiffness.force.y = 100;
    goal.stiffness.force.z = 100;
    goal.stiffness.torque.x = 50;
    goal.stiffness.torque.y = 50;
    goal.stiffness.torque.z = 50;

    // ToDo: 'sample' yaw
    // ToDo: perform pre_grasp
    // ToDo: delta goal
    // ToDo: first joint pos only

}

void WholeBodyPlanner::PublishMarkers()
{
    visualization_msgs::MarkerArray marker_array;
    for (unsigned int i = 0; i < constraints_.size(); i++)
    {
        visualization_msgs::Marker marker;
        marker.header = constraints_[i].position_constraint.header;
        marker.id     = i;
        marker.type   = 0; // Is arrow, to illustrate orientation as well
        marker.pose.position    = constraints_[i].position_constraint.position;
        marker.pose.orientation = constraints_[i].orientation_constraint.orientation;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.r = 1-(i+1)/constraints_.size();
        marker.color.g = (i+1)/constraints_.size();
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration(5.0);
        marker_array.markers.push_back(marker);
        marker_array_pub_.publish(marker_array);
    }
}

void WholeBodyPlanner::PublishTrajectory(nav_msgs::Path &trajectory)
{
    // ToDo: fill trajectory and publish result
    trajectory_pub_.publish(trajectory);
}
