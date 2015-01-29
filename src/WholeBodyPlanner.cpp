#include "whole_body_planner/WholeBodyPlanner.h"

#include <octomap_msgs/conversions.h>

WholeBodyPlanner::WholeBodyPlanner()
{

    ROS_INFO("Initializing whole body planner");
    ros::NodeHandle nh_private("~");

    /// Action servers
    // ToDo: move somewhere else (and keep this class clean)
    action_server_ = new actionlib::SimpleActionServer<amigo_whole_body_controller::ArmTaskAction>(nh_private, "motion_constraint", false);
    action_server_->registerGoalCallback(boost::bind(&WholeBodyPlanner::goalCB, this));
    action_server_->start();

    action_server_old_left_ = new actionlib::SimpleActionServer<tue_manipulation::GraspPrecomputeAction>(nh_private, "/grasp_precompute_left", false);
    action_server_old_left_->registerGoalCallback(boost::bind(&WholeBodyPlanner::goalCBOldLeft, this));
    action_server_old_left_->start();

    action_server_old_right_ = new actionlib::SimpleActionServer<tue_manipulation::GraspPrecomputeAction>(nh_private, "/grasp_precompute_right", false);
    action_server_old_right_->registerGoalCallback(boost::bind(&WholeBodyPlanner::goalCBOldRight, this));
    action_server_old_right_->start();

    /// Subscriber
    octomap_sub  = nh_private.subscribe<octomap_msgs::Octomap>("/octomap_binary", 10, &WholeBodyPlanner::octoMapCallback, this);

    /// Publishers
    marker_array_pub_ = nh_private.advertise<visualization_msgs::MarkerArray>("/whole_body_planner/constraints_out", 1);
    trajectory_pub_   = nh_private.advertise<nav_msgs::Path>("/whole_body_planner/trajectory", 1);

    /// Load parameters
    int planner;
    nh_private.param("planner_type", planner, 0);
    planner_ = planner;
    ROS_INFO("Planner type = %i",planner_);


    nh_private.param<int> ("/whole_body_planner/joint_space_feasibility/iterations", max_iterations_, 1000);

    // ToDo: don't hardcode
    simulator_.initialize(0.02);
    robot_state_interface_ = new RobotStateInterface();

    simulator_.setTransformListener(&listener_);
}

WholeBodyPlanner::~WholeBodyPlanner()
{
    delete robot_state_interface_;
    robot_state_interface_ = NULL;
    delete action_server_;
    action_server_ = NULL;
    delete action_server_old_left_;
    action_server_old_left_ = NULL;
    delete action_server_old_right_;
    action_server_old_right_ = NULL;

    octomap_sub.shutdown();
}

bool WholeBodyPlanner::planSimExecute(const amigo_whole_body_controller::ArmTaskGoal &goal)
{
    /// Set initial state simulator (setInitialJointConfiguration)
    robot_state_interface_->setAmclPose(); // Uses tf to get base pose
    simulator_.setInitialJointConfiguration(robot_state_interface_->getJointPositions(), robot_state_interface_->getAmclPose());

    /// Compute constraints
    bool plan_result = false;
    if (planner_ == 0)
    {
        //planner_empty_.setInitialJointPositions(joint_position_map);
        plan_result = planner_empty_.computeConstraints(goal, constraints_);
    }
    else if (planner_ == 1)
    {
        plan_result = planner_topological_.computeConstraints(goal, constraints_);
        ROS_INFO("Constraint 0 (%s) at %f,\t%f,\t%f", constraints_[0].goal_type.c_str(), constraints_[0].position_constraint.position.x, constraints_[0].position_constraint.position.y, constraints_[0].position_constraint.position.z);
    }
    else if (planner_ == 2)
    {

        /// Set the initial pose of the goal frame in map frame
        planner_global_.setStartPose(simulator_.getFramePose(goal.position_constraint.link_name));
        planner_global_.setBasePose(simulator_.getFramePose(goal.position_constraint.header.frame_id));

        /// Accordingly transform the goal pose of the goal frame from root to map frame
        KDL::Frame Frame_map_goal = simulator_.transformToMap(goal);

        /// Convert back to constraint-msg
        // Goal is const at this point, discuss with Janno
        amigo_whole_body_controller::ArmTaskGoal goal_map = goal;
        goal_map.position_constraint.header.frame_id = "map";
        goal_map.position_constraint.position.x = Frame_map_goal.p.x();
        goal_map.position_constraint.position.y = Frame_map_goal.p.y();
        goal_map.position_constraint.position.z = Frame_map_goal.p.z();
        Frame_map_goal.M.GetQuaternion(goal_map.orientation_constraint.orientation.x,
                               goal_map.orientation_constraint.orientation.y,
                               goal_map.orientation_constraint.orientation.z,
                               goal_map.orientation_constraint.orientation.w);

        plan_result = planner_global_.computeConstraints(goal_map, constraints_);

        /// Transform back to root frame
        simulator_.transformToRoot(constraints_, goal);
    }
    ROS_INFO("Computed plan, result = %d", plan_result);

    /// Check whether constraints are feasible in joint-space as well
    bool plan_feasible = false;
    if (plan_result)
    {
        /// Publish markers
        PublishMarkers();

        /// Check if plan is feasible (checkFeasibility)
        int error_index = 0;

        plan_feasible = simulator_.checkFeasibility(constraints_, max_iterations_, error_index);
        ROS_INFO("Checked feasibility, error_index = %i", error_index);

        /// Failure handling
        if(!plan_feasible){
            if (planner_ == 0)
            {
                ROS_WARN("Failure handling not implemented for plannerempty");
            }
            else if (planner_ == 1)
            {
                ROS_WARN("Failure handling not implemented for plannertopological");
            }
            else if (planner_ == 2)
            {
                constraints_.clear();
                ROS_WARN("plannerglobal: Replanning, with maximum clearance");
                bool plan_result = planner_global_.reComputeConstraints(constraints_);
                if (plan_result){

                    /// Reset the virtual WBC to the original starting position
                    simulator_.setInitialJointConfiguration(robot_state_interface_->getJointPositions(), robot_state_interface_->getAmclPose());
                    simulator_.transformToRoot(constraints_, goal);

                    plan_feasible = simulator_.checkFeasibility(constraints_, max_iterations_, error_index);

                }
            }
        }

        /// Publish computed trajectory
        nav_msgs::Path path = simulator_.getPath();
        PublishTrajectory(path);
    }

    /// If succeeded, send to whole body controller
    bool execute_result = false;

    if (plan_feasible)
    {
        execute_result = executer_.Execute(constraints_);
        if(!execute_result)
        {
            if (planner_ == 2){
                constraints_.clear();
                ROS_WARN("Replanning, resetting virtual WBC");

                /// Get the current robot pose
                delete robot_state_interface_;
                robot_state_interface_ = new RobotStateInterface();
                while (robot_state_interface_->getJointPositions().empty())
                {
                    ROS_WARN_ONCE("Waiting for new measurement..");
                    ros::spinOnce();
                }

                /// Start from the current robot pose
                simulator_.setInitialJointConfiguration(robot_state_interface_->getJointPositions(), robot_state_interface_->getAmclPose());
                planner_global_.setStartPose(simulator_.getFramePose(goal.position_constraint.link_name));
                bool plan_result = planner_global_.reComputeConstraints(constraints_);
                if (plan_result){
                    simulator_.transformToRoot(constraints_, goal);
                    int error_index;
                    plan_feasible = simulator_.checkFeasibility(constraints_, max_iterations_, error_index);
                    if(plan_feasible)
                    {
                         execute_result = executer_.Execute(constraints_);
                    }

                }
            }

        }
    }

    // ToDo: make more generic
    if (planner_ == 1) planner_topological_.setCurrentState(executer_.getCurrentState());

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
    simulator_.setInitialJointConfiguration(robot_state_interface_->getJointPositions(), robot_state_interface_->getAmclPose());

    /// Messages
    const tue_manipulation::GraspPrecomputeGoal& grasp_goal = *action_server_old_left_->acceptNewGoal();
    amigo_whole_body_controller::ArmTaskGoal goal;
    
	/// Set link names
    ROS_INFO("Setting link names");
    goal.position_constraint.link_name = "grippoint_left";
    goal.orientation_constraint.link_name = "grippoint_left";
    
    /// Convert goal
    if (!convertGoalType(grasp_goal, goal))
    {
        action_server_old_right_->setAborted();
        return;
    }
    
    //// HARDCODED FIX!!!
    if(goal.position_constraint.header.frame_id == "/amigo/base_link") goal.position_constraint.header.frame_id = "base_link";

    /// Plan, simulate and execute
    ROS_WARN("Plan, simulate, execute");
    bool result = planSimExecute(goal);

    /// If succeeded, pre-grasp required (meaning moving to an offset) and first_joint_pos_only is false:
    /// Perform grasp motion as well (but this time with target_point_offset.x zero)
    if (result && grasp_goal.PERFORM_PRE_GRASP && !grasp_goal.FIRST_JOINT_POS_ONLY)
    {
		goal.goal_type = "grasp";
        goal.position_constraint.target_point_offset.x = 0.0;
        result = planSimExecute(goal);
    }

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

    simulator_.setInitialJointConfiguration(robot_state_interface_->getJointPositions(), robot_state_interface_->getAmclPose());
	
	/// Messages
    const tue_manipulation::GraspPrecomputeGoal& grasp_goal = *action_server_old_right_->acceptNewGoal();
    amigo_whole_body_controller::ArmTaskGoal goal;

	/// Set link names
    goal.position_constraint.link_name = "grippoint_right";
    goal.orientation_constraint.link_name = "grippoint_right";
    
    /// Convert goal
    if (!convertGoalType(grasp_goal, goal))
    {
        action_server_old_right_->setAborted();
        return;
    }

    //// HARDCODED FIX!!!
    if(goal.position_constraint.header.frame_id == "/amigo/base_link") goal.position_constraint.header.frame_id = "base_link";

    /// Plan, simulate and execute
    ROS_INFO("Plan, simulate, execute");
    bool result = planSimExecute(goal);

    /// If succeeded, pre-grasp required (meaning moving to an offset) and first_joint_pos_only is false:
    /// Perform grasp motion as well (but this time with target_point_offset.x zero)
    if (result && grasp_goal.PERFORM_PRE_GRASP && !grasp_goal.FIRST_JOINT_POS_ONLY)
    {
		goal.goal_type = "grasp";
        goal.position_constraint.target_point_offset.x = 0.0;
        result = planSimExecute(goal);
    }

    /// If succeeded, set server succeeded
    if (result)
    {
        action_server_old_right_->setSucceeded();
    }
    else
    {
        action_server_old_right_->setAborted();
    }
}

bool WholeBodyPlanner::convertGoalType(const tue_manipulation::GraspPrecomputeGoal& grasp_goal, amigo_whole_body_controller::ArmTaskGoal &goal)
{

    /// Check for absolute or delta (and ambiguous goals)
    bool absolute_requested=false, delta_requested=false;
    const double eps = 1e-6;
    if(fabs(grasp_goal.goal.x)>eps || fabs(grasp_goal.goal.y)>eps || fabs(grasp_goal.goal.z)>eps || fabs(grasp_goal.goal.roll)>eps || fabs(grasp_goal.goal.pitch)>eps || fabs(grasp_goal.goal.yaw)>eps)
        absolute_requested = true;
    if(fabs(grasp_goal.delta.x)>eps || fabs(grasp_goal.delta.y)>eps || fabs(grasp_goal.delta.z)>eps || fabs(grasp_goal.delta.roll)>eps || fabs(grasp_goal.delta.pitch)>eps || fabs(grasp_goal.delta.yaw)>eps)
        delta_requested = true;
    if(absolute_requested && delta_requested)
    {
        ROS_WARN("grasp_precompute_action: goal consists out of both absolute AND delta values. Choose only one!");
        return false;
    }

    if (absolute_requested)
    {
        /// Position constraint
        ROS_INFO("Position constraint: position");
        ROS_INFO("Position constraint: x: %f, y: %f, z: %f",grasp_goal.goal.x, grasp_goal.goal.y, grasp_goal.goal.z);
        goal.position_constraint.header = grasp_goal.goal.header;
        goal.position_constraint.position.x = grasp_goal.goal.x;
        goal.position_constraint.position.y = grasp_goal.goal.y;
        goal.position_constraint.position.z = grasp_goal.goal.z;

        /// Orientation constraint (static)
        ROS_INFO("Orientation constraint: roll: %f, pitch: %f, yaw: %f",grasp_goal.goal.roll, grasp_goal.goal.pitch, grasp_goal.goal.yaw);
        goal.orientation_constraint.header = grasp_goal.goal.header;
        goal.orientation_constraint.orientation = tf::createQuaternionMsgFromRollPitchYaw(grasp_goal.goal.roll, grasp_goal.goal.pitch, grasp_goal.goal.yaw);
    }
    else if (delta_requested)
    {
        /// Create temporary objects
        geometry_msgs::PointStamped point_in, point_out;
        geometry_msgs::QuaternionStamped quat_in, quat_out;

        /// Position constraint
        ROS_INFO("Position constraint: delta position");
        ROS_INFO("Position constraint: x: %f, y: %f, z: %f",grasp_goal.delta.x, grasp_goal.delta.y, grasp_goal.delta.z);
        point_in.header = grasp_goal.delta.header;
        point_in.point.x = grasp_goal.delta.x;
        point_in.point.y = grasp_goal.delta.y;
        point_in.point.z = grasp_goal.delta.z;

        /// Orientation constraint (static)
        ROS_INFO("Orientation constraint: roll: %f, pitch: %f, yaw: %f",grasp_goal.delta.roll, grasp_goal.delta.pitch, grasp_goal.delta.yaw);
        quat_in.header = grasp_goal.delta.header;
        quat_in.quaternion = tf::createQuaternionMsgFromRollPitchYaw(grasp_goal.delta.roll, grasp_goal.delta.pitch, grasp_goal.delta.yaw);
        ROS_INFO("Frame IDs are %s and %s %f %f %f %f", point_out.header.frame_id.c_str(), quat_in.header.frame_id.c_str(), quat_in.quaternion.x,quat_in.quaternion.y,quat_in.quaternion.z,quat_in.quaternion.w);

        /// Transform to base_link
        ros::Time stamp = ros::Time(0);
        try
        {
            listener_.waitForTransform("/amigo/base_link", point_in.header.frame_id,stamp,ros::Duration(1.0));
            listener_.transformPoint("/amigo/base_link", stamp, point_in, point_in.header.frame_id, point_out);
            listener_.transformQuaternion("/amigo/base_link", stamp ,quat_in, quat_in.header.frame_id, quat_out);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            return false;
        }

        // Temp: check frame_ids
        ROS_INFO("Frame IDs are %s and %s %f %f %f %f", point_out.header.frame_id.c_str(), quat_out.header.frame_id.c_str(), quat_out.quaternion.x,quat_out.quaternion.y,quat_out.quaternion.z,quat_out.quaternion.w);

        /// Find the pos of end effector
        KDL::Frame frame_map_tip = simulator_.getFramePose(goal.position_constraint.link_name);
        KDL::Frame frame_map_root = simulator_.getFramePose("base_link");
        std::cout<<frame_map_root.p.x()<<" "<<frame_map_root.p.y()<<" "<<frame_map_root.p.z()<<" "<<goal.position_constraint.header.frame_id<<std::endl;
        KDL::Frame ee_pos = frame_map_root.Inverse() * frame_map_tip;
        ROS_WARN("EE Position point: x: %f, y: %f, z: %f, frame %s and root %s",ee_pos.p.x() , ee_pos.p.y(), ee_pos.p.z(),goal.position_constraint.link_name.c_str(),goal.position_constraint.header.frame_id.c_str());
        point_out.point.x = ee_pos.p.x()    +   point_in.point.x;
        point_out.point.y = ee_pos.p.y()    +   point_in.point.y;
        point_out.point.z = ee_pos.p.z()    +   point_in.point.z;


        /// Fill in data in goal
        goal.position_constraint.header         = point_out.header;
        goal.position_constraint.position       = point_out.point;
        goal.orientation_constraint.header      = quat_out.header;


        /// Fill in orientation
        double rotx, roty, rotz, rotw;
        ee_pos.M.GetQuaternion(rotx, roty, rotz, rotw);

        goal.orientation_constraint.orientation.x = rotx + quat_out.quaternion.x;
        goal.orientation_constraint.orientation.y = roty + quat_out.quaternion.y;
        goal.orientation_constraint.orientation.z = rotz + quat_out.quaternion.z;
        goal.orientation_constraint.orientation.w = rotw + quat_out.quaternion.w;

        ROS_WARN("Delta Position constraint: x: %f, y: %f, z: %f, rotx: %f, roty: %f,rotz: %f,rotw: %f",goal.position_constraint.position.x, goal.position_constraint.position.y, goal.position_constraint.position.z, goal.orientation_constraint.orientation.x,goal.orientation_constraint.orientation.y,goal.orientation_constraint.orientation.z,goal.orientation_constraint.orientation.w);
    }
    else
    {
        ROS_WARN("Neither significant absolute nor significant delta goal, aborting");
        return false;
    }

    /// Default: sphere with radius 2 cm
    ROS_INFO("Position constriant: constraint region shape");
    goal.position_constraint.constraint_region_shape.type = goal.position_constraint.constraint_region_shape.SPHERE;
    goal.position_constraint.constraint_region_shape.dimensions.push_back(0.035);

    ROS_INFO("Orientation constraint: tolerances");
    goal.orientation_constraint.absolute_roll_tolerance = 0.3;
    goal.orientation_constraint.absolute_pitch_tolerance = 0.3;
    goal.orientation_constraint.absolute_yaw_tolerance = 0.3;

    /// Stiffness
    ROS_INFO("Stiffness");
    goal.stiffness.force.x = 70.0;
    goal.stiffness.force.y = 60.0;
    goal.stiffness.force.z = 50.0;
    goal.stiffness.torque.x = 3.0;
    goal.stiffness.torque.y = 3.0;
    goal.stiffness.torque.z = 3.0;

    /// Incase of pre-grasp = true: add pre-grasp offset to target_point_offset
    if (grasp_goal.PERFORM_PRE_GRASP)
    {
        goal.position_constraint.target_point_offset.x = 0.20;        
    } else {
        goal.position_constraint.target_point_offset.x = 0.0;
    }
    goal.position_constraint.target_point_offset.y = 0.0;
    goal.position_constraint.target_point_offset.z = 0.0;

    // ToDo: 'sample' yaw
    // ToDo: perform pre_grasp (test: target_point_offset needs to be integrated in whole-body-controller)
    //  How does it actually work?
    //  A position constraint basically has a target_point_offset and a position, can we use this?
    //  Might be useful anyway (also for looking at something)

    // ToDo: first joint pos only (test: target_point_offset needs to be integrated in whole-body-controller)

    return true;
}

void WholeBodyPlanner::PublishMarkers()
{
    visualization_msgs::MarkerArray marker_array;
    for (unsigned int i = 0; i < constraints_.size(); i++)
    {
        visualization_msgs::Marker marker;
        marker.header = constraints_[i].position_constraint.header;
        marker.header.frame_id = "/amigo"+marker.header.frame_id;
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

void WholeBodyPlanner::octoMapCallback(const octomap_msgs::Octomap::ConstPtr& msg)
{
        octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
        if(tree){
            octomap::OcTreeStamped* octree = dynamic_cast<octomap::OcTreeStamped*>(tree);
            if(!octree){
                ROS_ERROR("No Octomap created");
            }
            else{
                if (planner_==2){
                    planner_global_.task_space_roadmap_->setOctoMap(octree);
                }
                simulator_.collision_avoidance_->setOctoMap(octree);
            }
        }
        else{
            ROS_ERROR("Octomap conversion error");
            exit(1);
        }
}
