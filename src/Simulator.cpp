#include "whole_body_planner/Simulator.h"

Simulator::Simulator()
{

}

Simulator::Simulator(const double Ts)
{
    initialize(Ts);
}

void Simulator::initialize(const double Ts)
{
    wbc_ = new WholeBodyController(Ts);

    CollisionAvoidance::collisionAvoidanceParameters ca_param;
    loadParameterFiles(ca_param);

    collision_avoidance_ = new CollisionAvoidance(ca_param, Ts);
    if (!wbc_->addMotionObjective(collision_avoidance_)) {
        ROS_ERROR("Could not initialize collision avoidance");
        exit(-1);
    }

    q_ref_.resize(wbc_->getJointNames().size());
    q_ref_.setZero();
    qdot_ref_.resize(wbc_->getJointNames().size());
    qdot_ref_.setZero();
    joint_name_to_index_ = wbc_->getJointNameToIndex();
    marker_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    trajectory_pub_ = n_.advertise<nav_msgs::Path>("path", 1);

}

Simulator::~Simulator()
{
    marker_pub_.shutdown();
    delete wbc_;
    delete collision_avoidance_;
}

bool Simulator::checkFeasibility(const std::vector<amigo_whole_body_controller::ArmTaskGoal>& constraints, const unsigned int& max_iter, int& error_index)
{

    // Set stuff to zero and initialize
    //ROS_INFO("WBC-Simulator: Starting feasibility check");
    error_index = 0;
    path_.header.frame_id = constraints[0].position_constraint.header.frame_id;
    path_.poses.resize(0);

    for(std::vector<amigo_whole_body_controller::ArmTaskGoal>::const_iterator it_constraints = constraints.begin(); it_constraints != constraints.end(); ++it_constraints)
    {
        amigo_whole_body_controller::ArmTaskGoal constraint = *it_constraints;
        ROS_INFO("WBC-Simulator: Checking constraint %i (%s) for feasibility", error_index, constraint.goal_type.c_str());
        CartesianImpedance* cartesian_impedance = new CartesianImpedance(constraint.position_constraint.link_name);

        // Check if there is a motion objective for the frame
        if(wbc_->getCartesianImpedances(constraint.position_constraint.link_name,constraint.position_constraint.header.frame_id).size()>0)
        {
            for (unsigned int i = 0; i < wbc_->getCartesianImpedances(constraint.position_constraint.link_name,constraint.position_constraint.header.frame_id).size(); i++) {
                wbc_->removeMotionObjective(wbc_->getCartesianImpedances(constraint.position_constraint.link_name,constraint.position_constraint.header.frame_id)[i]);
            }
        }
        if (constraint.goal_type.compare("grasp")==0){
            ROS_INFO("Received grasp goal, removing object pose from STATIC octomap, this should be generalized!");
            collision_avoidance_->removeOctomapBBX(constraint.position_constraint.position, constraint.position_constraint.header.frame_id);
        }

        // Transform constrain to goal_pose for cartesian impedance wbc
        geometry_msgs::PoseStamped goal_pose;
        goal_pose.pose.position = constraint.position_constraint.position;
        goal_pose.pose.orientation = constraint.orientation_constraint.orientation;
        goal_pose.header.frame_id = constraint.position_constraint.header.frame_id;

        cartesian_impedance->setGoal(goal_pose);
        cartesian_impedance->setGoalOffset(constraint.position_constraint.target_point_offset);
        //ROS_INFO("WBC-Simulator: Set goal.");
        //ROS_INFO("WBC-Simulator: Goal: %f %f %f %f %f %f",goal_pose.pose.position.x,goal_pose.pose.position.y,goal_pose.pose.position.z,goal_pose.pose.orientation.x,goal_pose.pose.orientation.y,goal_pose.pose.orientation.z);
        cartesian_impedance->setImpedance(constraint.stiffness);
        cartesian_impedance->setPositionTolerance(constraint.position_constraint.constraint_region_shape);
        cartesian_impedance->setOrientationTolerance(constraint.orientation_constraint.absolute_roll_tolerance, constraint.orientation_constraint.absolute_pitch_tolerance, constraint.orientation_constraint.absolute_yaw_tolerance);
        if (!wbc_->addMotionObjective(cartesian_impedance)) {
            ROS_ERROR("Could not initialize cartesian impedance for new motion objective");
            exit(-1);
        }
        //ROS_INFO("WBC-Simulator: Motion objective added to simulation");

        // Update the wbc until either the constraint is satisfied or the maximum number of iterations has been met        
        unsigned int iter = 0;
        double delta_time = ros::Time::now().toSec();
        while (iter < max_iter && cartesian_impedance->getStatus() != 1)
        {
            wbc_->update(q_ref_, qdot_ref_);
            for (std::map<std::string, unsigned int>::iterator iter2 = joint_name_to_index_.begin(); iter2 != joint_name_to_index_.end(); ++iter2)
            {
                //ROS_INFO("Setting %s to %f",iter2->first.c_str(),q_ref_[iter2->second]);
                wbc_->setMeasuredJointPosition(iter2->first, q_ref_[iter2->second]);
                //ROS_INFO("Pushing back FK Pose");
                //path_.poses.push_back(wbc_->robot_state_.getFKPoseStamped("/amigo/"+goal_pose.header.frame_id)); // ToDo: remove hack

                //geometry_msgs::PoseStamped fk = wbc_->robot_state_.getFKPoseStamped(goal_pose.header.frame_id);
                //ROS_INFO("FK tip = [%f\t,%f,\t%f]", fk.pose.position.x, fk.pose.position.y, fk.pose.position.z);
            }
            //std::cout<<"Error x: "<<cartesian_impedance->getError().vel.data[0]<<" y: "<<cartesian_impedance->getError().vel.data[1]<<" z: "<<cartesian_impedance->getError().vel.data[2]<<" r: "<<cartesian_impedance->getError().rot.data[0]<<" p: "<<cartesian_impedance->getError().rot.data[1]<<" y: "<<cartesian_impedance->getError().rot.data[2]<<std::endl;
            ++iter;
        }
        ROS_WARN("Forward simulation took %f [s] and %i/%i iterations",(ros::Time::now().toSec()-delta_time), iter, max_iter);

        if (iter == max_iter)
        {
            // Display current position
            //FKpos = wbc_->robot_state_.getFK(goal_pose.header.frame_id);
            //ROS_INFO("FKPOS: x: %f, y: %f, z: %f",FKpos.p.x(),FKpos.p.y(),FKpos.p.z());
            ROS_INFO("Maximum iterations reached: remaining error (x,y,z): (%f, %f, %f,)",cartesian_impedance->getError().vel.data[0],cartesian_impedance->getError().vel.data[1],cartesian_impedance->getError().vel.data[2]);
            PublishMarkers(constraint,false);
            trajectory_pub_.publish(path_);
            return false;
        }
        else
        {
            ROS_INFO("Feasible joint-space trajectory found for constraint: %i",error_index);
            PublishMarkers(constraint,true);

            ++error_index;
        }
    }

    error_index = -1; // All constraints are feasible
    trajectory_pub_.publish(path_);
    return true;
}

bool Simulator::setInitialJointConfiguration(const std::map<std::string,double>& joint_positions, const geometry_msgs::PoseWithCovarianceStamped& amcl_pose)
{
    ROS_INFO("WBC-Simulator : Setting initial joint configurations");
    /// Set joint configuration
    for(std::map<std::string, double>::const_iterator iter = joint_positions.begin(); iter != joint_positions.end(); ++iter)
    {
        ROS_DEBUG("%s = %f",iter->first.c_str(),iter->second);
        wbc_->setMeasuredJointPosition(iter->first, iter->second);
    }
    /// Set amcl pose
    ROS_INFO("AMCL pose x: %f, y: %f",amcl_pose.pose.pose.position.x,amcl_pose.pose.pose.position.y);

    /// Convert to KDL::Frame
    KDL::Frame amcl_pose_;
    amcl_pose_.p.x(amcl_pose.pose.pose.position.x);
    amcl_pose_.p.y(amcl_pose.pose.pose.position.y);

    amcl_pose_.M = KDL::Rotation::Quaternion(   amcl_pose.pose.pose.orientation.x,
                                                amcl_pose.pose.pose.orientation.y,
                                                amcl_pose.pose.pose.orientation.z,
                                                amcl_pose.pose.pose.orientation.w);

    wbc_->robot_state_.setAmclPose(amcl_pose_);
    return true;
}

nav_msgs::Path Simulator::getPath()
{
    return path_;
}

KDL::Frame Simulator::getFramePose(const std::string& frame_name)
{
    wbc_->robot_state_.collectFKSolutions();
    return wbc_->robot_state_.getFK(frame_name);
}

void Simulator::PublishMarkers( const amigo_whole_body_controller::ArmTaskGoal& constraint, bool result)
{
    visualization_msgs::Marker marker;
    marker.header = constraint.position_constraint.header;
    marker.header.frame_id = "/amigo/base_link";
    marker.header.stamp = ros::Time();
    marker.lifetime = ros::Duration(15.0);
    marker.ns = "forward_sim";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::ARROW;
    static int id_sim = 0;
    marker.id = id_sim++;
    marker.scale.x = 0.5;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.pose.position = constraint.position_constraint.position;
    marker.pose.orientation = constraint.orientation_constraint.orientation;
    marker.color.a = 0.3;
    if (result){
        marker.color.r = 0.1; marker.color.g = 0.99; marker.color.b = 0.1; //green
    }
    else{
        marker.color.r = 0.99; marker.color.g = 0.1; marker.color.b = 0.1; //red
    }
    marker_pub_.publish( marker );
}

KDL::Frame Simulator::transformToMap( const amigo_whole_body_controller::ArmTaskGoal& constraint )
{
    /// Get the goal pose in root frame
    KDL::Frame Frame_root_goal;
    Frame_root_goal.p.x(constraint.position_constraint.position.x);
    Frame_root_goal.p.y(constraint.position_constraint.position.y);
    Frame_root_goal.p.z(constraint.position_constraint.position.z);
    Frame_root_goal.M = KDL::Rotation::Quaternion(constraint.orientation_constraint.orientation.x,
                                                   constraint.orientation_constraint.orientation.y,
                                                   constraint.orientation_constraint.orientation.z,
                                                   constraint.orientation_constraint.orientation.w);

    //ROS_INFO("FK pose goal in root frame (x,y,z) = (%f,%f,%f)",Frame_root_goal.p.x(),Frame_root_goal.p.y(),Frame_root_goal.p.z());

    /// Get the pose of the root frame (of the goal) in map
    std::map<std::string, KDL::Frame>::iterator itrRF = wbc_->robot_state_.fk_poses_.find(constraint.position_constraint.header.frame_id);
    KDL::Frame Frame_map_root = itrRF->second;
    //ROS_INFO("FK pose root frame in map (x,y,z) = (%f,%f,%f)", Frame_map_root.p.x(), Frame_map_root.p.y(), Frame_map_root.p.z());

    /// Convert the goal pose to map
    return Frame_map_root * Frame_root_goal;
    //ROS_INFO("FK pose goal in map frame (x,y,z) = (%f,%f,%f)",Frame_map_goal.p.x(),Frame_map_goal.p.y(),Frame_map_goal.p.z());

}

void Simulator::transformToRoot(std::vector<amigo_whole_body_controller::ArmTaskGoal>& constraints, const amigo_whole_body_controller::ArmTaskGoal &goal)
{
    /// Get the pose of the root frame (of the goal) in map
    std::map<std::string, KDL::Frame>::iterator itrRF = wbc_->robot_state_.fk_poses_.find(goal.position_constraint.header.frame_id);
    KDL::Frame Frame_root_map = itrRF->second.Inverse();

    for( int constraint_id = 0; constraint_id  < int ( constraints.size() ); ++constraint_id )
    {
        /// Get the constraints poses in map frame
        KDL::Frame Frame_map_goal;
        Frame_map_goal.p.x(constraints[constraint_id].position_constraint.position.x);
        Frame_map_goal.p.y(constraints[constraint_id].position_constraint.position.y);
        Frame_map_goal.p.z(constraints[constraint_id].position_constraint.position.z);
        Frame_map_goal.M = KDL::Rotation::Quaternion(constraints[constraint_id].orientation_constraint.orientation.x,
                                                     constraints[constraint_id].orientation_constraint.orientation.y,
                                                     constraints[constraint_id].orientation_constraint.orientation.z,
                                                     constraints[constraint_id].orientation_constraint.orientation.w);
        //ROS_INFO("FK pose goal in map frame (x,y,z) = (%f,%f,%f)",Frame_map_goal.p.x(),Frame_map_goal.p.y(),Frame_map_goal.p.z());

        /// Transform the goal pose to root from map
        //ROS_INFO("FK pose root frame in map (x,y,z) = (%f,%f,%f)", Frame_map_root.p.x(), Frame_map_root.p.y(), Frame_map_root.p.z());
        KDL::Frame Frame_root_goal = Frame_root_map * Frame_map_goal;

        /// Convert back to constraint
        constraints[constraint_id].position_constraint.position.x = Frame_root_goal.p.x();
        constraints[constraint_id].position_constraint.position.y = Frame_root_goal.p.y();
        constraints[constraint_id].position_constraint.position.z = Frame_root_goal.p.z();
        Frame_root_goal.M.GetQuaternion(constraints[constraint_id].orientation_constraint.orientation.x,
                                        constraints[constraint_id].orientation_constraint.orientation.y,
                                        constraints[constraint_id].orientation_constraint.orientation.z,
                                        constraints[constraint_id].orientation_constraint.orientation.w);
        constraints[constraint_id].position_constraint.header.frame_id = goal.position_constraint.header.frame_id;
        //ROS_INFO("FK pose goal in root frame (x,y,z) = (%f,%f,%f)",Frame_root_goal.p.x(),Frame_root_goal.p.y(),Frame_root_goal.p.z());
    }

}

void Simulator::loadParameterFiles(CollisionAvoidance::collisionAvoidanceParameters &ca_param)
{
    ros::NodeHandle n("~");
    std::string ns = ros::this_node::getName();
    n.param<double> (ns+"/collision_avoidance/self_collision/F_max", ca_param.self_collision.f_max, 1.0);
    n.param<double> (ns+"/collision_avoidance/self_collision/d_threshold", ca_param.self_collision.d_threshold, 1.0);
    n.param<int> (ns+"/collision_avoidance/self_collision/order", ca_param.self_collision.order, 1);

    n.param<double> (ns+"/collision_avoidance/environment_collision/F_max", ca_param.environment_collision.f_max, 1.0);
    n.param<double> (ns+"/collision_avoidance/environment_collision/d_threshold", ca_param.environment_collision.d_threshold, 1.0);
    n.param<int> (ns+"/collision_avoidance/environment_collision/order", ca_param.environment_collision.order, 1);
    n.getParam("/map_3d/resolution", ca_param.environment_collision.octomap_resolution);
    std::cout<<ca_param.self_collision.f_max<<ca_param.self_collision.d_threshold<<ca_param.environment_collision.f_max<<ca_param.environment_collision.d_threshold<<std::endl;
}

