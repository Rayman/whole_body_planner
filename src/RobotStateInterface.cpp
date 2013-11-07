#include "whole_body_planner/RobotStateInterface.h"

// tf
#include <tf/transform_listener.h>

RobotStateInterface::RobotStateInterface()
{
    ros::NodeHandle nh_private("~");

    // ToDo: make nice
    //sub_base_ = nh_private.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10, &RobotStateInterface::baseMeasurementCallback, this);
    sub_torso_ = nh_private.subscribe<sensor_msgs::JointState>("/amigo/torso/measurements", 10, &RobotStateInterface::jointMeasurementCallback, this);
    sub_left_arm_ = nh_private.subscribe<sensor_msgs::JointState>("/amigo/left_arm/measurements", 10, &RobotStateInterface::jointMeasurementCallback, this);
    sub_right_arm_ = nh_private.subscribe<sensor_msgs::JointState>("/amigo/right_arm/measurements", 10, &RobotStateInterface::jointMeasurementCallback, this);
    sub_head_pan_ = nh_private.subscribe<std_msgs::Float64>("/head_pan_angle", 10, &RobotStateInterface::headPanMeasurementCallback, this);
    sub_head_tilt_ = nh_private.subscribe<std_msgs::Float64>("/head_tilt_angle", 10, &RobotStateInterface::headTiltMeasurementCallback, this);

    /// Initialize amcl_pose_;
    setAmclPose();

}

RobotStateInterface::~RobotStateInterface()
{

}

std::map<std::string, double> RobotStateInterface::getJointPositions()
{
    return joint_positions_map_;
}

geometry_msgs::PoseWithCovarianceStamped RobotStateInterface::getAmclPose()
{
    return amcl_pose_;
}

//void RobotStateInterface::baseMeasurementCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
//{
//    amcl_pose_ = *msg;
//

void RobotStateInterface::jointMeasurementCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    for(unsigned int i = 0; i < msg->name.size(); ++i)
    {
        joint_positions_map_[msg->name[i]] = msg->position[i];
    }
}

void RobotStateInterface::headPanMeasurementCallback(const std_msgs::Float64::ConstPtr& msg)
{
    ROS_WARN_ONCE("whole body planner cannot process head pan measurements yet");
}

void RobotStateInterface::headTiltMeasurementCallback(const std_msgs::Float64::ConstPtr& msg)
{
    ROS_WARN_ONCE("whole body planner cannot process head tilt measurements yet");
}

void RobotStateInterface::setAmclPose()
{
    listener_.waitForTransform("/map","/amigo/base_link",ros::Time(0),ros::Duration(1.0)); // Is the latest available transform
    geometry_msgs::PoseStamped base_link_pose, map_pose;
    base_link_pose.header.frame_id = "/amigo/base_link";
    base_link_pose.pose.position.x = 0.0;
    base_link_pose.pose.position.y = 0.0;
    base_link_pose.pose.position.z = 0.0;
    base_link_pose.pose.orientation.x = 0.0;
    base_link_pose.pose.orientation.y = 0.0;
    base_link_pose.pose.orientation.z = 0.0;
    base_link_pose.pose.orientation.w = 1.0;
    listener.transformPose("/map", base_link_pose, map_pose);
    amcl_pose_.pose.pose = map_pose.pose;
}
