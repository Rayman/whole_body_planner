#include "whole_body_planner/RobotStateInterface.h"

RobotStateInterface::RobotStateInterface(const std::map<std::string, unsigned int> joint_name_to_index)
{
    ros::NodeHandle nh_private("~");

    // ToDo: make nice
    sub_base_ = nh_private.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10, &RobotStateInterface::baseMeasurementCallback, this);
    sub_torso_ = nh_private.subscribe<sensor_msgs::JointState>("/torso_controller/measurements", 10, &RobotStateInterface::jointMeasurementCallback, this);
    sub_left_arm_ = nh_private.subscribe<sensor_msgs::JointState>("/arm_left_controller/measurements", 10, &RobotStateInterface::jointMeasurementCallback, this);
    sub_right_arm_ = nh_private.subscribe<sensor_msgs::JointState>("/arm_right_controller/measurements", 10, &RobotStateInterface::jointMeasurementCallback, this);
    sub_head_pan_ = nh_private.subscribe<std_msgs::Float64>("/head_pan_angle", 10, &RobotStateInterface::headPanMeasurementCallback, this);
    sub_head_tilt_ = nh_private.subscribe<std_msgs::Float64>("/head_tilt_angle", 10, &RobotStateInterface::headTiltMeasurementCallback, this);

    /// Set joint name to index map and initialize vectors with joint names and joint positions
    joint_name_to_index_ = joint_name_to_index;

    for (std::map<std::string, unsigned int>::iterator it = joint_name_to_index_.begin(); it != joint_name_to_index_.end(); ++it)
    {
        joint_names_.push_back(it->first);
        joint_positions_.push_back(0);
    }

}

RobotStateInterface::~RobotStateInterface()
{

}

std::map<std::string, double> RobotStateInterface::getJointPositions()
{
    return joint_positions_map_;
}

void RobotStateInterface::baseMeasurementCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    ROS_WARN_ONCE("whole body planner cannot process base measurements yet");
}

void RobotStateInterface::jointMeasurementCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    for(unsigned int i = 0; i < msg->name.size(); ++i) {
        /*        unsigned int index = joint_name_to_index_.find(msg->name[i])->second;
        joint_positions_[index] = msg->position[i];
*/
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
