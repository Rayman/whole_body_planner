/*!
 * \author Janno Lunenburg
 * \date June 2013
 * \version 0.1
 */

#ifndef ROBOTSTATEINTERFACE_H_
#define ROBOTSTATEINTERFACE_H_


// ROS
#include "ros/ros.h"

// Messages
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

class RobotStateInterface
{

public:

    /**
      * Constructor
      */
    RobotStateInterface();

    /**
      * Deconstructor
      */
    virtual ~RobotStateInterface();

    /**
      * Returns the current map with joint names and joint positions
      */
    std::map<std::string, double> getJointPositions();

    /**
      * Returns the current amcl_pose
      */
    geometry_msgs::PoseWithCovarianceStamped getAmclPose();

    /**
      * Function sets base pose (using tf)
      * This is required since amcl only publishes when the pose is updated
      */
    void setAmclPose();

protected:

    /// Subscribers and callback functions
    tf::TransformListener listener_;
    //ros::Subscriber sub_base_;
    ros::Subscriber sub_torso_;
    ros::Subscriber sub_left_arm_;
    ros::Subscriber sub_right_arm_;
    ros::Subscriber sub_head_pan_;
    ros::Subscriber sub_head_tilt_;

    /**
      * Callback function for amcl pose (=base pose in map frame)
      */
    void baseMeasurementCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    /**
      * Callback function for generic joint state messages
      */
    void jointMeasurementCallback(const sensor_msgs::JointState::ConstPtr& msg);

    /**
      * Callback function for head pan angle
      * Is obsolete as soon as head controller publishes joint state messages
      */
    void headPanMeasurementCallback(const std_msgs::Float64::ConstPtr& msg);

    /**
      * Callback function for head tilt angle
      * Is obsolete as soon as head controller publishes joint state messages
      */
    void headTiltMeasurementCallback(const std_msgs::Float64::ConstPtr& msg);

    /**
      * Map containing the joint names and corresponding positions
      */
    std::map<std::string, double> joint_positions_map_;

    /**
      * Message containing the amcl-pose, i.e., the pose of the base_link frame in map frame
      */
    geometry_msgs::PoseWithCovarianceStamped amcl_pose_;

};

#endif
