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
    RobotStateInterface(const std::map<std::string, unsigned int> joint_name_to_index);

    /**
      * Deconstructor
      */
    virtual ~RobotStateInterface();

    /**
      * Returns the current map with joint names and joint positions
      */
    std::map<std::string, double> getJointPositions();

protected:

    /// Subscribers and callback functions
    ros::Subscriber sub_base_;
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
      * Map containing strings of the various joint names and the corresponding index in the joint vector
      */
    std::map<std::string, unsigned int> joint_name_to_index_;

    /**
      * Vector containing the joint names
      */
    std::vector<std::string> joint_names_;

    /**
      * Vector containing the joint values
      */
    std::vector<double> joint_positions_;

    /**
      * Map containing the joint names and corresponding positions
      */
    std::map<std::string, double> joint_positions_map_;



};

#endif
