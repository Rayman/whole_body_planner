// ROS
#include "ros/ros.h"

// WholeBodyPlanner
#include "whole_body_planner/WholeBodyPlanner.h"

const double loop_rate_ = 150;

int main(int argc, char **argv) {

    // Initialize node
    ros::init(argc, argv, "whole_body_planner");
    ros::NodeHandle nh_private("~");
    ros::Rate rate(loop_rate_);

    WholeBodyPlanner wbp;

    while (ros::ok()) {

        ros::spinOnce();

        rate.sleep();
    }
}
