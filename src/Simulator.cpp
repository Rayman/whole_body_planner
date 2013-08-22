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
    q_ref_.resize(wbc_->getJointNames().size());
    q_ref_.setZero();
    qdot_ref_.resize(wbc_->getJointNames().size());
    qdot_ref_.setZero();
    joint_name_to_index_ = wbc_->getJointNameToIndex();
}

Simulator::~Simulator()
{
    delete wbc_;
}

bool Simulator::checkFeasibility(const std::vector<amigo_whole_body_controller::ArmTaskGoal>& constraints, const unsigned int& max_iter, int& error_index)
{

    // Set stuff to zero and initialize
    ROS_INFO("Starting feasibility check");
    error_index = 0;
    path_.header.frame_id = constraints[0].position_constraint.header.frame_id;
    path_.poses.resize(0);

    for(std::vector<amigo_whole_body_controller::ArmTaskGoal>::const_iterator it_constraints = constraints.begin(); it_constraints != constraints.end(); ++it_constraints)
    {
        ROS_INFO("Checking constraint %i for feasibility",error_index);
        amigo_whole_body_controller::ArmTaskGoal constraint = *it_constraints;
        CartesianImpedance* cartesian_impedance = new CartesianImpedance(constraint.position_constraint.link_name);

        // Check if there is a motion objective for the frame
        if(wbc_->getCartesianImpedances(constraint.position_constraint.link_name,constraint.position_constraint.header.frame_id).size()>0)
        {
            for (unsigned int i = 0; i < wbc_->getCartesianImpedances(constraint.position_constraint.link_name,constraint.position_constraint.header.frame_id).size(); i++) {
                wbc_->removeMotionObjective(wbc_->getCartesianImpedances(constraint.position_constraint.link_name,constraint.position_constraint.header.frame_id)[i]);
            }
        }

        // Transform constrain to goal_pose for cartesian impedance wbc
        geometry_msgs::PoseStamped goal_pose;
        goal_pose.pose.position = constraint.position_constraint.position;
        goal_pose.pose.orientation = constraint.orientation_constraint.orientation;
        goal_pose.header.frame_id = constraint.position_constraint.header.frame_id;
        cartesian_impedance->setGoal(goal_pose);
        cartesian_impedance->setImpedance(constraint.stiffness);
        cartesian_impedance->setPositionTolerance(constraint.position_constraint.constraint_region_shape);
        cartesian_impedance->setOrientationTolerance(constraint.orientation_constraint.absolute_roll_tolerance, constraint.orientation_constraint.absolute_pitch_tolerance, constraint.orientation_constraint.absolute_yaw_tolerance);

        if (!wbc_->addMotionObjective(cartesian_impedance)) {
            ROS_ERROR("Could not initialize cartesian impedance for new motion objective");
            exit(-1);
        }
        ROS_INFO("Motion objective added to simulation");

        // Display current position
        //KDL::Frame FKpos = wbc_->robot_state_.getFK(goal_pose.header.frame_id);
        //ROS_INFO("FKPOS: x: %f, y: %f, z: %f",FKpos.p.x(),FKpos.p.y(),FKpos.p.z());

        // Update the wbc until either the constraint is satisfied or the maximum number of iterations has been met        
        unsigned int iter = 0;
        while (iter < max_iter && cartesian_impedance->getStatus() != 1)
        {
            //ROS_INFO("Updating WBC");
            wbc_->update(q_ref_, qdot_ref_);
            //ROS_INFO("Updated WBC");
            /*
            for(unsigned int i = 0; i < wbc_->getJointNames().size(); ++i)
            {
                wbc_->setMeasuredJointPosition(wbc_->getJointNames()[i], wbc_->getJointReferences()[i]);

                //std::cout<<wbc_->getJointNames()[i]<<": "<<wbc_->getJointReferences()[i]<<std::endl;
            }
            */
            for (std::map<std::string, unsigned int>::iterator iter2 = joint_name_to_index_.begin(); iter2 != joint_name_to_index_.end(); ++iter2)
            {
                ROS_INFO("Setting %s to %f",iter2->first.c_str(),q_ref_[iter2->second]);
                wbc_->setMeasuredJointPosition(iter2->first, q_ref_[iter2->second]);
                //ROS_INFO("Pushing back FK Pose");
                path_.poses.push_back(wbc_->robot_state_.getFKPoseStamped(goal_pose.header.frame_id));
            }
            std::cout<<"Error x: "<<cartesian_impedance->getError().vel.data[0]<<" y: "<<cartesian_impedance->getError().vel.data[1]<<" z: "<<cartesian_impedance->getError().vel.data[2]<<std::endl;
            ++iter;
        }

        if (iter == max_iter)
        {
            // Display current position
            //FKpos = wbc_->robot_state_.getFK(goal_pose.header.frame_id);
            //ROS_INFO("FKPOS: x: %f, y: %f, z: %f",FKpos.p.x(),FKpos.p.y(),FKpos.p.z());
            std::cout<<"Remaining error: x: "<<cartesian_impedance->getError().vel.data[0]<<" y: "<<cartesian_impedance->getError().vel.data[1]<<" z: "<<cartesian_impedance->getError().vel.data[2]<<std::endl;
            return false;
        }
        else
        {
            ++error_index;
        }
    }

    error_index = -1; // All constraints are feasible
    return true;
}

bool Simulator::setInitialJointConfiguration(const std::map<std::string,double>& joint_positions, const geometry_msgs::PoseWithCovarianceStamped& amcl_pose)
{
    ROS_INFO("Setting initial joint configurations");
    /// Set joint configuration
    for(std::map<std::string, double>::const_iterator iter = joint_positions.begin(); iter != joint_positions.end(); ++iter)
    {
        ROS_INFO("%s = %f",iter->first.c_str(),iter->second);
        wbc_->setMeasuredJointPosition(iter->first, iter->second);
    }
    /// Set amcl pose
    ROS_INFO("AMCL pose x: %f, y: %f",amcl_pose.pose.pose.position.x,amcl_pose.pose.pose.position.y);
    wbc_->robot_state_.setAmclPose(amcl_pose);
    return true;
}

nav_msgs::Path Simulator::getPath()
{
    return path_;
}
