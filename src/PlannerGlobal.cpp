#include "whole_body_planner/PlannerGlobal.h"
PlannerGlobal::PlannerGlobal()
{
    // Setup planning object
    task_space_roadmap_ = new TaskSpaceRoadmap();

    // Setup visualization object
    visualizer_ = new PlanningVisualizer();
}

PlannerGlobal::~PlannerGlobal()
{
    delete wbc_;
    delete task_space_roadmap_;
    delete visualizer_;
}

bool PlannerGlobal::computeConstraints(const amigo_whole_body_controller::ArmTaskGoal& goal_constraint, std::vector<amigo_whole_body_controller::ArmTaskGoal>& constraints)
{
    /// Get Start Position and convert from KDL to PoseStamped message
    KDL::Frame kdl_start_pose =  wbc_->robot_state_.getFK(goal_constraint.position_constraint.link_name);
    geometry_msgs::PoseStamped msg_start_pose;
    msg_start_pose.header.frame_id = "/map";
    msg_start_pose.pose.position.x = kdl_start_pose.p.x();
    msg_start_pose.pose.position.y = kdl_start_pose.p.y();
    msg_start_pose.pose.position.z = kdl_start_pose.p.z();
    kdl_start_pose.M.GetQuaternion(msg_start_pose.pose.orientation.x,
                                   msg_start_pose.pose.orientation.y,
                                   msg_start_pose.pose.orientation.z,
                                   msg_start_pose.pose.orientation.w);

    /// Create roadmap and find global path
    if(!task_space_roadmap_->plan(goal_constraint,msg_start_pose))
    {
        ROS_INFO("NO PLAN!");
    }

    /// Visualize roadmap and plan
    std::vector<std::vector<double> > coordinates = task_space_roadmap_->convertSolutionToVector();
    visualizer_->displaySamples(task_space_roadmap_->getPlanData());
    visualizer_->displayGraph(task_space_roadmap_->getPlanData());
    visualizer_->displayPath(coordinates,1);

    /// Get the path
    constraints = task_space_roadmap_->getPlan();
    ROS_INFO("Constraints: size = %i",(int)constraints.size());

    /// Forward simulation
    KDL::JntArray q_current;
    q_current.resize(wbc_->getJointNames().size());
    Eigen::VectorXd q_ref(q_current.data.rows());
    Eigen::VectorXd qdot_ref(q_current.data.rows());

    for(std::vector<amigo_whole_body_controller::ArmTaskGoal>::iterator it_constraints = constraints.begin(); it_constraints != constraints.end(); ++it_constraints)
    {
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

        if (!wbc_->addMotionObjective(cartesian_impedance)) {
            ROS_ERROR("Could not initialize cartesian impedance for new motion objective");
            exit(-1);
        }

        // Update the wbc
        for(unsigned int num_iter = 0; num_iter < 10; ++num_iter)
        {
            wbc_->update(q_ref, qdot_ref);
            for(unsigned int i = 0; i < wbc_->getJointNames().size(); ++i)
            {
                wbc_->setMeasuredJointPosition(wbc_->getJointNames()[i], wbc_->getJointReferences()[i]);
                //std::cout<<wbc_->getJointNames()[i]<<": "<<wbc_->getJointReferences()[i]<<std::endl;
            }
            std::cout<<"Error x: "<<cartesian_impedance->getError().vel.data[0]<<" y: "<<cartesian_impedance->getError().vel.data[1]<<" z: "<<cartesian_impedance->getError().vel.data[2]<<std::endl;
        }
    }
    return true;
}
