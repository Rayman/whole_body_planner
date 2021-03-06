/*!
 * \author Teun Derksen
 * \date May, 2013
 * \version 0.1
 */

#ifndef TASKSPACEROADMAP_H_
#define TASKSPACEROADMAP_H_

// ROS
#include <ros/ros.h>

// OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>

// Sampling method
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/base/samplers/MaximizeClearanceValidStateSampler.h>
#include <whole_body_planner/VisibilityBasedValidStateSampler.h>

// Motion checker method
#include <whole_body_planner/BoundingBoxMotionValidator.h>

// Arm-navigation_msgs
#include <arm_navigation_msgs/PositionConstraint.h>
#include <arm_navigation_msgs/OrientationConstraint.h>

#include <amigo_whole_body_controller/ArmTaskGoal.h>

// Misc
#include <vector>
#include <kdl/frames.hpp>

// Map
#include <octomap_msgs/Octomap.h>

#include <fstream>

namespace ob = ompl::base;
namespace og = ompl::geometric;



/** \brief Class for global planning  */
class TaskSpaceRoadmap{

public:

    /** \brief Constructor */
    TaskSpaceRoadmap();

    /** \brief Deconstructor */
    virtual ~TaskSpaceRoadmap();

    /** \brief Initial planning */
    bool plan(const amigo_whole_body_controller::ArmTaskGoal& goal_constraint, const KDL::Frame& start_pose, const KDL::Frame& base_pose);

    /** \brief Re planning */
    bool replan(const KDL::Frame& start_pose);

    /** \brief Shortcut plan to vector format */
    std::vector<std::vector<double> > shortCutPlanToVector();

    /** \brief Simplifiy plan to ArmTaskGoal format */
    std::vector<amigo_whole_body_controller::ArmTaskGoal> shortCutPlan();

    /** \brief Smooth plan to vector format */
    std::vector<std::vector<double> > smoothPlanToVector();

    /** \brief Simplifiy plan to ArmTaskGoal format */
    std::vector<amigo_whole_body_controller::ArmTaskGoal> smoothPlan();

    /** \brief Simplifiy plan to vector format */
    std::vector<std::vector<double> > simplifyPlanToVector();

    /** \brief Simplifiy plan to ArmTaskGoal format */
    std::vector<amigo_whole_body_controller::ArmTaskGoal> simplifyPlan();

    /** \brief Interpolate plan to vector format */
    std::vector<std::vector<double> > interpolatePlanToVector();

    /** \brief Simplifiy plan to ArmTaskGoal format */
    std::vector<amigo_whole_body_controller::ArmTaskGoal> interpolatePlan();


    /** \brief Print tags of each sample, used for debugging */
    void printTags();

    /** \brief Convert states in real vector space */
    std::vector<std::vector<double> > convertSolutionToVector();
    std::vector<amigo_whole_body_controller::ArmTaskGoal> convertSolutionToArmTaskGoal();


    /** \brief Octomap instance */
    octomap::OcTreeStamped* octomap_;

    /** \brief Get the planned data */
    ompl::base::PlannerDataPtr getPlanData()
    {
        return planner_data_;
    }
    /** \brief Get the planner status */
    int getStatus()
    {
        return status;
    }

    void setOctoMap(octomap::OcTreeStamped* octree);

    std::ofstream myfile;
protected:

    /** \brief Possible sampling methods (default = VisibilityBased) */
    enum sampling_method {VisibilityBased, ObstacleBased, Uniformly, MaximumClearance} sampler_;

private:

    /** \brief The resulting roadmap that was searched */
    ob::PlannerDataPtr planner_data_;

    /** \brief Save the simple setup until the program ends so that the planner
    data is not lost */
    og::SimpleSetupPtr simple_setup_;

    /** \brief The used planner, accesible to perform roadmap operations */
    og::PRM *prm;

    /** \brief A shared private node handle */
    ros::NodeHandle n_;

    /** \brief Ros subscriber to octomap */
    ros::Subscriber octomap_sub;

    /** \brief The number of dimensions (ToDo = Make Variable) */
    static const unsigned int DIMENSIONS = 3;

    /** \brief  Goal constraint received from planner class */
    amigo_whole_body_controller::ArmTaskGoal goal_constraint_;

    double solution_time;
    double validity_checking_resolution;
    double octomap_resolution;
    double simplification_time;
    int clearance_attempts;

    /** \brief  Status of the planner:
    0 = No path
    1 = Initial feasible plan
    2 = Infeasible plan
    3 = Feasible re-plan found */
    int status;

    /** \brief Callback function for octomap */
    void octoMapCallback(const octomap_msgs::Octomap::ConstPtr& msg);

    /** \brief Solve the problem definition */
    bool solve();

    /** \brief Helper Function: gets the {x,y,z} coordinates for a given vertex_id} */
    std::vector<double> getCoordinates(unsigned int vertex_id);

    /** \brief Validity checker of states */
    bool isStateValid(const ob::State *state);

    /** \brief Allocate sampling method */
    ob::ValidStateSamplerPtr allocValidStateSampler(const ob::SpaceInformation *si);

    ob::ValidStateSamplerPtr allocMaximizeClearanceStateSampler(const ob::SpaceInformation *si);

    /** \brief Construct the space */
    ompl::base::StateSpacePtr constructSpace(const unsigned int dimension = 3);

    /** \brief Set the bounds */
    void setBounds(ompl::base::StateSpacePtr space, const KDL::Frame& start_pose, const KDL::Frame& base_pose);
};

#endif
