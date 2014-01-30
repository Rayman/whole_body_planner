/*!
 * \author Teun Derksen
 * \date June 2013
 * \version 0.1
 */

#ifndef PLANNINGVISUALIZER_H_
#define PLANNINGVISUALIZER_H_

// RViz
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Planner
#include <whole_body_planner/TaskSpaceRoadmap.h>

/** \brief Class for global planning visualization */
class PlanningVisualizer{

public:

    /** \brief Constructor */
    PlanningVisualizer();

    /** \brief Deconstructor */
    virtual ~PlanningVisualizer();

    /** \brief Display explored graph (roadmap) */
    void displayGraph(ompl::base::PlannerDataPtr planner_data_);

    /** \brief Display milestones */
    void displaySamples(ompl::base::PlannerDataPtr planner_data_);

    /** \brief Display resulting path */
    /** \param Int {init|simplified|replan} = {1|2|3} */
    /** \param Coordinates */
    void displayPath( const std::vector<std::vector<double> > coordinates, int plan_type = 0);

    /** \brief Display forward simulated robot */
    void displayRobot( const std::vector<double> coordinate, bool result);

protected:

private:

      /** \brief Helper Function: gets the {x,y,z} coordinates for a given vertex_id} */
    std::vector<double> getCoordinates(ompl::base::PlannerDataVertex vertex);

    /** \brief Helper Function: to visualize a line */
    void interpolateLine( double x1, double y1, double z1, double x2, double y2, double z2, visualization_msgs::Marker* marker, std_msgs::ColorRGBA* color );

    /** \brief A shared ROS publisher for visualization in RViz */
    ros::Publisher marker_pub_,markera_pub_;

    /** \brief A shared private node handle */
    ros::NodeHandle n_;
};

#endif
