#include <whole_body_planner/PlanningVisualizer.h>

PlanningVisualizer::PlanningVisualizer()
{
    marker_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    markera_pub_ = n_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
}

PlanningVisualizer::~PlanningVisualizer()
{
    marker_pub_.shutdown();
}

void PlanningVisualizer::displayRobot( const std::vector<double> coordinate, bool result)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";

    // Set the timestamp.
    marker.header.stamp = ros::Time();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    marker.ns = "forward_sim";

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;

    // Set the marker type.
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://amigo_robot_model/Media/meshes/forearmtextured.dae";

    // Set the id
    static int id_sim = 100;
    marker.id = id_sim++;

    // Set the scaling factor
    marker.scale.x = 0.0004;
    marker.scale.y = 0.0004;
    marker.scale.z = 0.0004;

    // Set the orientation needs to be included in ´coordinate´
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 1.17; //zero = pi/2
    marker.pose.orientation.w = 1.0;

    // Make line color { converged||failed }
    marker.color.a = 0.3; // Transparancy

    if (result) // green
    {
        marker.color.r = 0.1;
        marker.color.g = 0.99;
        marker.color.b = 0.1;

    }
    else //red
    {
        marker.color.r = 0.99;
        marker.color.g = 0.1;
        marker.color.b = 0.1;
    }

    marker.pose.position.x = coordinate[0]-0.2;
    marker.pose.position.y = coordinate[1];
    marker.pose.position.z = coordinate[2];
    //marker.pose.orientation.x = coordinate[3];
    //marker.pose.orientation.y = coordinate[4];
    //marker.pose.orientation.z = coordinate[5]; //zero = pi/2
    //marker.pose.orientation.w = coordinate[6];
    marker_pub_.publish( marker );

}


void PlanningVisualizer::displayPath( const std::vector<std::vector<double> > coordinates, int plan_type)
{
    if (coordinates.size()==0) ROS_WARN("No coordinates received!");
    if (plan_type == 0) ROS_INFO("Do not know what planning-phase, default = initial");

    visualization_msgs::Marker marker;

    // Set the frame ID and timestamp.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.lifetime = ros::Duration(35.0);

    // Set the namespace and id for this marker.  This serves to create a unique ID
    marker.ns = "result_path";

    // Set the marker type.
    marker.type = visualization_msgs::Marker::LINE_LIST;

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;

    // Provide a new id every call to this function
    static int result_id;
    result_id++;
    marker.id = result_id;

    marker.scale.x = 0.004;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 1.0;

    // Color depended of plan_type
    if (plan_type == 3) // re-plan
    {
        marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0;

    }
    else if (plan_type == 2) // simplified
    {
        marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.5;
    }
    else // default = initial
    {
        marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;
    }

    // Get the initial points
    double x1 = coordinates[0][0];
    double y1 = coordinates[0][1];
    double z1 = coordinates[0][2];

    // Declare the second points
    double x2;
    double y2;
    double z2;

    // Convert path coordinates to red line
    for( unsigned int i = 1; i < coordinates.size(); ++i )
    {
        x2 = coordinates[i][0];
        y2 = coordinates[i][1];
        z2 = coordinates[i][2];

        // Points
        geometry_msgs::Point point_a;
        geometry_msgs::Point point_b;

        // First point
        point_a.x = x1;
        point_a.y = y1;
        point_a.z = z1;

        // Create a second point
        point_b.x = x2;
        point_b.y = y2;
        point_b.z = z2;

        // Add the point pair to the line message
        marker.points.push_back( point_a );
        marker.points.push_back( point_b );

        // Save these coordinates for next line
        x1 = x2;
        y1 = y2;
        z1 = z2;
    }

    // Publish the marker
    marker_pub_.publish( marker );

}

void PlanningVisualizer::displaySamples(ompl::base::PlannerDataPtr planner_data_)
{
    if (planner_data_->numVertices() == 0) ROS_WARN("PlannerDataPtr is empty!, cannot display milestones");

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.lifetime = ros::Duration(5.0);

    // Set the namespace and id for this marker.  This serves to create a unique ID
    marker.ns = "sample_locations";

    // Set the marker type.
    marker.type = visualization_msgs::Marker::SPHERE_LIST;

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;

    // Set the recurring stuff
    marker.id = 0;

    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;

    // Declare vector with coordinates of Samplemarkera_pub_
    std::vector<double> this_vertex(3);

    // Make line color
    marker.color.a = 1.0; // Transparancy

    // Point
    geometry_msgs::Point point;

    // Loop through all verticies
    for( int vertex_id = 0; vertex_id < int( planner_data_->numVertices() ); ++vertex_id )
    {

        this_vertex = getCoordinates( planner_data_->getVertex( vertex_id ) );

        // First point
        point.x = this_vertex[0];
        point.y = this_vertex[1];
        point.z = this_vertex[2];

        // Start and Goal milestones diff colours
        if (vertex_id == 0) //cyan, start
        {
            marker.color.r = 0.1; marker.color.g = 1; marker.color.b = 1;
        }

        else if (vertex_id == 1) //purple, end
        {
            marker.color.r = 1; marker.color.g = 0.1; marker.color.b = 1;
        }
        else //red
        {
            marker.color.r = 0.8; marker.color.g = 0.1; marker.color.b = 0.1;
        }

        // Add the point pair to the line message
        marker.points.push_back( point );
        marker.colors.push_back( marker.color );

    }
    marker_pub_.publish( marker );
}

void PlanningVisualizer::displayGraph(ompl::base::PlannerDataPtr planner_data_)
{

    if (planner_data_->numVertices() == 0) ROS_WARN("PlannerDataPtr is empty!, cannot display graphs");

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.lifetime = ros::Duration(5.0);

    // Set the namespace and id for this marker.  This serves to create a unique ID
    marker.ns = "space_exploration";

    // Set the marker type.
    marker.type = visualization_msgs::Marker::LINE_LIST;

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 0;

    marker.scale.x = 0.002;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    std::vector<double> this_vertex(3);
    std::vector<double> next_vertex(3);


    // Make line color
    std_msgs::ColorRGBA color;
    color.r = 0.0;    color.g = 0.0;    color.b = 1.0;    color.a = 1.0;

    // Loop through all verticies
    for( int vertex_id = 0; vertex_id < int( planner_data_->numVertices() ); ++vertex_id )
    {

        this_vertex = getCoordinates( planner_data_->getVertex( vertex_id ) );

        // Get the out edges from the current vertex
        std::vector<unsigned int> edge_list;
        planner_data_->getEdges( vertex_id, edge_list );

        // Now loop through each edge
        for( std::vector<unsigned int>::const_iterator edge_it = edge_list.begin();
             edge_it != edge_list.end(); ++edge_it)
        {
            // Convert vertex id to next coordinates
            next_vertex = getCoordinates( planner_data_->getVertex( *edge_it ) );
            //std::cout<<*next_vertex<<std::endl;
            //next_vertex = getCoordinates( *edge_it );
            interpolateLine( this_vertex[0], this_vertex[1], this_vertex[2], next_vertex[0], next_vertex[1], next_vertex[2], &marker, &color );
        }

    }
    // Publish the marker
    marker_pub_.publish( marker );

}

std::vector<double> PlanningVisualizer::getCoordinates( ompl::base::PlannerDataVertex vertex )
{
    // Setup vector to return coordinates
    std::vector<double> coordinate(3);

    // Get this vertex's coordinates
    const ompl::base::State *state = vertex.getState();

    if (!state)
    {
        ROS_ERROR("No state found for a vertex");
        exit(1);
    }

    // Convert to RealVectorStateSpace
    const ompl::base::RealVectorStateSpace::StateType *real_state =
            static_cast<const ompl::base::RealVectorStateSpace::StateType*>(state);


    coordinate[0] = real_state->values[0];
    coordinate[1] = real_state->values[1];
    coordinate[2] = real_state->values[2];

    return coordinate;
}


void PlanningVisualizer::interpolateLine( double x1, double y1, double z1, double x2, double y2, double z2, visualization_msgs::Marker* marker, std_msgs::ColorRGBA* color )
{
    // Declare the first point to the line message
    geometry_msgs::Point p;
    p.x = x1;
    p.y = y1;
    p.z = z1;

    // Add the point to the line message
    marker->points.push_back( p );

    // Add colors
    marker->colors.push_back( *color );

    // Declare the second point to the line message
    p.x = x2;
    p.y = y2;
    p.z = z2;

    // Add the point pair to the line message
    marker->points.push_back( p );

    // Add colors
    marker->colors.push_back( *color );
}


