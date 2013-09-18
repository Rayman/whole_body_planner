#include "whole_body_planner/TaskSpaceRoadmap.h"

ob::ValidStateSamplerPtr TaskSpaceRoadmap::allocValidStateSampler(const ob::SpaceInformation *si)
{   
    switch (sampler_) {
    case 1:
        ROS_INFO("Setting up sampling method, obstacle based sampling.");
        return ob::ValidStateSamplerPtr(new ob::ObstacleBasedValidStateSampler(si));
        break;
    case 2:
        ROS_INFO("Setting up sampling method, uniformly sampling.");
        return ob::ValidStateSamplerPtr(new ob::UniformValidStateSampler(si));
        break;
    case 3:
        ROS_INFO("Setting up sampling method, maximum clearance sampling.");
        return ob::ValidStateSamplerPtr(new ob::MaximizeClearanceValidStateSampler(si));
        break;
    default:
        ROS_INFO("Setting up sampling method, visibility based sampling.");
        return ob::ValidStateSamplerPtr(new ob::VisibilityBasedValidStateSampler(si));
    }
}

ob::ValidStateSamplerPtr TaskSpaceRoadmap::allocMaximizeClearanceStateSampler(const ob::SpaceInformation *si)
{
    ob::MaximizeClearanceValidStateSampler *s = new ob::MaximizeClearanceValidStateSampler(si);
    s->setNrImproveAttempts(55);
    return ob::ValidStateSamplerPtr(s);
}

bool TaskSpaceRoadmap::isStateValid(const ob::State *state)
{
    const ob::RealVectorStateSpace::StateType& coordinate = *state->as<ob::RealVectorStateSpace::StateType>();
    octomap::OcTreeNode* octree_node = octomap_->search((double)coordinate[0],(double)coordinate[1],(double)coordinate[2]);

    if (octree_node){
        if (!octomap_->isNodeOccupied(octree_node)){
            return true;
        }
        else{
            return false;
        }
    }
    //ROS_WARN("Octree cannot be found, for x = %f, y = %f, z = %f .", coordinate[0],coordinate[1],coordinate[2]);
    return true;
}

TaskSpaceRoadmap::TaskSpaceRoadmap()
{
    //ROS_INFO_STREAM( "OMPL version: " << OMPL_VERSION );
    ROS_INFO("Initializing plannerglobal");

    // Octomap subscription
    octomap_ = new octomap::OcTree(0.05);
    octomap_sub  = n_.subscribe<octomap_msgs::OctomapBinary>("/octomap_binary", 10, &TaskSpaceRoadmap::octoMapCallback,this);

    // Construct space
    ob::StateSpacePtr space = constructSpace();

    // Define a simple setup class
    simple_setup_ = og::SimpleSetupPtr( new og::SimpleSetup(space));

    // Set the planner (PRM)
    prm = new og::PRM( simple_setup_->getSpaceInformation() );
    simple_setup_->setPlanner(ob::PlannerPtr(prm));

    // Set Sampling method {VisibilityBased, ObstacleBased, Uniformly, MaximumClearance}
    sampler_ = VisibilityBased;
    simple_setup_->getSpaceInformation()->setValidStateSamplerAllocator(boost::bind(&TaskSpaceRoadmap::allocValidStateSampler, this, _1));

    // Set Validiy checking
    simple_setup_->setStateValidityChecker(boost::bind(&TaskSpaceRoadmap::isStateValid, this, _1));

    // The interval in which obstacles are checked for between states ( trade-off speed // accuracy )
    simple_setup_->getSpaceInformation()->setStateValidityCheckingResolution(0.005); //0.005
    ROS_INFO("Initialized plannerglobal");

}

TaskSpaceRoadmap::~TaskSpaceRoadmap()
{
    ROS_INFO("Shutting down global planning");
    octomap_sub.shutdown();
    delete octomap_;
}

bool TaskSpaceRoadmap::plan(const amigo_whole_body_controller::ArmTaskGoal &goal_constraint, const KDL::Frame &start_pose)
{
    // Reset constraints
    constraints_.clear();

    // Save goal_constraint
    goal_constraint_ = goal_constraint;

    ROS_INFO("Creating roadmap");

    // Set the bounds
    setBounds(simple_setup_->getStateSpace(), start_pose);

    if (!prm->milestoneCount()==0)
    {
        // Roadmap already exists, only deleting start and goal states.
        prm->clearQuery();
        prm->getProblemDefinition()->clearSolutionPaths();
        prm->getProblemDefinition()->clearGoal();
        prm->getProblemDefinition()->clearStartStates();
    }

    // Set the start and goal states
    ob::ScopedState<> start(simple_setup_->getStateSpace());

    start[0] = start_pose.p.x();
    start[1] = start_pose.p.y();
    start[2] = start_pose.p.z();

    //ROS_WARN("ROADMAP: Start pos, x = %f, y = %f z = %f",start[0],start[1],start[2]);
    if(!start.satisfiesBounds())
    {
        // DIRTY HACK (see SetBounds())
        ROS_WARN("Start pose out of bounds, x = %f, y = %f z = %f",start[0],start[1],start[2]);
        start.enforceBounds();
        ROS_WARN("Start became, x = %f, y = %f z = %f",start[0],start[1],start[2]);
    }

    ob::ScopedState<> goal(simple_setup_->getStateSpace());
    goal[0] = goal_constraint.position_constraint.position.x;
    goal[1] = goal_constraint.position_constraint.position.y;
    goal[2] = goal_constraint.position_constraint.position.z;
   //ROS_WARN("ROADMAP: goal pos, x = %f, y = %f z = %f in %s",goal[0],goal[1],goal[2],goal_constraint.position_constraint.header.frame_id.c_str());

    simple_setup_->setStartAndGoalStates(start, goal);

    // Auto setup parameters
    simple_setup_->setup();

    // Solve
    ob::PlannerStatus solved = simple_setup_->solve( 10.0 );

    if (solved)
    {
        ROS_INFO("Solution found");
        // Get information about the exploration data structure the motion planner used. Used later in visualizing
        planner_data_.reset( new ob::PlannerData( simple_setup_->getSpaceInformation() ) );
        simple_setup_->getPlannerData( *planner_data_ );
        status = 1;
    }
    else
    {
        ROS_WARN("No solution can be found!");
    }
    return solved;
}

bool TaskSpaceRoadmap::replan(const KDL::Frame& start_pose)
{
    constraints_.clear();
    simple_setup_->clear();
    simple_setup_->getSpaceInformation()->setValidStateSamplerAllocator(boost::bind(&TaskSpaceRoadmap::allocMaximizeClearanceStateSampler, this, _1));
    // Set the start and goal states
    ob::ScopedState<> start(simple_setup_->getStateSpace());

    start[0] = start_pose.p.x();
    start[1] = start_pose.p.y();
    start[2] = start_pose.p.z();

    simple_setup_->setStartState(start);

    // Auto setup parameters
    simple_setup_->setup();

    // Solve
    ob::PlannerStatus solved = simple_setup_->solve( 10.0 );

    if (solved)
    {
        ROS_INFO("Solution found");
        // Get information about the exploration data structure the motion planner used. Used later in visualizing
        planner_data_.reset( new ob::PlannerData( simple_setup_->getSpaceInformation() ) );
        simple_setup_->getPlannerData( *planner_data_ );
        status = 1;
    }
    else
    {
        ROS_WARN("No solution can be found!");
    }
    return solved;

}

std::vector<amigo_whole_body_controller::ArmTaskGoal>& TaskSpaceRoadmap::getPlan()
{
    // Get the path
    og::PathGeometric path = simple_setup_->getSolutionPath();
    const std::vector<ob::State*>& states = path.getStates();

    // Format it in correct msg type
    amigo_whole_body_controller::ArmTaskGoal constraint;

    for( int state_id = 0; state_id < int ( states.size() ); ++state_id )
    {
        const ob::State *state = states[ state_id ];

        if (!state){
            ROS_INFO("STATE NOT VALID");
            continue; // no data?
        }

        // Convert to RealVectorStateSpace
        const ob::RealVectorStateSpace::StateType *real_state =
                static_cast<const ob::RealVectorStateSpace::StateType*>(state);

        // Setup constraints
        constraint.position_constraint.link_name = goal_constraint_.position_constraint.link_name;
        constraint.position_constraint.header.frame_id = goal_constraint_.position_constraint.header.frame_id;
        constraint.position_constraint.position.x = real_state->values[0];
        constraint.position_constraint.position.y = real_state->values[1];
        constraint.position_constraint.position.z = real_state->values[2];

        // Add Stiffness

        // Add Constraint Region

        // Add Tolerances

        // Add to constraint vector

        // ToDo nice way to find goal and add orientation constraint
        constraints_.push_back(constraint);
    }
    return constraints_;
}

ob::StateSpacePtr TaskSpaceRoadmap::constructSpace(const unsigned int dimension)
{
    ROS_INFO("plannerglobal plans in %d dimension space, STILL HARDCODED", dimension);
    ob::StateSpacePtr space( new ob::RealVectorStateSpace( DIMENSIONS ));
    return space;
}
void TaskSpaceRoadmap::setBounds(ob::StateSpacePtr space, const KDL::Frame& start_pose)
{
    // Set the bounds for the R^3 now completely dependent on octomap dimensions
    // Problem: when arm is behind kinect, then always outside bounds! and area is TOO big for most planning problems
    // Hardcoded
    ob::RealVectorBounds bounds( DIMENSIONS );


    double x,y,z;
    // Set minimum bound
    octomap_->getMetricMin(x,y,z);
    bounds.setLow(0,start_pose.p.x()-0.4); bounds.setLow(1,start_pose.p.y()-0.25); bounds.setLow(2,0.2);
    //ROS_INFO("Octomap minimum dimensions x= %f, y = %f, z= %f, planning dimensions x= %f, y = %f, z= %f", x,y,z,start_pose.p.x()-0.4,start_pose.p.y()-0.4,0.05 );


    // Set maximum bound
    octomap_->getMetricMax(x,y,z);
    bounds.setHigh(0,start_pose.p.x()+0.5); bounds.setHigh(1,start_pose.p.y()+0.25); bounds.setHigh(2,1.05);
    //ROS_INFO("Octomap maximum dimensions x = %f, y = %f, z = %f", x,y,z);

    space->as<ob::RealVectorStateSpace>()->setBounds( bounds );
}


std::vector<std::vector<double> > TaskSpaceRoadmap::simplifyPlanToVector()
{
    simple_setup_->simplifySolution();
    std::vector<std::vector<double> > coordinates;
    return coordinates = convertSolutionToVector();
}
std::vector<amigo_whole_body_controller::ArmTaskGoal> TaskSpaceRoadmap::simplifyPlan()
{
    simple_setup_->simplifySolution(0.0001);
    std::vector<amigo_whole_body_controller::ArmTaskGoal> constraints_simplified;
    return constraints_simplified = convertSolutionToArmTaskGoal();
}

std::vector<std::vector<double> > TaskSpaceRoadmap::interpolatePlanToVector()
{
    simple_setup_->getSolutionPath().interpolate();
    std::vector<std::vector<double> > coordinates;
    return coordinates = convertSolutionToVector();
}

std::vector<amigo_whole_body_controller::ArmTaskGoal> TaskSpaceRoadmap::interpolatePlan()
{
    simple_setup_->getSolutionPath().interpolate();
    std::vector<amigo_whole_body_controller::ArmTaskGoal> constraints_interpolated;
    return constraints_interpolated = convertSolutionToArmTaskGoal();
}

void TaskSpaceRoadmap::printTags()
{
    for( int vertex_id = 0; vertex_id < int( planner_data_->numVertices() ); ++vertex_id )
    {
        const ob::PlannerDataVertex& vtx = planner_data_->getVertex(vertex_id);
        ROS_INFO("Vertex number %d has tag %d",vertex_id,vtx.getTag());
    }
}

std::vector<std::vector<double> > TaskSpaceRoadmap::convertSolutionToVector()
{
    // Setup vector to return coordinates
    std::vector<double> coordinate(DIMENSIONS);

    // Setup matrix to return results
    std::vector<std::vector<double> > coordinates;

    // Get data
    og::PathGeometric path = simple_setup_->getSolutionPath();

    const std::vector<ob::State*>& states = path.getStates();

    for( int state_id = 0; state_id < int ( states.size() ); ++state_id )
    {
        const ob::State *state = states[ state_id ];
        if (!state)
            continue; // no data?


        // Convert to RealVectorStateSpace
        const ob::RealVectorStateSpace::StateType *real_state =
                static_cast<const ob::RealVectorStateSpace::StateType*>(state);

        coordinate[0] = real_state->values[0];
        coordinate[1] = real_state->values[1];
        coordinate[2] = real_state->values[2];

        // Add to matrix of results {x,y,z}
        coordinates.push_back(coordinate);
    }
    return coordinates;
}

std::vector<amigo_whole_body_controller::ArmTaskGoal> TaskSpaceRoadmap::convertSolutionToArmTaskGoal()
{
    // Setup vector to return coordinates
    std::vector<amigo_whole_body_controller::ArmTaskGoal> constraints;

    // Get data
    og::PathGeometric path = simple_setup_->getSolutionPath();

    const std::vector<ob::State*>& states = path.getStates();
    constraints.resize(states.size());

    for( int state_id = 0; state_id < int ( states.size() ); ++state_id )
    {
        const ob::State *state = states[ state_id ];
        if (!state)
            continue; // no data?


        // Convert to RealVectorStateSpace
        const ob::RealVectorStateSpace::StateType *real_state =
                static_cast<const ob::RealVectorStateSpace::StateType*>(state);

        constraints[state_id].position_constraint.position.x = real_state->values[0];
        constraints[state_id].position_constraint.position.y = real_state->values[1];
        constraints[state_id].position_constraint.position.z = real_state->values[2];
        constraints[state_id].position_constraint.link_name = goal_constraint_.position_constraint.link_name;
        constraints[state_id].position_constraint.header.frame_id = goal_constraint_.position_constraint.header.frame_id;
    }
    return constraints;
}

//unsigned int TaskSpaceRoadmap::deleteMilestone(const std::vector<double> coordinate)
//{
//    ROS_INFO("Deleting");
//}

/////////////////////////
////     Private     ////
/////////////////////////
void TaskSpaceRoadmap::octoMapCallback(const octomap_msgs::OctomapBinary::ConstPtr& msg)
{
    octomap_ = octomap_msgs::binaryMsgDataToMap(msg->data);
}

void TaskSpaceRoadmap::setTags(og::PathGeometric path)
{

    ob::ScopedState<> pathstate(planner_data_->getSpaceInformation());
    ob::ScopedState<> roadmapstate(planner_data_->getSpaceInformation());
    const std::vector<ob::State*>& states = path.getStates();
    for( int state_id = 0; state_id < int ( states.size() ); ++state_id )
    {
        // SCOPEDSTATE NEEDED TO COMPARE STATES, DIFFERENT STATESPACES, FIND WORKAROUND ADDED NEW STATES AFTER EACH ITER
        // ALSO ScopedState ARE NOT REMOVED AFTER EXITING FUNCTION
        // THIS SEEMS STUPID, PATH IS ALREADY KNOWN, WHY LOOP THROUGH EVERY STATE TO CHECK IF IT IS IN SOLUTION!?
        const ob::State *state = states[ state_id ];
        pathstate = state;


        /// TAG STATE WHICH IS IN SOLUTION
        for( int vertex_id = 0; vertex_id < int( planner_data_->numVertices() ); ++vertex_id )
        {
            roadmapstate = planner_data_->getVertex(vertex_id).getState();
            if ( pathstate == roadmapstate )
            {
                planner_data_->tagState (planner_data_->getVertex(vertex_id).getState(), 1000);
                break;
            }
        }
    }

}
std::vector<double> TaskSpaceRoadmap::getCoordinates( unsigned int vertex_id )
{
    ob::PlannerDataVertex vertex = planner_data_->getVertex( vertex_id );

    // Setup vector to return coordinates
    std::vector<double> coordinate(DIMENSIONS);

    // Get this vertex's coordinates
    const ob::State *state = vertex.getState();

    if (!state)
    {
        ROS_ERROR("No state found for a vertex");
        exit(1);
    }

    // Convert to RealVectorStateSpace
    const ob::RealVectorStateSpace::StateType *real_state =
            static_cast<const ob::RealVectorStateSpace::StateType*>(state);


    coordinate[0] = real_state->values[0];
    coordinate[1] = real_state->values[1];
    coordinate[2] = real_state->values[2];

    return coordinate;
}



void TaskSpaceRoadmap::updateMilestones()
{
    for( int vertex_id = 0; vertex_id < int( planner_data_->numVertices() ); ++vertex_id )
    {

        // Acces Planner data, vertex
        const ob::PlannerDataVertex& vtx = planner_data_->getVertex(vertex_id);

        // Get the state information of vertex
        const ob::State *vi = vtx.getState();

        // CHECK IF THE MILESTONE IS STILL VALID
        if(!isStateValid(vi)) //Remove the milestone and its connections
        {
            ROS_INFO("NOT OKAY, vertex %d needs to be removed with tag %d",vertex_id,vtx.getTag());

            // CHECK IF THE MILESTONE IS PART OF THE ROUGH SOLUTION
            if(!vtx.getTag()==0)
            {

                // ADDITIONAL PRIMAIRY CHECKS TO MAKE SURE START AND GOAL ARE OKAY!
                // Check if goal vertex is still okay
                if(planner_data_->isGoalVertex(vertex_id))
                {
                    ROS_INFO("Goal vertex became invalid");

                    // Check how many goal vertices are available
                    if(planner_data_->numGoalVertices()==1)
                    {
                        static double roadmap_build_time_goal = 0.0;
                        roadmap_build_time_goal += 0.2;

                        ROS_WARN("No goal vertices available, growing for %f and expanding for %f [s]",2*roadmap_build_time_goal,roadmap_build_time_goal);
                        ROS_INFO("Number of current states: %i",planner_data_->numVertices());

                        // Grow the roadmap to connect to the goal state
                        prm->growRoadmap(2*roadmap_build_time_goal);
                        ROS_INFO("Number of states after growth: %i",planner_data_->numVertices());

                        // Expand the roadmap to connect to the goal state
                        prm->expandRoadmap(roadmap_build_time_goal);
                        ROS_INFO("Number of states after expansion: %i",planner_data_->numVertices());

                    }
                }
                // Check if start vertex is still okay
                if(planner_data_->isStartVertex(vertex_id))
                {

                    ROS_INFO("Start vertex became invalid");
                    // Exiting
                    exit(-1);
                }

                ROS_INFO("Removed solution vertex_tag = %d", vtx.getTag());
                ROS_INFO("Vertex part of solution, calculate new");

                //// PLANNAR_DATA UPDATE exploration states only updated after next functions are called! ALSO ASSIGN NEW TAGS TO SOLUTION
                //// ONLY REMOVES VERTICES NOT STATES YET!
                std::cout<<"No of vertices: "<<planner_data_->numVertices();
                //planner_data_->removeVertex(vtx);
                std::cout<<"No of vertices after removeVertex: "<<planner_data_->numVertices();
                planner_data_->getSpaceInformation()->freeState(const_cast<ob::State*>(vi));
                std::cout<<"No of vertices after freeState: "<<planner_data_->numVertices();
                solution_flag = false;
                ROS_INFO("Number of vertices %d", planner_data_->numVertices());


                while (!solution_flag)
                {
                    ob::PlannerStatus solved = simple_setup_->solve( 10.0 );
                    if (solved)
                    {
                        ROS_INFO("New Solution Found");
                        // Get information about the exploration data structure the motion planner used. Used later in visualizing
                        planner_data_.reset( new ob::PlannerData( simple_setup_->getSpaceInformation() ) );
                        simple_setup_->getPlannerData( *planner_data_ );

                        //TODO make this an function of its own: void newSolution(coordinates);

                        // Setup
                        solution_flag = true;
                    }
                    else
                    {
                        static double roadmap_build_time = 0.0;
                        roadmap_build_time += 0.2;
                        ROS_INFO("No new solution found, growing for %f and expanding for %f [s]",2*roadmap_build_time,roadmap_build_time);

                        prm->growRoadmap(2*roadmap_build_time);
                        prm->expandRoadmap(roadmap_build_time);
                    }
                }
                ROS_INFO("REPLAN done info gained, no vertices: %d", planner_data_->numVertices() );
                break;
            }
            else // JUST REMOVE THE VERTEX AND CONTINUE
            {
                ROS_INFO("JUST REMOVE");
                planner_data_->removeVertex(vertex_id);
                vertex_id--;
            }
            // Update Rviz Viewer, when a vertex is removed
            // Samples and roadmap need to be updated because also edges are removed

        }
    }
}
/*
void TaskSpaceRoadmap::updatePath()
{
    // Retrieve the solution path
    //const ob::PathPtr path = prm->getProblemDefinition()->getSolutionPath();
    //og::PathGeometric* gPath = static_cast<og::PathGeometric*>(path.get());
    og::PathGeometric path = simple_setup_->getSolutionPath();

    if ( path.check()){
        status = 1;
    }
    else
    {
        //prm->getProblemDefinition()->clearSolutionPaths();
        static double roadmap_build_time = 0.0;



        roadmap_build_time += 0.1;
        ROS_INFO("No new solution found, growing roadmap for %f and expanding for %f [s]",2*roadmap_build_time,roadmap_build_time);

        //prm->growRoadmap(2*roadmap_build_time);
        //prm->expandRoadmap(roadmap_build_time);
        simple_setup_->getProblemDefinition()->clearSolutionPaths();
        ROS_INFO("Has solution, %i, %i",simple_setup_->getProblemDefinition()->hasSolution(),prm->getProblemDefinition()->hasSolution());

        ob::PlannerStatus replanned = simple_setup_->solve(1);




        if (replanned){
            status = 3;
            ROS_INFO("No vertices: %i and no edges: %i. ", planner_data_->numVertices(), planner_data_->numEdges());
            planner_data_.reset( new ob::PlannerData( simple_setup_->getSpaceInformation() ) );
            ROS_INFO("Has solution, %i",simple_setup_->getProblemDefinition()->hasSolution());
            ROS_INFO("No vertices: %i and no edges: %i. ", planner_data_->numVertices(), planner_data_->numEdges());
            simple_setup_->getPlannerData( *planner_data_ );

            ROS_INFO("Has solution, %i",simple_setup_->getProblemDefinition()->hasSolution());
            ROS_INFO("No vertices: %i and no edges: %i. ", planner_data_->numVertices(), planner_data_->numEdges());
            ROS_INFO("Replanning succeeded!");
            ROS_INFO("Path length for simple setup: %f, solution count: %i ", simple_setup_->getSolutionPath().length(),simple_setup_->getProblemDefinition()->getSolutionCount());
        }
        else {
            status = 2;
            ROS_INFO("No valid path found after resampling");
            exit;
        }
    }


    // Check if a better path is available
    // Based on what? time, energy, path length?

}
*/

