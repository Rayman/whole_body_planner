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
    s->setNrImproveAttempts(clearance_attempts);
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
    return true;
}

TaskSpaceRoadmap::TaskSpaceRoadmap() : octomap_(NULL)
{
    //ROS_INFO_STREAM( "OMPL version: " << OMPL_VERSION );
    ROS_INFO("Initializing plannerglobal");

    // Load parameters
    n_.param<double> ("/whole_body_planner/task_space_roadmap/validity_checking_resolution", validity_checking_resolution, 0.005);
    n_.param<double> ("/whole_body_planner/task_space_roadmap/solution_time", solution_time, 1.0);
    n_.param<double> ("/whole_body_planner/task_space_roadmap/simplification_time", simplification_time, 0.001);
    n_.param<double> ("/whole_body_planner/task_space_roadmap/octomap_resolution", octomap_resolution, 0.05);
    n_.param<int> ("/whole_body_planner/task_space_roadmap/clearance_attempts", clearance_attempts, 10);

    // Octomap
    octomap_ = new octomap::OcTreeStamped(octomap_resolution);

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

    // The interval in which obstacles are checked for between states
    simple_setup_->getSpaceInformation()->setStateValidityCheckingResolution(validity_checking_resolution); //0.005
    ROS_INFO("Initialized plannerglobal");

}

TaskSpaceRoadmap::~TaskSpaceRoadmap()
{
    ROS_INFO("Shutting down plannerglobal");
    octomap_sub.shutdown();
    delete octomap_;
}

bool TaskSpaceRoadmap::plan(const amigo_whole_body_controller::ArmTaskGoal &goal_constraint, const KDL::Frame &start_pose, const KDL::Frame &base_pose)
{
    // Save goal_constraint
    goal_constraint_ = goal_constraint;

    ROS_INFO("Creating roadmap");

    // Set the bounds
    setBounds(simple_setup_->getStateSpace(), start_pose, base_pose);

    if (!prm->milestoneCount()==0)
    {
        // Roadmap already exists, only deleting start and goal states.
        prm->clearQuery();
        prm->getProblemDefinition()->clearSolutionPaths();
        prm->getProblemDefinition()->clearGoal();
        prm->getProblemDefinition()->clearStartStates();

        //prm->clear();
    }

    // Set the start and goal states
    ob::ScopedState<> start(simple_setup_->getStateSpace());

    start[0] = start_pose.p.x();
    start[1] = start_pose.p.y();
    start[2] = start_pose.p.z();

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

    simple_setup_->setStartAndGoalStates(start, goal);

    return solve();
}

bool TaskSpaceRoadmap::replan(const KDL::Frame& start_pose)
{
    // Set new sampling method
    simple_setup_->clear();
    // Set Sampling method {VisibilityBased, ObstacleBased, Uniformly, MaximumClearance}
    sampler_ = VisibilityBased;
    simple_setup_->getSpaceInformation()->setValidStateSamplerAllocator(boost::bind(&TaskSpaceRoadmap::allocValidStateSampler, this, _1));
    //simple_setup_->getSpaceInformation()->setValidStateSamplerAllocator(boost::bind(&TaskSpaceRoadmap::allocMaximizeClearanceStateSampler, this, _1));

    // Set the new start state, goal has not changed
    ob::ScopedState<> start(simple_setup_->getStateSpace());

    start[0] = start_pose.p.x();
    start[1] = start_pose.p.y();
    start[2] = start_pose.p.z();

    simple_setup_->setStartState(start);

    return solve();
}

bool TaskSpaceRoadmap::solve(){

    // Auto setup parameters
    simple_setup_->setup();

    // Solve
    ob::PlannerStatus solved = simple_setup_->solve( solution_time );

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

std::vector<std::vector<double> > TaskSpaceRoadmap::convertSolutionToVector()
{

    // Get the found solution
    og::PathGeometric path = simple_setup_->getSolutionPath();
    const std::vector<ob::State*>& states = path.getStates();

    // Setup matrix to return results
    std::vector<std::vector<double> > coordinates;

    // Setup vector to return coordinates
    std::vector<double> coordinate(DIMENSIONS);

    // Transfer ompl states to coordinates
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

        coordinates.push_back(coordinate);
    }
    return coordinates;
}

std::vector<amigo_whole_body_controller::ArmTaskGoal> TaskSpaceRoadmap::convertSolutionToArmTaskGoal()
{
    // Get the found solution
    og::PathGeometric path = simple_setup_->getSolutionPath();
    const std::vector<ob::State*>& states = path.getStates();

    // Setup vector to return results
    std::vector<amigo_whole_body_controller::ArmTaskGoal> constraints;

    // Constraint with same link and root frames
    amigo_whole_body_controller::ArmTaskGoal constraint;
    constraint.goal_type = goal_constraint_.goal_type;
    constraint.position_constraint.link_name = goal_constraint_.position_constraint.link_name;
    constraint.position_constraint.header.frame_id = goal_constraint_.position_constraint.header.frame_id;

    // Transfer ompl states to constraints
    for( int state_id = 1; state_id < int ( states.size() ); ++state_id )
    {
        const ob::State *state = states[ state_id ];
        if (!state)
            continue; // no data?

        // Convert ompl to RealVectorStateSpace
        const ob::RealVectorStateSpace::StateType *real_state =
                static_cast<const ob::RealVectorStateSpace::StateType*>(state);

        constraint.position_constraint.position.x = real_state->values[0];
        constraint.position_constraint.position.y = real_state->values[1];
        constraint.position_constraint.position.z = real_state->values[2];
        constraints.push_back(constraint);
    }
    return constraints;
}

ob::StateSpacePtr TaskSpaceRoadmap::constructSpace(const unsigned int dimension)
{
    ROS_INFO("plannerglobal plans in %d dimension space, STILL HARDCODED", dimension);
    // Construct the SE3 space to set bounds of the planning domain, but only actually plan in R^3
    ob::StateSpacePtr se3_space( new ob::SE3StateSpace());


    ob::StateSpacePtr space( new ob::RealVectorStateSpace( DIMENSIONS ));
    return space;
}
void TaskSpaceRoadmap::setBounds(ob::StateSpacePtr space, const KDL::Frame& start_pose, const KDL::Frame& base_pose)
{
    ob::RealVectorBounds bounds( DIMENSIONS );

    // Load parameters for planning w.r.t. root frame
    double x, x_min, x_max, y, y_min, y_max, z, z_min, z_max;
    n_.param<double> ("/whole_body_planner/task_space_roadmap/planning_domain/x_max", x_max, 0.8);
    n_.param<double> ("/whole_body_planner/task_space_roadmap/planning_domain/y_max", y_max, 0.8);
    n_.param<double> ("/whole_body_planner/task_space_roadmap/planning_domain/z_max", z_max, 1.25);
    n_.param<double> ("/whole_body_planner/task_space_roadmap/planning_domain/x_min", x_min, 0.0);
    n_.param<double> ("/whole_body_planner/task_space_roadmap/planning_domain/y_min", y_min, 0.0);
    n_.param<double> ("/whole_body_planner/task_space_roadmap/planning_domain/z_min", z_min, 0.05);

    // Check which side we are planning for
    if (goal_constraint_.position_constraint.link_name.find("right") != std::string::npos){
        y_min = -y_min;
        y_max = -y_max;
        std::swap(y_min,y_max);
    }

    // Corners of box
    KDL::Frame bbx__1, bbx__2, bbx__3, bbx__4;
    bbx__1.p.x(x_max);    bbx__1.p.y(y_max);    bbx__1.p.z(z_max);
    bbx__2.p.x(x_min);    bbx__2.p.y(y_min);    bbx__2.p.z(z_min);
    bbx__3.p.x(x_max);    bbx__3.p.y(y_min);    bbx__3.p.z(z_max);
    bbx__4.p.x(x_min);    bbx__4.p.y(y_max);    bbx__4.p.z(z_min);

    // Transform box to map
    KDL::Frame bbx_1 = base_pose*bbx__1;
    KDL::Frame bbx_2 = base_pose*bbx__2;
    KDL::Frame bbx_3 = base_pose*bbx__3;
    KDL::Frame bbx_4 = base_pose*bbx__4;

    // Max
    x_max = bbx_1.p.x();
    y_max = bbx_1.p.y();
    if (bbx_2.p.x() > x_max){
        x_max = bbx_2.p.x();
    }
    if (bbx_3.p.x() > x_max){
        x_max = bbx_3.p.x();
    }
    if (bbx_4.p.x() > x_max){
        x_max = bbx_4.p.x();
    }

    if (bbx_2.p.y() > y_max){
        y_max = bbx_2.p.y();
    }
    if (bbx_3.p.y() > y_max){
        y_max = bbx_3.p.y();
    }
    if (bbx_4.p.y() > y_max){
        y_max = bbx_4.p.y();
    }

    // Min
    x_min = bbx_1.p.x();
    y_min = bbx_1.p.y();
    if (bbx_2.p.x() < x_min){
        x_min = bbx_2.p.x();
    }
    if (bbx_3.p.x() < x_min){
        x_min = bbx_3.p.x();
    }
    if (bbx_4.p.x() < x_min){
        x_min = bbx_4.p.x();
    }

    if (bbx_2.p.y() < y_min){
        y_min = bbx_2.p.y();
    }
    if (bbx_3.p.y() < y_min){
        y_min = bbx_3.p.y();
    }
    if (bbx_4.p.y() < y_min){
        y_min = bbx_4.p.y();
    }

    if (octomap_->size() > 0){

        // Mininimum
        octomap_->getMetricMin(x,y,z);
        ROS_INFO("Octomap minimum dimension in map frame (xyz): %f %f %f", x, y, z);
        if (x > x_min){
            bounds.setLow(0,x);
            std::cout<<"X_min = "<<x<<std::endl;
        }
        else {
            bounds.setLow(0,x_min);
            std::cout<<"bX_min = "<<x_min<<std::endl;
        }
        if (y > y_min){
            bounds.setLow(1,y);
            std::cout<<"Y_min = "<<y<<std::endl;
        }
        else{
            bounds.setLow(1,y_min);
            std::cout<<"bY_min = "<<y_min<<std::endl;
        }
        if (z > z_min){
            bounds.setLow(2,z);
        }
        else{
            bounds.setLow(2,z_min);
        }

        // Maximum
        octomap_->getMetricMax(x,y,z);
        ROS_INFO("Octomap maximum dimension in map frame (xyz): %f %f %f", x, y, z);
        if (x < x_max){
            bounds.setHigh(0,x);
            std::cout<<"X_max = "<<x<<std::endl;
        }
        else{
            bounds.setHigh(0,x_max);
            std::cout<<"bX_max = "<<x_max<<std::endl;
        }
        if (y < y_max){
            bounds.setHigh(1,y);
            std::cout<<"Y_max = "<<y<<std::endl;
        }
        else{
            bounds.setHigh(1,y_max);
            std::cout<<"bY_max = "<<y_max<<std::endl;
        }
        if (z < z_max){
            bounds.setHigh(2,z);
        }
        else{
            bounds.setHigh(2,z_max);
        }
    }
    else {
        ROS_WARN("Octomap has not been loaded, every sample is considered valid!");
        bounds.setLow(0,x_min);
        bounds.setLow(1,y_min);
        bounds.setLow(2,z_min);
        bounds.setHigh(0,x_max);
        bounds.setHigh(1,y_max);
        bounds.setHigh(2,z_max);
    }
    space->as<ob::RealVectorStateSpace>()->setBounds( bounds );
}

void TaskSpaceRoadmap::setOctoMap(octomap::OcTreeStamped* octree)
{
   // delete octomap_;
    octomap_ = octree;
    simple_setup_->getSpaceInformation()->setMotionValidator(ompl::base::MotionValidatorPtr(new BoundingBoxMotionValidator(simple_setup_->getSpaceInformation(), octomap_)));
}

std::vector<std::vector<double> > TaskSpaceRoadmap::shortCutPlanToVector()
{
    simple_setup_->getPathSimplifier()->shortcutPath(simple_setup_->getSolutionPath());
    std::vector<std::vector<double> > coordinates;
    return coordinates = convertSolutionToVector();
}

std::vector<amigo_whole_body_controller::ArmTaskGoal> TaskSpaceRoadmap::shortCutPlan()
{
    simple_setup_->getPathSimplifier()->shortcutPath(simple_setup_->getSolutionPath(),0, 0, 1.0, 0.01);
    std::vector<amigo_whole_body_controller::ArmTaskGoal> constraints_shortcut;
    return constraints_shortcut = convertSolutionToArmTaskGoal();
}

std::vector<std::vector<double> > TaskSpaceRoadmap::smoothPlanToVector()
{
    simple_setup_->getPathSimplifier()->smoothBSpline(simple_setup_->getSolutionPath(), 1);
    std::vector<std::vector<double> > coordinates;
    return coordinates = convertSolutionToVector();
}

std::vector<amigo_whole_body_controller::ArmTaskGoal> TaskSpaceRoadmap::smoothPlan()
{
    simple_setup_->getPathSimplifier()->smoothBSpline(simple_setup_->getSolutionPath());
    std::vector<amigo_whole_body_controller::ArmTaskGoal> constraints_smoothed;
    return constraints_smoothed= convertSolutionToArmTaskGoal();
}

std::vector<std::vector<double> > TaskSpaceRoadmap::simplifyPlanToVector()
{
    simple_setup_->simplifySolution(simplification_time);
    std::vector<std::vector<double> > coordinates;
    return coordinates = convertSolutionToVector();
}

std::vector<amigo_whole_body_controller::ArmTaskGoal> TaskSpaceRoadmap::simplifyPlan()
{
    simple_setup_->simplifySolution(simplification_time);
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
