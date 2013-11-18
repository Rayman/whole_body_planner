#include "whole_body_planner/ConnectivityGraph.h"
#include <limits.h>

/// To parse yaml files
//#include <ros/ros.h>
//#include <XmlRpcException.h>
#include <tf/transform_datatypes.h>

/// To update graph
#include <kdl/frames.hpp>

#include <algorithm>

Edge::Edge(Node *firstNode, Node *secNode, unsigned inCost)
{
    orgNode_ = firstNode;
    dstNode_ = secNode;
    cost_ = inCost;
}

Node* Edge::getDstNode()
{
    return dstNode_;
}

Node* Edge::getOrgNode()
{
    return orgNode_;
}

unsigned Edge::getCost()
{
    return cost_;
}

Node::Node(std::string id)
{
    name_ = id;
    status_ = NOT_VISITED;
    dof_constraints_.assign(6,FREE);
}

Node::~Node()
{
    adjNodeList_.clear();
}

enum Status Node::getStatus()
{
    return status_;
}

void Node::setStatus(enum Status st)
{
    status_ = st;
}

std::string Node::getName()
{
    return name_;
}

void Node::addAdjNode(Node *adj, unsigned cost)
{
    //create an edge with 'this' as the originating node and adj as the destination node
    Edge newEdge(this, adj, cost);
    adjNodeList_.push_back(newEdge);
}

std::vector<Edge> Node::getAdjNodeList()
{
    return adjNodeList_;
}

void Node::displayList()
{
    std::string edgeOp = " -> " ;
    for(unsigned int i=0 ; i < adjNodeList_.size() ; i++)
    {
        Edge edg = adjNodeList_[i];
        std::cout << name_ << " -> " << edg.getDstNode()->getName() << std::endl;
    }

}

void Node::setTentativeCost(unsigned int cost)
{
    tentativeCost_ = cost;
}

void Node::setTentativeParentNode(Node* tentativeParent)
{
    tentativeParentNode_ = tentativeParent;
}

Node* Node::getTentativeParentNode()
{
    return tentativeParentNode_;
}

unsigned int Node::getTentativeCost()
{
    return tentativeCost_;
}

void Node::setPositionConstraint(const arm_navigation_msgs::PositionConstraint& new_position_contraint)
{
    position_constraint_ = new_position_contraint;
}

Graph::Graph()
{
    ROS_INFO("Initializing graph");
    nodeList_.resize(0);
    initializeGraph();
}

Graph::~Graph()
{
    //free mem allocated to verticies
    for(unsigned int i=0 ; i < nodeList_.size() ; i++)
        delete nodeList_[i];
    nodeList_.clear();
}

bool Graph::initializeGraph()
{
    /// Get nodes from parameterserver
    ros::NodeHandle n("~");
    std::string ns = ros::this_node::getName();
    XmlRpc::XmlRpcValue nodes;
    ROS_WARN("Importing nodes, namespace hardcoded!");
    ns = "/whole_body_planner";
    n.getParam(ns+"/nodes", nodes);

    //std::cout << "Nodes: " << nodes << std::endl;
    //std::cout << nodes[0] << std::endl;
    ROS_INFO("Graph contains %i nodes", (int)nodes.size());

    /// Create nodes
    for (int i = 0; i < nodes.size(); i++)
    {
        XmlRpc::XmlRpcValue XmlNode = nodes[i];
        //std::cout << XmlNode << std::endl;
        std::string nodeName = static_cast<std::string>(XmlNode["name"]);
        Node newNode(nodeName);
        std::string frameID = static_cast<std::string>(XmlNode["frame_id"]);
        newNode.position_constraint_.header.frame_id = frameID;
        ROS_INFO("Node %s", frameID.c_str());

        /// Determine whether DoFs are free, statically constrained or constrained w.r.t. the goal
        XmlRpc::XmlRpcValue DofConstraints = XmlNode["DofConstraints"];
        determineDofConstraint(newNode, DofConstraints);

        /// Static DoFs
        XmlRpc::XmlRpcValue staticDofs = XmlNode["absCoords"];
        assignStaticDofs(newNode, staticDofs);

        /// Dynamic DoFs
        XmlRpc::XmlRpcValue dynamicDofs = XmlNode["relCoords"];
        assignDynamicDofs(newNode, dynamicDofs);

        /// Target offsets
        XmlRpc::XmlRpcValue target_offsets = XmlNode["offsets"];
        assignTargetOffsets(newNode, target_offsets);

        /// Position tolerance
        XmlRpc::XmlRpcValue position_tolerance = XmlNode["positionTolerance"];
        assignPositionTolerance(newNode, position_tolerance);

        /// Orientation tolerance
        XmlRpc::XmlRpcValue orientation_tolerance = XmlNode["orientationTolerance"];
        assignOrientationTolerance(newNode, orientation_tolerance);

        /// Stiffness
        XmlRpc::XmlRpcValue stiffness = XmlNode["stiffness"];
        assignStiffness(newNode, stiffness);

        /// Push node back into nodelist
        nodeList_.push_back(new Node(newNode));

    }

    /// Create edges
    for (int i = 0; i < nodes.size(); i++)
    {
        XmlRpc::XmlRpcValue XmlNode = nodes[i];
        std::string orgName = static_cast<std::string>(XmlNode["name"]);

        XmlRpc::XmlRpcValue dstNodes = XmlNode["dstNodes"];
        for (int j = 0; j < dstNodes.size(); j++)
        {
            XmlRpc::XmlRpcValue dstNode = dstNodes[j];
            std::string dstName = static_cast<std::string>(dstNode);
            findNodeByName(orgName)->addAdjNode(findNodeByName(dstName), 1);
        }
    }
    /*
    //typedef std::vector<XmlRpc::XmlRpcValue>::iterator XmlRpcIterator;
    typedef std::map<std::string, XmlRpc::XmlRpcValue>::iterator XmlRpcIterator;

    try
    {
        for (XmlRpcIterator iter = nodes.begin(); iter != nodes.end(); ++iter)
        {
            ROS_INFO("Importing node");
            //std::String name;
            //name = static_cast<std::string>(value["name"]);

        }
    }
    catch (XmlRpc::XmlRpcException ex)
    {
        ROS_ERROR("%s",ex.getMessage().c_str());
    }
*/
    return true;
}

void Graph::determineDofConstraint(Node& node, XmlRpc::XmlRpcValue& dof_constraints)
{
    if (dof_constraints.hasMember("x")) {
        ROS_INFO("Assigning x");
        node.dof_constraints_[0] = setDofContraint(static_cast<std::string>(dof_constraints["x"]));
        ROS_INFO("Assigned x");
    } else {
        ROS_ERROR("Constraint of DoF x not defined");
    }
    if (dof_constraints.hasMember("y")) {
        node.dof_constraints_[1] = setDofContraint(static_cast<std::string>(dof_constraints["y"]));
    } else {
        ROS_ERROR("Constraint of DoF y not defined");
    }
    if (dof_constraints.hasMember("z")) {
        node.dof_constraints_[2] = setDofContraint(static_cast<std::string>(dof_constraints["z"]));
    } else {
        ROS_ERROR("Constraint of DoF z not defined");
    }
    if (dof_constraints.hasMember("roll")) {
        node.dof_constraints_[3] = setDofContraint(static_cast<std::string>(dof_constraints["roll"]));
    } else {
        ROS_ERROR("Constraint of DoF roll not defined");
    }
    if (dof_constraints.hasMember("pitch")) {
        node.dof_constraints_[4] = setDofContraint(static_cast<std::string>(dof_constraints["pitch"]));
    } else {
        ROS_ERROR("Constraint of DoF pitch not defined");
    }
    if (dof_constraints.hasMember("yaw")) {
        node.dof_constraints_[5] = setDofContraint(static_cast<std::string>(dof_constraints["yaw"]));
    } else {
        ROS_ERROR("Constraint of DoF yaw not defined");
    }
    ////////// TESTSTUFF
    std::vector<std::string> test;
    test.resize(6);
    for (unsigned int j = 0; j < test.size(); j++) {
        if (node.dof_constraints_[j] == FREE) {
            test[j] = "free";
        } else if (node.dof_constraints_[j] == STATIC) {
            test[j] = "static";
        } else if (node.dof_constraints_[j] == DYNAMIC) {
            test[j] = "dynamic";
        } else {
            test[j] = "error";
        }
    }
    ROS_INFO("Node: [%s, %s, %s, %s, %s, %s]", test[0].c_str(), test[1].c_str(), test[2].c_str(), test[3].c_str(), test[4].c_str(), test[5].c_str());
    //////////
}

void Graph::assignStaticDofs(Node& node, XmlRpc::XmlRpcValue& static_dofs)
{
    /// Position
    if (static_dofs.hasMember("x")) {
        node.position_constraint_.position.x = static_cast<double>(static_dofs["x"]);
    }
    if (static_dofs.hasMember("y")) {
        node.position_constraint_.position.y = static_cast<double>(static_dofs["y"]);
    }
    if (static_dofs.hasMember("z")) {
        node.position_constraint_.position.z = static_cast<double>(static_dofs["z"]);
    }

    /// Orientation: roll, pitch, yaw
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    if (static_dofs.hasMember("roll")) {
        roll = static_cast<double>(static_dofs["roll"]);
    }
    if (static_dofs.hasMember("pitch")) {
        pitch = static_cast<double>(static_dofs["pitch"]);
    }
    if (static_dofs.hasMember("yaw")) {
        yaw = static_cast<double>(static_dofs["yaw"]);
    }

    /// Orientation: quaternion
    node.orientation_constraint_.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
}

void Graph::assignDynamicDofs(Node& node, XmlRpc::XmlRpcValue& dynamic_dofs)
{
    // ToDo: make more intelligent?
    if (dynamic_dofs.hasMember("x") && node.dof_constraints_[0] == DYNAMIC) {
        node.relative_coordinates_.linear.x = static_cast<double>(dynamic_dofs["x"]);
    } else {
        node.relative_coordinates_.linear.x = 0.0;
    }
    if (dynamic_dofs.hasMember("y") && node.dof_constraints_[1] == DYNAMIC) {
        node.relative_coordinates_.linear.y = static_cast<double>(dynamic_dofs["y"]);
    } else {
        node.relative_coordinates_.linear.y = 0.0;
    }
    if (dynamic_dofs.hasMember("z") && node.dof_constraints_[2] == DYNAMIC) {
        node.relative_coordinates_.linear.z = static_cast<double>(dynamic_dofs["z"]);
    } else {
        node.relative_coordinates_.linear.z = 0.0;
    }
    if (dynamic_dofs.hasMember("roll") && node.dof_constraints_[3] == DYNAMIC) {
        node.relative_coordinates_.angular.x = static_cast<double>(dynamic_dofs["roll"]);
    } else {
        node.relative_coordinates_.angular.x = 0.0;
    }
    if (dynamic_dofs.hasMember("pitch") && node.dof_constraints_[4] == DYNAMIC) {
        node.relative_coordinates_.angular.y = static_cast<double>(dynamic_dofs["pitch"]);
    } else {
        node.relative_coordinates_.angular.y = 0.0;
    }
    if (dynamic_dofs.hasMember("yaw") && node.dof_constraints_[5] == DYNAMIC) {
        node.relative_coordinates_.angular.z = static_cast<double>(dynamic_dofs["yaw"]);
    } else {
        node.relative_coordinates_.angular.z = 0.0;
    }

}

void Graph::assignTargetOffsets(Node& node, XmlRpc::XmlRpcValue& target_offsets)
{
    if (target_offsets.hasMember("x")) {
        node.position_constraint_.target_point_offset.x = static_cast<double>(target_offsets["x"]);
    }
    if (target_offsets.hasMember("y")) {
        node.position_constraint_.target_point_offset.x = static_cast<double>(target_offsets["y"]);
    }
    if (target_offsets.hasMember("z")) {
        node.position_constraint_.target_point_offset.x = static_cast<double>(target_offsets["z"]);
    }
}

void Graph::assignPositionTolerance(Node& node, XmlRpc::XmlRpcValue& tolerance)
{
    // ToDo: make more intelligent: it means something if a DoF is left unconstrained
    if (tolerance.hasMember("length")) {
        /// Must be a cylinder
        node.position_constraint_.constraint_region_shape.type = node.position_constraint_.constraint_region_shape.CYLINDER;
        node.position_constraint_.constraint_region_shape.dimensions.resize(2);
        node.position_constraint_.constraint_region_shape.dimensions[0] = static_cast<double>(tolerance["radius"]);
        node.position_constraint_.constraint_region_shape.dimensions[1] = static_cast<double>(tolerance["length"]);
    } else if (tolerance.hasMember("radius") && tolerance.hasMember("length")) {
        /// Must be a sphere
        node.position_constraint_.constraint_region_shape.type = node.position_constraint_.constraint_region_shape.SPHERE;
        node.position_constraint_.constraint_region_shape.dimensions.resize(1);
        node.position_constraint_.constraint_region_shape.dimensions[0] = static_cast<double>(tolerance["radius"]);
    } else if (tolerance.hasMember("size_x") && tolerance.hasMember("size_y") && tolerance.hasMember("size_z")) {
        /// Must be a box
        node.position_constraint_.constraint_region_shape.type = node.position_constraint_.constraint_region_shape.BOX;
        node.position_constraint_.constraint_region_shape.dimensions.resize(3);
        node.position_constraint_.constraint_region_shape.dimensions[0] = static_cast<double>(tolerance["size_x"]);
        node.position_constraint_.constraint_region_shape.dimensions[1] = static_cast<double>(tolerance["size_y"]);
        node.position_constraint_.constraint_region_shape.dimensions[2] = static_cast<double>(tolerance["size_z"]);
    } else {
        /// Default solution
        node.position_constraint_.constraint_region_shape.type = node.position_constraint_.constraint_region_shape.SPHERE;
        node.position_constraint_.constraint_region_shape.dimensions.resize(1);
        node.position_constraint_.constraint_region_shape.dimensions[0] = 0.03;
    }
}

void Graph::assignOrientationTolerance(Node& node, XmlRpc::XmlRpcValue& tolerance)
{
    // ToDo: robustness: check if something is/isn't constrained on purpose
    if (tolerance.hasMember("roll")) {
        node.orientation_constraint_.absolute_roll_tolerance = static_cast<double>(tolerance["roll"]);
    } else {
        node.orientation_constraint_.absolute_roll_tolerance = 6.3;
    }
    if (tolerance.hasMember("pitch")) {
        node.orientation_constraint_.absolute_pitch_tolerance = static_cast<double>(tolerance["pitch"]);
    } else {
        node.orientation_constraint_.absolute_pitch_tolerance = 6.3;
    }
    if (tolerance.hasMember("yaw")) {
        node.orientation_constraint_.absolute_yaw_tolerance = static_cast<double>(tolerance["yaw"]);
    } else {
        node.orientation_constraint_.absolute_yaw_tolerance = 6.3;
    }
}

void Graph::assignStiffness(Node& node, XmlRpc::XmlRpcValue& stiffness)
{
    if (stiffness.hasMember("x") && node.dof_constraints_[0] != FREE) {
        node.stiffness_.force.x = stiffness["x"];
    } else {
        node.stiffness_.force.x = 0.0;
    }
    if (stiffness.hasMember("y") && node.dof_constraints_[1] != FREE) {
        node.stiffness_.force.y = stiffness["y"];
    } else {
        node.stiffness_.force.y = 0.0;
    }
    if (stiffness.hasMember("z") && node.dof_constraints_[2] != FREE) {
        node.stiffness_.force.z = stiffness["z"];
    } else {
        node.stiffness_.force.z = 0.0;
    }
    if (stiffness.hasMember("roll") && node.dof_constraints_[3] != FREE) {
        node.stiffness_.torque.x = stiffness["roll"];
    } else {
        node.stiffness_.torque.x = 0.0;
    }
    if (stiffness.hasMember("pitch") && node.dof_constraints_[4] != FREE) {
        node.stiffness_.torque.y = stiffness["pitch"];
    } else {
        node.stiffness_.torque.y = 0.0;
    }
    if (stiffness.hasMember("yaw") && node.dof_constraints_[5] != FREE) {
        node.stiffness_.torque.z = stiffness["yaw"];
    } else {
        node.stiffness_.torque.z = 0.0;
    }
    ROS_INFO("Assigning stiffness for %s: force: [%f, %f, %f], torque: [%f, %f, %f]", node.getName().c_str(), node.stiffness_.force.x,
                                                                                                              node.stiffness_.force.y,
                                                                                                              node.stiffness_.force.z,
                                                                                                              node.stiffness_.torque.x,
                                                                                                              node.stiffness_.torque.y,
                                                                                                              node.stiffness_.torque.z);
}

enum DofConstraint Graph::setDofContraint(const std::string& type)
{
    if (!strcmp(type.c_str(), "free")) {
        return FREE;
    } else if (!strcmp(type.c_str(), "static")) {
        return STATIC;
    } else if (!strcmp(type.c_str(), "dynamic")) {
        return DYNAMIC;
    } else {
        ROS_ERROR("DoF constraint is neither free, static or dynamic... will be left free");
        return FREE;
    }
}

void Graph::updateGraph(const amigo_whole_body_controller::ArmTaskGoal &goal_constraint)
{
    ROS_INFO("Updating graph");
    /// ToDo: don't hardcode
    /// ToDo: include orientation

    tip_frame_ = goal_constraint.position_constraint.link_name;

    /// Nodes
    for (std::vector<Node*>::iterator iter = nodeList_.begin(); iter != nodeList_.end(); ++iter) {
        //ROS_INFO("Updating node %s", (*iter)->getName().c_str());
        /// Frame ids
        (*iter)->position_constraint_.link_name = goal_constraint.position_constraint.link_name;
        (*iter)->orientation_constraint_.link_name = goal_constraint.orientation_constraint.link_name;

        /// Positions
        if ((*iter)->dof_constraints_[0] == DYNAMIC) {
            //ROS_INFO("Updating x: abs = %f, rel = %f", goal_constraint.position_constraint.position.x, (*iter)->relative_coordinates_.linear.x);
            (*iter)->position_constraint_.position.x = goal_constraint.position_constraint.position.x + (*iter)->relative_coordinates_.linear.x;
        }
        if ((*iter)->dof_constraints_[1] == DYNAMIC) {
            //ROS_INFO("Updating y: abs = %f, rel = %f", goal_constraint.position_constraint.position.y, (*iter)->relative_coordinates_.linear.y);
            (*iter)->position_constraint_.position.y = goal_constraint.position_constraint.position.y + (*iter)->relative_coordinates_.linear.y;
        }
        if ((*iter)->dof_constraints_[2] == DYNAMIC) {
            //ROS_INFO("Updating z: abs = %f, rel = %f", goal_constraint.position_constraint.position.z, (*iter)->relative_coordinates_.linear.z);
            (*iter)->position_constraint_.position.z = goal_constraint.position_constraint.position.z + (*iter)->relative_coordinates_.linear.z;
        }

        /// Orientation (update of one of these is denoted dynamic)
        // ToDo: test this!
        if ((*iter)->dof_constraints_[3] == DYNAMIC || (*iter)->dof_constraints_[4] == DYNAMIC || (*iter)->dof_constraints_[5] == DYNAMIC) {
            /// Get orientation in KDL format
            KDL::Rotation input_orientation;
            geometry_msgs::Quaternion goal_quat = goal_constraint.orientation_constraint.orientation;
            input_orientation.Quaternion(goal_quat.x, goal_quat.y, goal_quat.z, goal_quat.w);

            /// Get rotation
            KDL::Rotation relative_rotation;
            relative_rotation.RPY((*iter)->relative_coordinates_.angular.x, (*iter)->relative_coordinates_.angular.y, (*iter)->relative_coordinates_.angular.z);

            /// Multiply
            KDL::Rotation output_orientation;
            output_orientation = input_orientation * relative_rotation;

            /// Put back in constraint
            output_orientation.GetQuaternion((*iter)->orientation_constraint_.orientation.x,
                                             (*iter)->orientation_constraint_.orientation.y,
                                             (*iter)->orientation_constraint_.orientation.z,
                                             (*iter)->orientation_constraint_.orientation.w);

        }

        //ROS_INFO("x: %f, y: %f, z: %f", (*iter)->position_constraint_.position.x, (*iter)->position_constraint_.position.y, (*iter)->position_constraint_.position.z);
    }

}

void Graph::displayGraph()
{
    for(unsigned int i=0 ; i < nodeList_.size() ; i++)
    {
        nodeList_[i]->displayList();
    }
}

void Graph::addNewNode(Node *nNode)
{
    nodeList_.push_back(nNode);
}

Node* Graph::findNodeByName(std::string name)
{
    for(unsigned int i = 0 ; i < nodeList_.size() ; i++)
    {
        if(nodeList_[i]->getName() == name)
            return nodeList_[i];
    }
    return NULL;
}

void Graph::clearVisited()
{
    //for(int i = 0; i < nodeList_.size() && !foundCycle ; i++)
    for(unsigned int i = 0; i < nodeList_.size(); i++)
    {
        nodeList_[i]->setStatus(NOT_VISITED);
        nodeList_[i]->setTentativeCost(INT_MAX);
    }
}

std::vector<amigo_whole_body_controller::ArmTaskGoal>& Graph::getPlan(const std::string startPosition, const std::string endPosition)
{

    /// Start by setting all nodes to unvisited and setting the constraints vector to empty
    clearVisited();
    constraints_.clear();

    /// Retrieve startnode and set tentativeCost to 0
    Node* startNode = findNodeByName(startPosition);
    startNode->setTentativeCost(0);

    /// Create current node
    Node* currentNode = startNode;

    /// Create a vector with unvisited nodes
    // ToDo: can we do this more efficiently?
    // Suggestion: use lists instead of vectors?
    std::vector<Node*> unvisitedNodeList = nodeList_;

    /// Iterate until endnode has been visited
    while (!findNodeByName(endPosition)->getStatus() == VISITED)
    {
        ROS_DEBUG("Current node = %s, cost = %i, endPositionVisited = %d",currentNode->getName().c_str(),currentNode->getTentativeCost(),!findNodeByName(endPosition)->getStatus() == VISITED);
        for (unsigned int i = 0; i < unvisitedNodeList.size(); i++)
        {
            ROS_DEBUG("Costs of node %s = %i",unvisitedNodeList[i]->getName().c_str(),unvisitedNodeList[i]->getTentativeCost());
        }

        /// Get vector with adjacent edges
        std::vector<Edge> adjNodelist = currentNode->getAdjNodeList();

        /// Iterate through all adjacent vertices
        for (unsigned int i = 0; i < adjNodelist.size(); i++)
        {
            /// Only consider nodes that have not yet been visited
            if (adjNodelist[i].getDstNode()->getStatus() != VISITED)
            {
                ROS_DEBUG("Adjacent node = %s",adjNodelist[i].getDstNode()->getName().c_str());

                /// If the costs of the current node + edge to next node are smaller than the tentative cost of that node, update the tentative cost
                if ( (currentNode->getTentativeCost() + adjNodelist[i].getCost()) < adjNodelist[i].getDstNode()->getTentativeCost())
                {
                    /// Update cost of neighbouring nodes
                    adjNodelist[i].getDstNode()->setTentativeCost(currentNode->getTentativeCost() + adjNodelist[i].getCost());

                    /// Update tentative parent node
                    adjNodelist[i].getDstNode()->setTentativeParentNode(currentNode);
                }
            }
        }

        /// Mark current node as visited
        currentNode->setStatus(VISITED);

        /// Remove it from the univisitedNodeList
        unvisitedNodeList.erase(std::remove(unvisitedNodeList.begin(), unvisitedNodeList.end(), findNodeByName(currentNode->getName())), unvisitedNodeList.end());

        /// Determine the next node
        currentNode = unvisitedNodeList[0];
        for (unsigned int i = 1; i < unvisitedNodeList.size(); i++)
        {
            if (unvisitedNodeList[i]->getTentativeCost() < currentNode->getTentativeCost())
            {
                currentNode = unvisitedNodeList[i];
            }
        }
    }

    // ToDo: again: more efficiently

    /// Move back through plan and add constraints to the plan
    Node* insertNode = findNodeByName(endPosition);
    while (!(insertNode->getName() == startPosition))
    {
        addNodeToPlan(insertNode);
        insertNode = insertNode->getTentativeParentNode();
    }

    /// Also insert the startnode
    addNodeToPlan(insertNode);

    /// Reverse order of control points
    std::reverse(constraints_.begin(), constraints_.end());

    /// Print results
    for (unsigned int i = 0; i < constraints_.size(); i++)
    {
        ROS_INFO("Constraint(%i) = %s",i,constraints_[i].goal_type.c_str());
    }

    return constraints_;
}

void Graph::addNodeToPlan(Node* insertNode)
{
    amigo_whole_body_controller::ArmTaskGoal cp;
    cp.goal_type = insertNode->getName();
    cp.position_constraint.link_name = tip_frame_;
    cp.position_constraint.header.frame_id = insertNode->position_constraint_.header.frame_id;
    cp.position_constraint = insertNode->position_constraint_;
    cp.orientation_constraint = insertNode->orientation_constraint_;
    cp.stiffness.force.x = insertNode->stiffness_.force.x;
    cp.stiffness.force.y = insertNode->stiffness_.force.y;
    cp.stiffness.force.z = insertNode->stiffness_.force.z;
    cp.stiffness.torque.x = insertNode->stiffness_.torque.x;
    cp.stiffness.torque.y = insertNode->stiffness_.torque.y;
    cp.stiffness.torque.z = insertNode->stiffness_.torque.z;
    constraints_.push_back(cp);
}
