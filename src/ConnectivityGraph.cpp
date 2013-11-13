#include "whole_body_planner/ConnectivityGraph.h"
#include <limits.h>

/// To parse yaml files
#include <ros/ros.h>
#include <XmlRpcException.h>

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
    staticDofs_.assign(6,false);
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
    //XmlRpc::XmlRpcValue::ValueArray nodes;
    ROS_WARN("Importing nodes, namespace hardcoded!");
    ns = "/whole_body_planner";
    n.getParam(ns+"/nodes", nodes);

    std::cout << "Nodes: " << nodes << std::endl;
    std::cout << nodes[0] << std::endl;

    /// Create nodes
    for (int i = 0; i < nodes.size(); i++)
    {
        XmlRpc::XmlRpcValue XmlNode = nodes[i];
        std::cout << XmlNode << std::endl;
        std::string nodeName = static_cast<std::string>(XmlNode["name"]);
        Node newNode(nodeName);
        std::string frameID = static_cast<std::string>(XmlNode["frame_id"]);
        newNode.position_constraint_.header.frame_id = frameID;

        /// Static DoFs
        // ToDo: make nice
        XmlRpc::XmlRpcValue staticDofs = XmlNode["absCoords"];
        /*for (int j = 0; j < staticDofs.size(); j++)
        {
            XmlRpc::XmlRpcValue staticDof = staticDofs[j];
            std::cout << "Static DoF: " << staticDof << std::endl;
            //std::cout << "Static DoF key: " << staticDof.hasMember(first() << std::endl;

        }*/
        if (staticDofs.hasMember("x")) {
            staticDofs[0] = true;
            newNode.position_constraint_.position.x = staticDofs["x"];
        }
        if (staticDofs.hasMember("y")) {
            staticDofs[1] = true;
            newNode.position_constraint_.position.y = staticDofs["y"];
        }
        if (staticDofs.hasMember("z")) {
            staticDofs[2] = true;
            newNode.position_constraint_.position.z = staticDofs["z"];
        }
        if (staticDofs.hasMember("roll")) {
            staticDofs[3] = true;
            newNode.position_constraint_.position.x = staticDofs["roll"];
        }
        if (staticDofs.hasMember("pitch")) {
            staticDofs[4] = true;
            newNode.position_constraint_.position.y = staticDofs["pitch"];
        }
        if (staticDofs.hasMember("yaw")) {
            staticDofs[6] = true;
            newNode.position_constraint_.position.z = staticDofs["yaw"];
        }
        /*try
        {
        for (std::map<std::string, XmlRpc::XmlRpcValue>::iterator iter = staticDofs.begin(); iter != staticDofs.end(); ++iter) {
            std::cout << "Static DoF: " << std::endl;
        }
        }
        catch (XmlRpc::XmlRpcException ex)
        {
            ROS_ERROR("%s",ex.getMessage().c_str());
        }*/
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

void Graph::createGraph(const amigo_whole_body_controller::ArmTaskGoal &goal_constraint)
{
    ROS_INFO("Creating graph");
    /// ToDo: don't hardcode
    /// ToDo: include orientation

    tip_frame_ = goal_constraint.position_constraint.link_name;

    /// Nodes
    // Carrying pose
    findNodeByName("carrying_pose")->position_constraint_.position.x = 0.265;
    findNodeByName("carrying_pose")->position_constraint_.position.y = 0.2;
    findNodeByName("carrying_pose")->position_constraint_.position.z = 0.8;

    // Pre grasp
    findNodeByName("pre_grasp")->position_constraint_.position.x = 0.4;
    findNodeByName("pre_grasp")->position_constraint_.position.y = 0.2;
    findNodeByName("pre_grasp")->position_constraint_.position.z = 0.8;

    // Grasp
    findNodeByName("grasp")->position_constraint_.position.x = goal_constraint.position_constraint.position.x;
    findNodeByName("grasp")->position_constraint_.position.y = goal_constraint.position_constraint.position.y;
    findNodeByName("grasp")->position_constraint_.position.z = goal_constraint.position_constraint.position.z;

    // Lift
    findNodeByName("lift")->position_constraint_.position.x = goal_constraint.position_constraint.position.x;
    findNodeByName("lift")->position_constraint_.position.y = goal_constraint.position_constraint.position.y;
    findNodeByName("lift")->position_constraint_.position.z = goal_constraint.position_constraint.position.z + 0.1;

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
    cp.position_constraint.position.x = insertNode->position_constraint_.position.x;
    cp.position_constraint.position.y = insertNode->position_constraint_.position.y;
    cp.position_constraint.position.z = insertNode->position_constraint_.position.z;
    constraints_.push_back(cp);
}
