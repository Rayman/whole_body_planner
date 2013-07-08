/*!
 * \author Janno Lunenburg
 * \date April, 2013
 * \version 0.1
 */

// Inspired by: http://www.cplusplus.com/forum/general/371/

#ifndef CONNECTIVITYGRAPH_H_
#define CONNECTIVITYGRAPH_H_

#include <vector>
//#include <stack>
#include <string>
#include <iostream>

/// Arm-navigation_msgs
#include <arm_navigation_msgs/PositionConstraint.h>
#include <arm_navigation_msgs/OrientationConstraint.h>

#include <amigo_whole_body_controller/ArmTaskGoal.h>

/// enum for the status of a node
enum Status {
    NOT_VISITED,
    VISITED
};

/// Forward declaration
class Node;

/// An object of this class represents an edge in the graph.
class Edge
{
private:
    /**
      * The originating vertex
      */
    Node *orgNode_;

    /**
      * The destination vertex
      */
    Node *dstNode_;

    /**
      * Cost of the edge
      */
    unsigned int cost_;

public:

    /**
      * Constructor
      */
    Edge(Node *firstNode, Node *secNode, unsigned int inCost);

    /**
      * Returns destination node
      */
    Node* getDstNode();

    /**
      * Returns the originating node
      */
    Node* getOrgNode();

    /**
      * Returns cost of this node
      */
    unsigned int getCost();

};

/// An object of this class holds a vertex of the graph
class Node
{
private:

    /**
      * Indicates name of this node, so has a semantic meaning
      */
    std::string name_;
    std::vector<Edge> adjNodeList_;//list of outgoing edges for this vertex
    enum Status status_;//used in dfs to mark the node visited

    /**
      * Tentative cost
      */
    unsigned int tentativeCost_;

    /**
      * Possible 'parent' node
      */
    Node* tentativeParentNode_;

public:

    /**
      * Constructor
      * @param id: name of this node
      */
    Node(std::string id);

    /**
      * Deconstructor
      */
    ~Node();

    /**
      * Returns status enum
      */
    enum Status getStatus();

    /**
      * Sets status
      * @param st: status (NOT_VISITED, VISITED)
      */
    void setStatus(enum Status st);

    /**
      * Returns name of this node
      */
    std::string getName();

    /**
      * Create an edge with 'this' as the originating node and adj as the destination node
      * @param adj: destination node
      * @param cost: cost of the newly created edge
      */
    void addAdjNode(Node *adj, unsigned int cost);

    /**
      * Returns vector of edges to adjoining nodes
      */
    std::vector<Edge> getAdjNodeList();

    /**
      * Displays all adjacent verticies of this vertex
      */
    void displayList();

    /**
      * Sets tentativeCost
      * @param cost: tentative cost
      */
    void setTentativeCost(unsigned int cost);

    /**
      * Returns tentative cost
      */
    unsigned int getTentativeCost();

    /**
      * Set tentative parent
      */
    void setTentativeParentNode(Node* tentativeParent);

    /**
      * Returns tentative parent node
      */
    Node* getTentativeParentNode();

    /**
      * Indicates position constraint
      */
    //ToDo: include stiffness
    //ToDo: public not nice?
    arm_navigation_msgs::PositionConstraint position_constraint_;

    /**
      * Indicates orientation contraint
      */
    arm_navigation_msgs::OrientationConstraint orientation_contraint_;

    /**
      * Sets position constraint
      */
    void setPositionConstraint(const arm_navigation_msgs::PositionConstraint& new_position_contraint);

    /**
      * Vector containing DoFs which are constant, independent of the target
      */
    std::vector<bool> staticDofs_;

};

//An object of class graph holds a directed graph
class Graph
{
private:
    /**
      * List of vertices
      */
    std::vector<Node*> nodeList_;

    /**
      * Sets all nodes to NOT_VISITED and sets tentativeCost to infinity
      */
    void clearVisited();

    /**
      * Adds new nodes to the graph
      * @param nNode: node to add
      */
    void addNewNode(Node *nNode);

    Node* findNodeByName(std::string name);

    /**
      * Initializes graph
      */
    bool initializeGraph();

    /**
      * Vector containing the constraints of the plan
      */
    std::vector<amigo_whole_body_controller::ArmTaskGoal> constraints_;

public:
    /**
      * Contructor
      */
    Graph();

    /**
      * Deconstructor
      */
    ~Graph();

    /**
      * Creates the graph
      * Is now hardcoded, should be created otherwise
      * Possibly create upon initializion
      * Parameterize upon goal callback
      */
    void createGraph(const amigo_whole_body_controller::ArmTaskGoal& goal_constraint);

    /**
      * Displays all vertices of this graph
      */
    void displayGraph();

    /**
      * Computes a path through the graph
      * @param startPosition: name of the start-node
      * @param endPosition: name of the destination-node
      */
    std::vector<amigo_whole_body_controller::ArmTaskGoal>& getPlan(const std::string startPosition, const std::string endPosition);

};

#endif
