/*
 * planner.h
 *
 *  Created on: 2016
 *      Author: martin
 */


//Route planning

#ifndef PLANNER_H_
#define PLANNER_H_

#include <deque>
#include <vector>
#include <string>

#include "waypoints.hpp"

//structs used by the path planner
//edge - node-to-node with cost
typedef struct _graphEdgeStruct {
	short to, from;
	double cost;
} GraphEdgeStruct;

//node - with a position and a set of edges
typedef struct _graphNodeStruct {
	short nodeIndex;
	vector<GraphEdgeStruct> edges;	//set of edge structs
	Waypoint *waypoint;
} GraphNodeStruct;

class Planner
{
public:
	Planner();
	~Planner();

	int FreePathPlan();

	//planning methods
	int PlanWaypointToWaypoint(const char *fromWaypoint, const char *toWaypoint);
	int PlanPointToPoint(Location fromPoint, Location toPoint);
	int PlanPointToWaypoint(Location fromPoint, const char *toWaypoint);

	bool route_plotted {false};

	//the final plan
	int planWaypointCount {0};
	deque<Location> planWaypoints;
	float totalPlanCost {0};

	//next step on the plan
	int routeWaypointIndex {0};

private:

	//the node list
	int nodeCount {0};
	vector<GraphNodeStruct> nodeList;

	//search parameters
	Location sourcePoint, targetPoint;	//positions
	int source {0};
	int target {0};					//node numbers

	//A* search stuff...

	//indexed into by node #. Contains the cost from adding GCosts[n] to
	//the heuristic cost from n to the target node. This is the vector the
	//iPQ indexes into.
	double           *FCosts = NULL;

	//indexed into by node. Contains the 'real' accumulated cost to that node
	double           *GCosts = NULL;

	//this vector contains the edges that comprise the shortest path tree -
	//a directed subtree of the graph that encapsulates the best paths from
	//every node on the SPT to the source node.
	GraphEdgeStruct		*shortestPathTree = NULL;

	//this is an indexed (by node) vector of 'parent' edges leading to nodes
	//connected to the SPT but that have not been added to the SPT yet. This is
	//a little like the stack or queue used in BST and DST searches.
	GraphEdgeStruct		*searchFrontier = NULL;

	//IPQ - Indexed Priority Queue
	//used to pick the most likely search frontier node next
	int				*IPQ = NULL;
	int				IPQCount = 0;

private:
	//private methods
	 int InitNodeList();

	//methods to initialize the DB
//	void waypoint_callback_method (Waypoint *wp);
//	void connection_callback_method (GraphEdge *conn);
//	int node_index {0};

	//planning
	int NewWaypointPath();
	int InitAstar();
	int closestNodeTo(Location posn, Location inDirectionOfTarget);	//find a node on the graph
	void doPathCycle();		//makes one iteration.
	double HeuristicCalculate(int _from, int _to);	//A-Star heuristic

	//priority queue
	void pqInsert(int n);
	int pqPop();
	void pqChangePriority(int n);

	friend void planner_waypoint_callback(Waypoint *wp, void *args);
	friend void planner_connection_callback(GraphEdge *conn, void *args);
};

#endif
