/*
 ============================================================================
 Name        : planner.cpp
 Author      : Martin
 Version     :
 Copyright   : (c) 2016 Martin Lane-Smith
 Description : Route plan
 ============================================================================
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <math.h>
#include <pthread.h>
#include <errno.h>

#include "planner.hpp"

#define DEBUGPRINT(...) {printf(__VA_ARGS__); printf("\n");}

Planner::Planner()
{
	InitNodeList();
}

//release all memory
Planner::~Planner()
{
	if (FCosts) free(FCosts);
	if (GCosts) free(GCosts);
	if (shortestPathTree) free(shortestPathTree);
	if (searchFrontier) free(searchFrontier);
	if (IPQ) free(IPQ);
	FreePathPlan();

	for (int i=0; i< nodeCount; i++)
	{
		//free edge structs
		nodeList[i].edges.clear();
	}
	nodeList.clear();
	nodeCount = 0;
}

int Planner::FreePathPlan()
{
	planWaypoints.clear();
	planWaypointCount = routeWaypointIndex = 0;
	return 0;
}

int Planner::InitNodeList()
{
	int i;
	//convert from the_waypoints() to an array

	//free old one, if any
	for (i=0; i< nodeCount; i++)
	{
		//free edge structs
		nodeList[i].edges.clear();
	}
	nodeList.clear();

	nodeCount = the_waypoints().waypoints_size();
	nodeList.resize(nodeCount);

	//init the structs
	for (i=0; i<nodeCount; i++)
	{
		Waypoint *current = &the_waypoints()[i];
		nodeList[i].nodeIndex = i;
		nodeList[i].waypoint = current;
		//do edges in second pass
	}
	//go through again and init edges
	for (i=0; i< nodeCount; i++)
	{
		Waypoint current = the_waypoints()[i];
		int edgeCount = current.edges_size();
		nodeList[i].edges.resize(edgeCount);

		//init edge structs

		for (int e=0; e<edgeCount; e++)
		{
			nodeList[i].edges[e].from = i;

			GraphEdge ge = current[e];
			string tName = ge.toWaypointName();
			int toNode = the_waypoints().GetWaypointIndexByName(tName);

			if (toNode >= 0)
			{
				//to found
				nodeList[i].edges[e].to = toNode;
				nodeList[i].edges[e].cost = (double) current.location().mm_range_to_point(the_waypoints()[toNode].location());

				DEBUGPRINT("planner: Edge from %s (%i) to %s (%i)", nodeList[i].waypoint->name().c_str(), i, nodeList[toNode].waypoint->name().c_str(), toNode);
			}
			else
			{
				DEBUGPRINT("planner: A* toNode %s not found", tName.c_str());
				return -1;
			}

		}
	}

	return 0;
}

int Planner::InitAstar()
{
	if (FCosts) free(FCosts);
	if (GCosts) free(GCosts);
	if (shortestPathTree) free(shortestPathTree);
	if (searchFrontier) free(searchFrontier);
	if (IPQ) free(IPQ);
	planWaypoints.clear();
    FCosts				= (double*) calloc(nodeCount, sizeof(double));
    GCosts				= (double*) calloc(nodeCount, sizeof(double));
    shortestPathTree	= (GraphEdgeStruct*) calloc(nodeCount, sizeof(GraphEdgeStruct));
    searchFrontier		= (GraphEdgeStruct*) calloc(nodeCount, sizeof(GraphEdgeStruct));
    IPQ					= (int*) calloc(nodeCount, sizeof(int));
    IPQCount = 0;

	for (int i = 0; i < nodeCount; i++){
		searchFrontier[i].to = -1;
		shortestPathTree[i].to = -1;
	}

	return 0;
}

//Public

//calls from the BT

int Planner::PlanWaypointToWaypoint(const char *fromWaypoint, const char *toWaypoint)		//named point to named point
{
	DEBUGPRINT("planner: %s to %s", fromWaypoint, toWaypoint);

	source = the_waypoints().GetWaypointIndexByName(fromWaypoint);
	if (source < 0)
	{
		DEBUGPRINT("planner: Source %s not found", fromWaypoint);
		return -1;
	}
	DEBUGPRINT("planner: Source %s is %i", fromWaypoint, source);

	target = the_waypoints().GetWaypointIndexByName(toWaypoint);

	if (target < 0)
	{
		DEBUGPRINT("planner: Target %s not found", toWaypoint);
		return -1;
	}
	DEBUGPRINT("planner: Target %s is %i", toWaypoint, target);

	return NewWaypointPath();
}

int Planner::PlanPointToPoint(Location fromPoint, Location toPoint)	//position to position
{
	DEBUGPRINT("planner: %f,%f to %f,%f", fromPoint.latitude_in_degrees(), fromPoint.longitude_in_degrees(), toPoint.latitude_in_degrees(), toPoint.longitude_in_degrees());

	source = closestNodeTo(fromPoint, toPoint);
	if (source < 0)
	{
		DEBUGPRINT("planner: No From WP found");
		return -1;
	}
	DEBUGPRINT("planner: Closest Source %s is %i", the_waypoints()[source].name().c_str(), source);

	target = closestNodeTo(toPoint, fromPoint);
	if (target < 0)
	{
		DEBUGPRINT("planner: No To WP found");
		return -1;
	}
	DEBUGPRINT("planner: Closest Target %s is %i", the_waypoints()[target].name().c_str(), target);
	return NewWaypointPath();
}

int Planner::PlanPointToWaypoint(Location fromPoint, const char *toWaypoint)		//position to named point
{
	DEBUGPRINT("planner: %f,%f to %s", fromPoint.latitude_in_degrees(), fromPoint.longitude_in_degrees(), toWaypoint);

	source = closestNodeTo(fromPoint, the_waypoints().GetWaypointByName(toWaypoint).location());
	if (source < 0)
	{
		DEBUGPRINT("planner: No From WP");
		return -1;
	}
	DEBUGPRINT("planner: Closest Source %s is %i", the_waypoints()[source].name().c_str(), source);

	target = the_waypoints().GetWaypointIndexByName(toWaypoint);
	if (target < 0)
	{
		DEBUGPRINT("planner: Target %s not found", toWaypoint);
		return -1;
	}
	DEBUGPRINT("planner: Target %s is %i", toWaypoint, target);

	return NewWaypointPath();
}

//common planning method
int Planner::NewWaypointPath()
{
	if (source < 0 || target < 0)
	{
		DEBUGPRINT("planner: Unreachable");
		return -1;
	}

	DEBUGPRINT("planner: Plan from %i to %i", source, target);

	InitAstar();

	//put the source node on the queue
	searchFrontier[source].to = source;
	FCosts[source] = GCosts[source] = 0.0f;
	pqInsert(source);

	bool searchComplete = false;

	while (1)
	{
		//uses an indexed priority queue of nodes. The nodes with the
		//lowest overall F cost (G+H) are positioned at the front.
		//while the queue is not empty
		//get lowest cost node from the queue
		int NextClosestNode = pqPop();

		//move this node/edge from the frontier to the spanning tree
		shortestPathTree[NextClosestNode] = searchFrontier[NextClosestNode];
		DEBUGPRINT("planner: NextNode = %i (from %i)", NextClosestNode, shortestPathTree[NextClosestNode].from);
		//if the target has been found exit
		if (NextClosestNode == target ) {
			searchComplete = true;
			DEBUGPRINT("planner: A* Search complete");
			break;
		}
		else
		{
			//if we've failed exit
			if (NextClosestNode < 0) {
				searchComplete = false;
				DEBUGPRINT("planner: A* Search failed");
				break;
			}
			else
			{
				//now to test all the edges attached to this node
				for (int i = 0; i < the_waypoints()[NextClosestNode].edges_size(); i++)
				{
					int newNode = nodeList[NextClosestNode].edges[i].to;

					//calculate the heuristic cost from this node to the target (H)
					double HCost = HeuristicCalculate(target, newNode);

					//calculate the 'real' cost to this node from the source (G)
					double GCost = GCosts[NextClosestNode] + nodeList[NextClosestNode].edges[i].cost;

					//if the node has not been added to the frontier, add it and update
					//the G and F costs
					if (searchFrontier[newNode].to == -1)
					{
						FCosts[newNode] = GCost + HCost;
						GCosts[newNode] = GCost;

						pqInsert(newNode);

						searchFrontier[newNode] = nodeList[NextClosestNode].edges[i];
					}

					//if this node is already on the frontier but the cost to get here
					//is cheaper than has been found previously, update the node
					//costs and frontier accordingly.
					else if ((GCost < GCosts[newNode]) && (shortestPathTree[newNode].to == -1))
					{
						FCosts[newNode] = GCost + HCost;
						GCosts[newNode] = GCost;

						pqChangePriority(newNode);

						searchFrontier[newNode] = nodeList[NextClosestNode].edges[i];
					}
				}
			}
		}
	}
	
	if (searchComplete)
	{
		//count path steps
		int nd = target;
		int pathCount = 1;

		while ((nd != source) && (shortestPathTree[nd].to != -1))
		{
			pathCount++;
			nd = shortestPathTree[nd].from;
		}
		planWaypointCount = pathCount;
		totalPlanCost = 0.0;

		nd = target;

		while ((nd != source) && (shortestPathTree[nd].to != -1))
		{
			planWaypoints.push_front(the_waypoints()[nd].location());
			totalPlanCost += shortestPathTree[nd].cost;

			nd = shortestPathTree[nd].from;
		}
		planWaypoints.push_front(the_waypoints()[source].location());

		DEBUGPRINT("planner: A* %i steps, total cost %f", planWaypointCount, totalPlanCost);

		for (int i=0; i<planWaypointCount; i++)
		{
			DEBUGPRINT("planner: %2i: %s", i+1, planWaypoints[i].name.c_str());
		}

		routeWaypointIndex 	= 0;

		return 0;
	}
	else
	{
		DEBUGPRINT("planner: A* Fail");
		return -1;
	}
}

//find a suitable start/end waypoint
int Planner::closestNodeTo(Location fromPoint, Location inDirectionOfTarget)
{
	int i;
	double bestRangeFromStart = 1000000.0;
	int node = -1;

	//find closest to start
	//Minimize range from start

	for ( i=0; i< nodeCount; i++)
	{
		double rangeFromStart = (double) (fromPoint.mm_range_to_point(the_waypoints()[i].location()));

		if (rangeFromStart < bestRangeFromStart)
		{
			bestRangeFromStart = rangeFromStart;
			node = i;
		}
	}

	if (node < 0) return -1;

	double bestRangeToTarget = 1000000.0;

	//now select all nearby (+ 0.5m). See which one is in the right direction
	//minimize range to target

#define RANGE_BRACKET 1000		//mm
	for ( i=0; i< nodeCount; i++)
	{
		double rangeFromStart = (double) (fromPoint.mm_range_to_point(the_waypoints()[i].location()));
		double rangeToTarget = (double) (inDirectionOfTarget.mm_range_to_point(the_waypoints()[i].location()));

		if (((rangeFromStart - bestRangeFromStart) < RANGE_BRACKET) && (rangeToTarget < bestRangeToTarget))
		{
			bestRangeToTarget = rangeToTarget;
			node = i;
		}
	}

	return node;
}

double Planner::HeuristicCalculate(int _from, int _to)
{
	//A-Star heuristic
	return (double) the_waypoints()[_from].location().mm_range_to_point(the_waypoints()[_to].location());
}

//priority queue
void Planner::pqInsert(int n)
{
	IPQ[IPQCount++] = n;

	DEBUGPRINT("planner: IPQ inserted %i at %i", n, IPQCount);
}
int Planner::pqPop()
{
	int minCost = 100000;
	int index = -1;
	int result = -1;

	for (int i = 0; i < IPQCount; i++){
		if ( minCost > FCosts[IPQ[i]]) {
			minCost = FCosts[IPQ[i]];
			index = i;
		}
	}

	if (index >= 0){
		result = IPQ[index];
		for (int i = index + 1; i < IPQCount; i++){
			IPQ[i - 1] = IPQ[i];
		}
		IPQCount--;
	}

	DEBUGPRINT("planner: IPQ pop = %i, IPQCount = %i", result, IPQCount);

	return result;
}
void Planner::pqChangePriority(int n)
{
	//null because priority handled during pop
}


