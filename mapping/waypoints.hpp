/*
 * waypoints.hpp
 *
 *  Created on: 2015
 *      Author: martin
 */

//Waypoint database

#ifndef WAYPOINTS_H_
#define WAYPOINTS_H_

#include <vector>
#include <string>
#include "location.hpp"

using namespace std;
class Waypoint;

//a connection to another waypoint
class GraphEdge{
public:
	GraphEdge() {}
	GraphEdge(Waypoint *fromWaypoint, string toWaypointName);		//linked waypoint name
	Waypoint *fromWaypoint() {return fromWaypointPtr;}
	Waypoint *toWaypoint();
	string toWaypointName() {return toName;}

private:
	Waypoint *fromWaypointPtr {nullptr};	//cached pointer
	Waypoint *toWaypointPtr {nullptr};	//cached pointer
	string 	 toName;
};

//a waypoint on a route - with a location and connections
class Waypoint {
public:
	Waypoint(){}
	Waypoint(Location &L) {wpLocation = L;}

	//member getters
	Location 	location() 	{return wpLocation;}
	double		latitude()	{return wpLocation.latitude_in_degrees();}
	double 		longitude()	{return wpLocation.longitude_in_degrees();}
	std::string	name()		{return wpLocation.name;}

	//setter
	int setLocation(Location &L){wpLocation = L; return 0;}

	//GraphEdge methods
	int AddGraphEdge(const string wp2);
	int IterateGraphEdges(void (*callback)(int index, GraphEdge *conn, void *args), void *_args);
	GraphEdge &operator[](int);

	int edges_size();

private:
	//data members
	Location wpLocation;
	vector<GraphEdge> edges;

	GraphEdge null_graphedge;
};

//the database
class Waypoints
{
public:

	//clear the DB
	int ResetWaypointList();
	
	//iterate through the DB
    int IterateWaypoints(void (*callback)(int index, Waypoint *wp, void *args), void *_args);
    Waypoint &operator[](int);
    
    //lookup
    Waypoint &GetWaypointByName(const char *name);
    Waypoint &GetWaypointByName(string name);

	int		GetWaypointIndexByName(string name);


	//Update waypoints
	int AddWaypoint(Location wp);
	int DeleteWaypoint(string wp_name);
	int AddWaypointConnection(const string wp1, const string wp2);

	//file operations
	int LoadWaypointDatabase(const char *filename);
	int SaveWaypointDatabase(const char *filename);

	//count of waypoints
	int waypoints_size();

private:
	Waypoints(){}
	~Waypoints(){ResetWaypointList();}
	
	vector<Waypoint> waypoints;

	int InsertWaypointConnection(const string wp_name, const string target_name);
	
	Waypoint null_waypoint;

	friend Waypoints &the_waypoints();
};

Waypoints &the_waypoints();

#endif
