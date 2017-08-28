/*
 ============================================================================
 Name        : waypoints.cpp
 Author      : Martin
 Version     :
 Copyright   : (c) 2015 Martin Lane-Smith
 Description : Load and maintain waypoint list
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

#include "mxml.h"
#include "waypoints.hpp"

#define DEBUGPRINT(...) {printf(__VA_ARGS__); printf("\n");}

//////////////////////////////////// Graph Edge

GraphEdge::GraphEdge(Waypoint *fromWaypoint, string toWaypointName)
{
	fromWaypointPtr = fromWaypoint;
	toName = toWaypointName;
}

Waypoint *GraphEdge::toWaypoint()
{
	if (!toWaypointPtr)
	{
		toWaypointPtr = &the_waypoints().GetWaypointByName(toName);
	}
	return toWaypointPtr;
}

//////////////////////////////////// Waypoint Class

//GraphEdge methods
int Waypoint::AddGraphEdge(const string wp2)
{
	GraphEdge ge(this, wp2);
	edges.push_back(ge);
	return 0;
}

int Waypoint::IterateGraphEdges(void (*callback)(int index, GraphEdge *conn, void *args), void *_args)
{
	int count = (int) edges.size();
	for (int i=0; i<count; i++)
    {
        (callback)(i, &edges[i], _args);
    }
    return 0;
}

GraphEdge &Waypoint::operator[](int i)
{
	if (i < (int) edges.size())
	{
		return edges[i];
	}
	else return null_graphedge;
}

int Waypoint::edges_size() {return (int) edges.size();}

//////////////////////////////////// Waypoints Class
Waypoints &the_waypoints()
{
    static Waypoints me;
    return me;
}

int Waypoints::ResetWaypointList()
{
	waypoints.clear();
	return 0;
}

int Waypoints::waypoints_size()
{
	 return (int) waypoints.size();
}

int Waypoints::IterateWaypoints(void (*callback)(int index, Waypoint *wp, void *args), void *_args)
{
	int count = (int) waypoints.size();

    for (int i=0; i<count; i++)
    {
        (callback)(i, &waypoints[i], _args);
    }
    return 0;
}

Waypoint &Waypoints::operator[](int i)
{
	if (i < (int) waypoints.size())
	{
		return waypoints[i];
	}
	else return null_waypoint;
}

int Waypoints::AddWaypoint(Location _loc)
{
    for (auto iter=waypoints.begin(); iter!=waypoints.end(); iter++)
    {
    	if (iter->name() == _loc.name)
    	{
        	//exists - update position
    		iter->setLocation(_loc);
    	}
    }

    //new
    //make the wp struct
    Waypoint wp(_loc);
    waypoints.push_back(wp);

	return 0;
}


Waypoint &Waypoints::GetWaypointByName(const char *name)
{
	return GetWaypointByName(string(name));
}

Waypoint &Waypoints::GetWaypointByName(string _name)
{
    for (auto iter=waypoints.begin(); iter!=waypoints.end(); iter++)
    {
    	if (iter->name() == _name)
    	{
        	//exists
    		return *iter;
    	}
    }

	return null_waypoint;
}

int		Waypoints::GetWaypointIndexByName(string name)
{
	int count = (int) waypoints.size();

    for (int i=0; i<count; i++)
    {
        if (waypoints[i].name() == name) return i;
    }
    return -1;
}

int Waypoints::DeleteWaypoint(const string _name)
{
    for (auto iter=waypoints.begin(); iter!=waypoints.end(); iter++)
    {
    	if (iter->name() == _name)
    	{
        	//exists
    		waypoints.erase(iter);
    		return 0;
    	}
    }
	return -1;
}

int Waypoints::InsertWaypointConnection(const string wp_name, const string target_name)
{
	for (auto iter=waypoints.begin(); iter!=waypoints.end(); iter++)
	{
		if (iter->name() == wp_name)
		{
			iter->AddGraphEdge(target_name);
			return 0;
		}
	}
	return -1;
}

int Waypoints::AddWaypointConnection(const string wp1, const string wp2)
{
	if (InsertWaypointConnection(wp1, wp2) < 0) return -1;
	if (InsertWaypointConnection(wp2, wp1) < 0) return -1;
	return 0;
}

mxml_type_t type_cb(mxml_node_t *node)
{
    const char *type;
    
    type = mxmlElementGetAttr(node, "type");
    if (type == NULL)
        type = mxmlGetElement(node);
    
    if (!strcmp(type, "real"))
        return (MXML_REAL);
    else
        return (MXML_TEXT);
}


int Waypoints::LoadWaypointDatabase(const char *filename)
{
    FILE *fp;
    mxml_node_t *xml;    /* <?xml ... ?> */

    mxml_node_t *waypoint;   /* <waypoint> */
    mxml_node_t *name;  /* <name> */
    mxml_node_t *latitude;  /* <latitude> */
    mxml_node_t *longitude;  /* <longitude> */

    mxml_node_t *connection;  /* <connection> */
    mxml_node_t *cName;

    char *waypointName;
    Location position;

    fp = fopen(filename, "r");

    if (fp)
    {
    	waypoints.clear();

    	xml = mxmlLoadFile(NULL, fp, type_cb);
    	fclose(fp);

    	//walk tree
    	//iterate waypoints
        for (waypoint = mxmlFindElement(xml, xml,
                                    "waypoint",
                                    NULL, NULL,
                                    MXML_DESCEND);
        		waypoint != NULL;
        		waypoint = mxmlFindElement(waypoint, xml,
                                    "waypoint",
                                    NULL, NULL,
                                    MXML_DESCEND))
        {
        	name = mxmlFindElement(waypoint, waypoint,
        	                                    "name",
        	                                    NULL, NULL,
        	                                    MXML_DESCEND);
        	waypointName = const_cast<char*>(mxmlGetText(name, NULL));

        	latitude = mxmlFindElement(waypoint, waypoint,
        	                                    "latitude",
        	                                    NULL, NULL,
        	                                    MXML_DESCEND);
        	position.set_latitude_in_degrees(mxmlGetReal(latitude));

        	longitude = mxmlFindElement(waypoint, waypoint,
            	                                    "longitude",
            	                                    NULL, NULL,
            	                                    MXML_DESCEND);
            position.set_longitude_in_degrees(mxmlGetReal(longitude));

            position.type = Location::NAMED_WAYPOINT;

            if (waypointName)
            {
            	position.name = std::string(waypointName);

            	AddWaypoint(position);

            	DEBUGPRINT("WPXML: %s", waypointName);

            	//iterate connections
                for (connection = mxmlFindElement(waypoint, waypoint,
                                            "connection",
                                            NULL, NULL,
                                            MXML_DESCEND);
                		connection != NULL;
                		connection = mxmlFindElement(connection, waypoint,
                                            "connection",
                                            NULL, NULL,
                                            MXML_DESCEND))
                {
                	cName = mxmlFindElement(connection, connection,
                	                                    "name",
                	                                    NULL, NULL,
                	                                    MXML_DESCEND);
                	const char *connectionName = mxmlGetText(cName, NULL);

                	if (connectionName)
                	{
                		InsertWaypointConnection(waypointName, connectionName);

                       	DEBUGPRINT("WPXML: %s -> %s", waypointName, connectionName);

                    }
                	else
                	{
                		DEBUGPRINT("WPXML: Missing connectionName in '%s'", waypointName);
                	}
                }
            }
            else
            {
        		DEBUGPRINT("WPXML: Missing waypointName");
            }
        }
    	mxmlDelete(xml);
    }
    else
    {
		DEBUGPRINT("WPXML: can't open %s for read", filename);
        return -1;
    }
    return 0;
}

int Waypoints::SaveWaypointDatabase(const char *filename)
{
    mxml_node_t *xml;    /* <?xml ... ?> */
    mxml_node_t *data;   /* <data> */

    mxml_node_t *waypoint;   /* <waypoint> */
    mxml_node_t *name;      /* <name> */
    mxml_node_t *latitude;  /* <latitude> */
    mxml_node_t *longitude;  /* <longitude> */

    mxml_node_t *connections;  /* <connections> */
    mxml_node_t *connection;  /* <connection> */

    xml = mxmlNewXML("1.0");

    data = mxmlNewElement(xml, "data");

	for (Waypoint current : waypoints)
	{
		waypoint = mxmlNewElement(data, "waypoint");

		name = mxmlNewElement(waypoint, "name");
        mxmlElementSetAttr (name, "type", "text");
        mxmlNewText(name, 0, current.name().c_str());

        latitude = mxmlNewElement(waypoint, "latitude");
        mxmlElementSetAttr (latitude, "type", "real");
        mxmlNewReal(latitude, current.latitude());

        longitude = mxmlNewElement(waypoint, "longitude");
        mxmlElementSetAttr (longitude, "type", "real");
        mxmlNewReal(longitude, current.longitude());

        connections = mxmlNewElement(waypoint, "connections");

        int count = current.edges_size();

        for (int i=0; i<count; i++)
        {
        	GraphEdge currentConnection = current[i];

        	connection = mxmlNewElement(connections, "connection");

    		name = mxmlNewElement(connection, "name");
            mxmlElementSetAttr (name, "type", "text");
            mxmlNewText(name, 0, currentConnection.toWaypointName().c_str());
        }
	}

    FILE *fp;

    fp = fopen(filename, "w");

    if (fp)
    {
    	mxmlSaveFile(xml, fp, MXML_NO_CALLBACK);
    	fclose(fp);
    }
    else
    {
		DEBUGPRINT("WPXML: can't open %s for write", filename);
        return -1;
    }
    mxmlDelete(xml);
    return 0;
}

