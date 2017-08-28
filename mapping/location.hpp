//
//  location.hpp
//  RobotFramework
//
//  Created by Martin Lane-Smith on 5/18/16.
//  Copyright © 2016 Martin Lane-Smith. All rights reserved.
//

#ifndef location_h
#define location_h

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string>

#include "vector.h"

#define LATLONG_SCALE			10000000
#define MM_PER_DEGREE			111120000
#define MM_PER_SCALED_DEGREE	11.112
#define DEGREES_TO_MM(N) ((N) * 60 * 1852000)
#define MM_TO_SCALED_DEGREES(N) ((int32_t)((N) / MM_PER_SCALED_DEGREE))
#define SCALED_DEGREES_TO_MM(N) ((N) * MM_PER_SCALED_DEGREE)
#define FLOAT_TO_SCALED_INT(D) ((int32_t) ((D) * LATLONG_SCALE))
#define SCALED_INT_TO_FLOAT(D) (((double)(D)) / LATLONG_SCALE)

#define RADIANS_TO_DEGREES(r) ((r) * 180.0 / M_PI)
#define DEGREES_TO_RADIANS(d) ((d) * M_PI / 180.0)

typedef Vector<2> vector2D;

class Location
{
public:
	//default constructor
	Location() {}
	
	//named location
	Location(std::string _name) {name = _name; type = NAMED_WAYPOINT;}
	
	//copy constructor
	Location(const Location &p);
	
	//HPINT constructor
	Location(int32_t lat, int32_t lon) {latitude = lat; longitude = lon; type = UNNAMED_WAYPOINT;}
	//named HPINT constructor
	Location(std::string _name, int32_t lat, int32_t lon) {name = _name; latitude = lat; longitude = lon; type = NAMED_WAYPOINT;}
	
	//float degrees constructor
	Location(float lat, float lon) {latitude = FLOAT_TO_SCALED_INT(lat); longitude = FLOAT_TO_SCALED_INT(lon); type = UNNAMED_WAYPOINT;}
	//named float degrees constructor
	Location(std::string _name, float lat, float lon) {name = _name; latitude = FLOAT_TO_SCALED_INT(lat); longitude = FLOAT_TO_SCALED_INT(lon); type = NAMED_WAYPOINT;}

	//properties
	std::string name;
	int32_t latitude	{0};		//scaled by LATLONG_SCALE (10,000,000)
	int32_t longitude	{0};

	typedef enum {
		UNKNOWN_LOCATION, GPS_LOCATION, NAMED_WAYPOINT, UNNAMED_WAYPOINT
	} location_type_enum;

	location_type_enum type = UNKNOWN_LOCATION;

	//conversion methods
	double latitude_in_degrees() {return SCALED_INT_TO_FLOAT(latitude);}
	double longitude_in_degrees() {return SCALED_INT_TO_FLOAT(longitude);}
    void   set_latitude_in_degrees(double _latitude) {latitude = FLOAT_TO_SCALED_INT(_latitude);}
    void   set_longitude_in_degrees(double _longitude) {longitude = FLOAT_TO_SCALED_INT(_longitude);}
    
	//trig methods
	//vector to another point
    vector2D v2d_mm_to_point(Location w);
	//bearing to another point
	double bearing_to_point(Location w);
	//range to another point
	int mm_range_to_point(Location w);
	
	//location at a given range and bearing
	Location range_bearing(int _rangeMM, double bearing_deg);

	const std::string &description();

protected:

	std::string string_description;

};

#endif /* location_h */
