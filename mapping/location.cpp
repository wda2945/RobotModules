/*
 ============================================================================
 Name        : Location.cpp
 Author      : Martin
 Version     :
 Copyright   : (c) 2015 Martin Lane-Smith
 Description : Load and maintain waypoint list
 ============================================================================
 */

#include <math.h>
#include <errno.h>

#include "location.hpp"

Location::Location(const Location &p)
{
	latitude = p.latitude; longitude = p.longitude; name = p.name; type = p.type;
}

double Location::bearing_to_point(Location w)
{
	int32_t northing = (w.latitude - latitude);
	int32_t easting = (w.longitude - longitude);

	double bearing = RADIANS_TO_DEGREES(atan2((double)easting, (double)northing));
	if (bearing < 0) return (bearing + 360);
	else return bearing;
}

int Location::mm_range_to_point(Location w)
{
	double northing = (double)(w.latitude - latitude);
	double easting = (double)(w.longitude - longitude);

	double range = sqrt(northing*northing + easting*easting);
	return (int) SCALED_DEGREES_TO_MM(range);
}

Location Location::range_bearing(int _rangeMM, double bearing_deg)
{
	double bearingRadians = bearing_deg * M_PI / 180.0;
	double northing = (double) (cos(bearingRadians) * _rangeMM);
	double easting = (double) (sin(bearingRadians) * _rangeMM);
	int32_t lat = (latitude + MM_TO_SCALED_DEGREES(northing));
	int32_t lon = (longitude + MM_TO_SCALED_DEGREES(easting));

	return Location(lat, lon);
}

vector2D Location::v2d_mm_to_point(Location w)
{
	vector2D vec;

	vec.x() = (double) SCALED_DEGREES_TO_MM(w.longitude - longitude);
	vec.y() = (double) SCALED_DEGREES_TO_MM(w.latitude - latitude);

	return vec;
}

const std::string &Location::description()
{
	string_description = name + " @ " + std::to_string(latitude_in_degrees()) + " " + std::to_string(longitude_in_degrees());
	return string_description;
}
