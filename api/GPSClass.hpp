/*
 * GPS.h
 *
 *      Author: martin
 */

#ifndef GPSCLASS_H_
#define GPSCLASS_H_

#include <chrono>
#include <stdint.h>
#include <stdbool.h>

class GPSClass
{
public:
	GPSClass() {}
	virtual ~GPSClass() {}

	bool		dateValid {false};
	int16_t		year {0};
	uint8_t		month {0};
	uint8_t		day {0};
	bool		timeValid {false};
	uint8_t		hour {0};
	uint8_t		min {0};
	uint8_t		sec {0};
	uint8_t		numSatsInView {0};		//satellites in view

	bool fix {false};

	bool		positionValid {false};
	int32_t		longitude {0};			//degrees * 10,000,000
	int32_t		latitude {0};			//degrees * 10,000,000
	uint32_t	horizontalAcc {0};		//horizontal accuracy mm

	bool		speedValid {false};
	int32_t		velocityN {0};			//mm/s
	int32_t		velocityE {0};			//mm/s
	int32_t		groundSpeed {0};		//mm/s
	uint32_t	speedAcc {0};			//mm/s

	bool		headingValid {false};
	int32_t		headMotion {0};			//degrees
	uint32_t	headAcc {0};			//degrees

	bool		magValid {false};
	float 		magvariation {0};		//degrees * 100

	std::chrono::system_clock::time_point 		update_time;		//last gps message
	std::chrono::system_clock::time_point 		fix_time;			//last gps fix

	bool dgps_data {false};

	virtual bool new_gps_data() {return false;}

private:

};

GPSClass &the_gps();

#endif
