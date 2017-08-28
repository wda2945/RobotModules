/*
 * IMUClass.hpp
 *
 *      Author: martin
 */

#ifndef IMUCLASS_HPP
#define IMUCLASS_HPP

#include <chrono>

#define IMU_DATA_TIMEOUT 30

class IMUClass
{
public:
	IMUClass() {};
	virtual ~IMUClass() {};

	float heading {0.0};
	float pitch {0.0};
	float roll {0.0};

	std::chrono::system_clock::time_point updated;

	virtual bool new_imu_data() {return false;}

protected:


};

IMUClass &the_imu();

#endif
