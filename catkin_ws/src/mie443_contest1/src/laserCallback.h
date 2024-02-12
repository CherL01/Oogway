#ifndef LASER_HEADER
#define LASER_HEADER

#include "globals.h"

// Laser related functions
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

#endif