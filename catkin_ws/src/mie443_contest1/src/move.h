#ifndef MOVE_HEADER
#define MOVE_HEADER

#include "globals.h"

extern float targetYaw;
extern float remainingYaw;

void odomCallback (const nav_msgs::Odometry::ConstPtr& msg);


std::pair<float, float> turnCCW (float targetYaw);
std::pair<float, float> turnCW (float targetYaw);

#endif