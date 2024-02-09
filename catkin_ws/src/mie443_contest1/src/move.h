#ifndef MOVE_HEADER
#define MOVE_HEADER

#include "globals.h"

extern float targetYaw;
extern float remainingYaw;
extern float angular, linear;
extern bool isMoving;

extern float remainingDist;
extern float dist;


void odomCallback (const nav_msgs::Odometry::ConstPtr& msg);

//functions to turn
void turnCCW (float& targetYaw, float& angular, float& linear, float& remainingYaw);
void turnCW (float& targetYaw, float& angular, float& linear, float& remainingYaw);

//check functions to see if turn completed - run while moving
void checkTurnCCW (float& targetYaw, float& angular, float& linear, float& remainingYaw);
void checkTurnCW (float& targetYaw, float& angular, float& linear, float& remainingYaw);

//functions to move
void moveFront (float& targetDist, float& currentX, float& currentY, float& angular, float& linear, float& turtleSpeed);
void moveBack (float& targetDist, float& currentX, float& currentY, float& angular, float& linear, float&turtleSpeed);

//check functions to see if move completed - run while moving
void checkMoveFront (float& targetDist, float& currentX, float& currentY, float& angular, float& linear, float&turtleSpeed);
void checkMoveBack (float& targetDist, float& currentX, float& currentY, float& angular, float& linear, float& turtleSpeed);

#endif