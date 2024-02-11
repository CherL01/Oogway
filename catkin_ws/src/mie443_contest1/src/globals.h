#ifndef GLOBAL_HEADER
#define GLOBAL_HEADER

//libraries needed 

#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>

#include <stdio.h>
#include <cmath>

#include <chrono>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

using namespace std;

// equations, definitions
#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad)*180. /M_PI)
#define DEG2RAD(deg) ((deg)* M_PI /180.)


//global variables


const float slowDownLimit = 0.8;
const float stopLimit = 0.5; //cannot detect 0.5 -> increase to 0.6 or 0.7?
const float clearLimit = 0.5;

//State variables: Determines which state turtlebot is in

const int TRAVEL_STEP = 1; //travel state
const int SCAN_STEP = 0; //scanning state
const int travelLoopLimit = 10; //gittering loop limit

extern bool isMoving; //boolean to determing if turtlebot is moving or not

extern int stepsCount; //sets state of turtlebot
extern int subStepsCount; //substates
extern int travelLoop; //loop count during travelling

extern uint64_t travelTimeLimit;



//odometery variables
extern float posX, posY, yaw;

extern float targetDist; //store target distance for moving
extern float currentX; //record current position for moving
extern float currentY;

//moving variables
extern float angular, linear;
extern int turning;
extern bool isMoving;

const float normal=0.25, slowDown=0.1; //speed settings
const float normalAngular = M_PI/6, slowDownAngular = M_PI/4; //turn faster when close to walls during scanning

extern float turtleSpeed; //speed override
extern float turtleAngle; //angular spped override

extern float openYaw; //openYaw for turning after scan

extern float turnAngle; //turn increment during travel


//bumper variables
extern uint8_t bumper[3];
extern bool anyBumperPressed;

extern uint8_t leftState, rightState, centerState;



//laser variables
extern float minLaserDist, leftLaserDist, rightLaserDist;
extern int32_t desiredAngle;
extern int32_t nLasers, desiredNLasers;

#endif