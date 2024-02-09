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

// equations, definitions
#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad)*180. /M_PI)
#define DEG2RAD(deg) ((deg)* M_PI /180.)


//global variables


const float slowDownLimit = 0.8;
const float stopLimit = 0.5;
const float maxOpening = 0.95;
const float midLimit = 0.7;

//odometery variables
extern float posX, posY, yaw;

//moving variables
extern float angular, linear;
extern int turning;
extern bool isMoving;

const float normal=0.25, slowDown=0.1;

//bumper variables
extern uint8_t bumper[3];
extern bool anyBumperPressed;

extern uint8_t leftState, rightState, centerState;




//laser variables
extern float minLaserDist, leftLaserDist, rightLaserDist;
extern int32_t desiredAngle;
extern int32_t nLasers, desiredNLasers;

#endif