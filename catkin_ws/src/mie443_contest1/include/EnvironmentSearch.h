// INCLUDE //
#ifndef ES_HEADER
#define ES_HEADER

// ROS //
#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

// Other //
#include <stdio.h>
#include <cmath>

#include <chrono>
#include <thread>

#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)

using namespace std;
using namespace std::this_thread;
using namespace std::chrono;

class EnvironmentSearch {

    private:
        // 
        ros::NodeHandle nh;
        ros::Publisher vel_pub;
        ros::Subscriber bumper_sub, laser_sub, odom;
        geometry_msgs::Twist vel;

        float posX = 0.0, posY = 0.0, yaw = 0.0;
        uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED}; // Global variable to store bumper state
        
        // Lasers
        float minLaserDist = std::numeric_limits<float>::infinity();
        float minLeftLaserDist = std::numeric_limits<float>::infinity();
        float minRightLaserDist = std::numeric_limits<float>::infinity();

        float maxLeftLaserDist = 0.0;
        float maxRightLaserDist = 0.0;
        float maxLaserDist = 0.0;
        
        int32_t nLasers=0, desiredNLasers=0, desiredAngle=15; // Global variable to store values from laser callback
        int leftIndex;
        int rightIndex;
        int backingUpCounter = 0;
        float directionArray[4] = {0,M_PI/2,M_PI,M_PI*1.5};
        int directionIndex = 0;

        // time
        std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
        uint64_t secondsElapsed = 0;

        // Limits and Thresholds
        float wallLimit = 0.5;
        float currYaw = 0.0;
        // float yawAdjustment = 0.0;
        // float linearAdjustment = 0.0;

        //// Controller
        //float kp = 1.5;
        //float pController(float minLeftDist, float minRightDist, float leftIndex, float rightIndex, float kp, float prevAngular);
        

        void envSearchMain(uint64_t secondsElapsed);

        // float randomScan();

        void publishVelocity(float angular, float linear);

        void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg);
        void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    public:
        
        EnvironmentSearch() {
            cout << "Class created" << endl;
            
            bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &EnvironmentSearch::bumperCallback, this);
            laser_sub = nh.subscribe("scan", 10, &EnvironmentSearch::laserCallback, this);
            odom = nh.subscribe("odom", 1, &EnvironmentSearch::odomCallback, this);

            vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
  
        }
        
        void search(uint64_t secondsElapsed);


};

#endif