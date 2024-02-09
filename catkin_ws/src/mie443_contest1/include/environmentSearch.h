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

#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)

using namespace std;

class environmentSearch {

    private:
        // hi
        ros::NodeHandle nh;
        ros::Publisher vel_pub;
        ros::Subscriber bumper_sub, laser_sub, odom;
        
        float posX = 0.0, posY = 0.0, yaw = 0.0;
        uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED}; // Global variable to store bumper state
        float minLaserDist = std::numeric_limits<float>::infinity();
        float leftMaxLaserDist = 0.0;
        float rightMaxLaserDist = 0.0;
        float maxLaserDist = 0.0;
        int32_t nLasers=0, desiredNLasers=0, desiredAngle=15; // Global variable to store values from laser callback

        void envSearchMain();

        void avoidWall();

        void publishVelocity(float angular, float linear);

        void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg);
        void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    public:
        // hey
        
        environmentSearch() {
            cout << "Class created" << endl;
            
            bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &environmentSearch::bumperCallback, this);
            laser_sub = nh.subscribe("scan", 10, &environmentSearch::laserCallback, this);
            odom = nh.subscribe("odom", 1, &environmentSearch::odomCallback, this);

            vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
  
        }
        
        void setup();
        void search(uint64_t secondsElapsed);


};

#endif