#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <stdio.h>
#include <cmath>

#include <chrono>

#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)
float posX = 0.0, posY = 0.0, yaw = 0.0;
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED}; // Global variable to store bumper state
float minLaserDist = std::numeric_limits<float>::infinity();
float leftMaxLaserDist = 0.0;
float rightMaxLaserDist = 0.0;
float maxLaserDist = 0.0;
int32_t nLasers=0, desiredNLasers=0, desiredAngle=15; // Global variable to store values from laser callback


void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	// Access using bumper[kobuki_msgs::BumperEvent::{}] LEFT, CENTER, or RIGHT
    bumper[msg->bumper] = msg->state;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	maxLaserDist = 0.0;
    minLaserDist = std::numeric_limits<float>::infinity();
    nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    desiredNLasers = desiredAngle*M_PI / (180*msg->angle_increment);
    ROS_INFO("Size of laser scan array: %i and size of offset: %i", nLasers, desiredNLasers);
    
    if (desiredAngle * M_PI / 180 < msg->angle_max && -desiredAngle * M_PI / 180 > msg->angle_min) {
        for (uint32_t laser_idx = nLasers / 2 - desiredNLasers; laser_idx < nLasers / 2 + desiredNLasers; ++laser_idx){
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
            maxLaserDist = std::max(maxLaserDist, msg->ranges[laser_idx]);
        }
// right
        for (uint32_t laser_idx = 0; laser_idx < nLasers / 2 - desiredNLasers; ++laser_idx){
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
            rightMaxLaserDist = std::max(maxLaserDist, msg->ranges[laser_idx]);
        }
// left
        for (uint32_t laser_idx = nLasers/2+ desiredNLasers; laser_idx < nLasers; ++laser_idx){
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
            leftMaxLaserDist = std::max(maxLaserDist, msg->ranges[laser_idx]);
        }

        

    }
    else {
        for (uint32_t laser_idx = 0; laser_idx < nLasers; ++laser_idx) {
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
            maxLaserDist = std::max(maxLaserDist, msg->ranges[laser_idx]);
        }
    }
}

void odomCallback (const nav_msgs::Odometry::ConstPtr& msg)
{
    posX = msg->pose.pose.position.x;           // Set the robot’s (X, Y) position along with its orientation
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);       // Orientation is given as a quaternion to convert it to the yaw angle
    ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, RAD2DEG(yaw)); // Print the robot’s position and orientation for testing purposes
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
    ros::Subscriber odom = nh.subscribe("odom", 1, &odomCallback);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    ros::Rate loop_rate(50);

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    float angular = 0.0;
    float linear = 0.0;

    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();

        bool turning = false;
        bool any_bumper_pressed = false;
        for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
            any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
        }

        // Spin at the start
        if (secondsElapsed <= 30) {
            angular = 0.37;
            linear = 0.0;
            turning = true;
        } else if (secondsElapsed >= 90 && secondsElapsed < 120) {
            angular = 0.37;
            linear = 0.0;
            turning = true;
        } else if (secondsElapsed >= 180 && secondsElapsed < 210) {
            angular = 0.37;
            linear = 0.0;
            turning = true;
        } else if (secondsElapsed >= 270 && secondsElapsed < 300) {
            angular = 0.37;
            linear = 0.0;
            turning = true;
        } else if (secondsElapsed >= 360 && secondsElapsed < 390) {
            angular = 0.37;
            linear = 0.0;
            turning = true;
        } 

        // Open space to the left
        if (leftMaxLaserDist > rightMaxLaserDist && minLaserDist > 0.5 && turning == false) {
            angular = 0.12;
            linear = 0.18;

        // Open space to the right    
        } else if (rightMaxLaserDist > leftMaxLaserDist && minLaserDist > 0.5 && turning == false) {
            angular = -0.12;
            linear = 0.18;
        } else if (minLaserDist <= 0.5 && turning == false) {
            
            if (leftMaxLaserDist > rightMaxLaserDist) {
                angular = 0.5;
                linear = -0.05;
            } else if (rightMaxLaserDist > leftMaxLaserDist) {
                angular = -0.5;
                linear = -0.05;
            }

        }

        
        /*
        // Control logic after bumpers are being pressed.
        ROS_INFO("Postion: (%f, %f) Orientation: %f degrees Range: %f", posX, posY, RAD2DEG(yaw), minLaserDist);
        if (posX < 0.5 && yaw < M_PI / 12 && !any_bumper_pressed && minLaserDist > 0.7) {
            angular = 0.0;
            linear = 0.2;
        }
        else if (yaw < M_PI / 2 && posX > 0.5 && !any_bumper_pressed && minLaserDist > 0.5) {
            angular = M_PI / 6;
            linear = 0.0;
        }
        else if (minLaserDist > 1. && !any_bumper_pressed) {
            linear = 0.1;
            if (yaw < 17 / 36 * M_PI || posX > 0.6) {
                angular = M_PI / 12.;
            }
            else if (yaw < 19 / 36 * M_PI || posX < 0.4) {
                angular = -M_PI / 12.;
            }
            else {
                angular = 0;
            }
        }
        else {
            angular = 0.0;
            linear = 0.0;
        }
        */


        /*
        // BUMPER EXAMPLE //  
        // Check if any of the bumpers were pressed.
        bool any_bumper_pressed = false;
        for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
            any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
        }
        //
        // Control logic after bumpers are being pressed.
        if (posX < 0.5 && yaw < M_PI / 12 && !any_bumper_pressed) {
            angular = 0.0;
            linear = 0.2;
        }
        else if (yaw < M_PI / 2 && posX > 0.5 && !any_bumper_pressed) {
            angular = M_PI / 6;
            linear = 0.0;
        }
        else {
            angular = 0.0;
            linear = 0.0;
            break;
        }
        */


        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}