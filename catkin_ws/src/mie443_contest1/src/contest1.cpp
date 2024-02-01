#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>

#include <stdio.h>
#include <cmath>

#include <chrono>
#include <thread>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad)*180./M_PI)
#define DEG2RAD(deg) ((deg)*M_PI /180.)

float angular = 0.0;
float linear = 0.0;

//float posX = 0.0, posY = 0.0, yaw = 0.0;
double posX = 0.0, posY = 0.0, yaw = 0.0;

uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};

float minLaserDist = std::numeric_limits<float>::infinity();
int32_t nLasers=0, desiredNLasers=0, desiredAngle=5;

// uint8_t leftState = bumper[kobuki_msgs::BumperEvent::LEFT];
// uint8_t centerState = bumper[kobuki_msgs::BumperEvent::CENTER];
// uint8_t rightState = bumper[kobuki_msgs::BumperEvent::RIGHT];

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	//fill with your code
    // LEFT, CENTER, or RIGHT
    bumper[msg->bumper] = msg->state;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//fill with your code
    minLaserDist = std::numeric_limits<float>::infinity();
    nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    desiredNLasers = DEG2RAD(desiredAngle)/msg->angle_increment;
    ROS_INFO("Size of laser scan array: %i and size of offset: %i", nLasers, desiredNLasers);
    if (desiredAngle * M_PI / 180 <msg->angle_max && -desiredAngle * M_PI / 180 > msg->angle_min){
        for (uint32_t laser_idx = nLasers / 2 - desiredNLasers; laser_idx < nLasers / 2 + desiredNLasers; ++laser_idx){
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
        }
    }
    else {
        for(uint32_t laser_idx = 0; laser_idx < nLasers; ++laser_idx) {
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
        }
    }

}

void odomCallback (const nav_msgs::Odometry::ConstPtr& msg)
{
    //fill w code
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, RAD2DEG(yaw));
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "maze_explorer");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    ros::Subscriber odom = nh.subscribe("odom", 1, &odomCallback);

    //ros::Subscriber odom = nh.subscribe("odom", 1, odomCallback);

    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    // float angular = 0.0;
    // float linear = 0.0;

    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();
        
        bool any_bumper_pressed = false;
        for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
            any_bumper_pressed |=(bumper[b_idx]==kobuki_msgs::BumperEvent::PRESSED);
        }

        //Control logic after bumpers are being pressed
        // ROS_INFO("Position: (%f, %f) Orientation: %f degrees Range: %f", posX, posY, RAD2DEG(yaw), minLaserDist);

        bool scan_360 = true;
        bool change_turn_direction = false;

        if (scan_360) {
            ROS_INFO("scanning 360 deg");
            angular = 2 * M_PI - M_PI/120;
            linear = 0.0;
            scan_360 == false;
        }

        if (any_bumper_pressed) {
            ROS_INFO("bumper pressed, too close! (attempt to back up and turn slightly)");
            angular = M_PI / 9;
            // linear = 0.01;
            linear = -0.01;
        }

        else if (minLaserDist < 0.7 && yaw < M_PI / 120) {
            ROS_INFO("laser distance < 0.7m, need to turn! (attempt to turn left/right to avoid hitting obstacles)");
            linear = 0.0;
            angular = M_PI / 2;
            
        }

        else if (yaw < M_PI / 120) {
            ROS_INFO("cruising");
            linear = 0.25;
            angular = 0.0;
        }


        // if (posX < 0.5 && yaw < M_PI / 12 && !any_bumper_pressed && minLaserDist > 0.7) {
        //     ROS_INFO("in first if statement");
        //     angular = 0.0;
        //     linear = 0.1;
        // }

        // else if (posX > 0.5 && yaw < M_PI / 2 && !any_bumper_pressed && minLaserDist > 0.5) {
        //     ROS_INFO("in first elif statement");
        //     angular = M_PI / 6;
        //     linear = 0.0;
        // }

        // else if (minLaserDist > 1. && !any_bumper_pressed) {
        //     ROS_INFO("in second elif statement");
        //     linear = 0.1;
        //     if (yaw < 17 / 36 * M_PI || posX > 0.6) {
        //         ROS_INFO("in first if statement of second elif statement");
        //         angular = M_PI / 12;
        //     }

        //     else if (yaw < 19 / 36 * M_PI || posX < 0.4) {
        //         ROS_INFO("in first elif statement of second elif statement");
        //         angular = -M_PI / 12;
        //     }
        //     else {
        //         ROS_INFO("in else statement of second elif statement");
        //         angular = 0;
        //     }
        // }

        // else {
        //     ROS_INFO("in else statement");
        //     angular = 0.0;
        //     linear = 0.0;
        //     break;
        // } 

        if (secondsElapsed % 10 == 0) {
            ROS_INFO("it's been 10 seconds, time to scan");
            scan_360 == true;
        }

        if (change_turn_direction == true) {
            ROS_INFO("turning right");
            angular *= -1;
        }

        else if (posX < 0.5 && posY < 0.5 && secondsElapsed > 120) {
            ROS_INFO("changed to turning right");
            change_turn_direction == true;
        }

        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);

        

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}
