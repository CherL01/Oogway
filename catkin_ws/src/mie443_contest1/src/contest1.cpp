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
int32_t nLasers=0, desiredNLasers=0, desiredAngle=5; // Global variable to store values from laser callback


void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	// Access using bumper[kobuki_msgs::BumperEvent::{}] LEFT, CENTER, or RIGHT
    bumper[msg->bumper] = msg->state;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	
    minLaserDist = std::numeric_limits<float>::infinity();
    nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    desiredNLasers = desiredAngle*M_PI / (180*msg->angle_increment);
    ROS_INFO("Size of laser scan array: %i and size of offset: %i", nLasers, desiredNLasers);
    
    if (desiredAngle * M_PI / 180 < msg->angle_max && -desiredAngle * M_PI / 180 > msg->angle_min) {
        for (uint32_t laser_idx = nLasers / 2 - desiredNLasers; laser_idx < nLasers / 2 + desiredNLasers; ++laser_idx){
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
        }
    }
    else {
        for (uint32_t laser_idx = 0; laser_idx < nLasers; ++laser_idx) {
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
        }
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
    ros::Subscriber odom = nh.subscribe("odom", 1, &odomCallback);


    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;


    float angular = 0.0;
    float linear = 0.0;


    while(ros::ok()) {
        /*
        if (loopCount == 5) {
            break;
        }*/
        

        ros::spinOnce();
        
        bool any_bumper_pressed = false;
        for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
            any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
        }
        
        // Spin at the start
        if (secondsElapsed <= 15) {
            angular = 0.5;
        }


        if (minLaserDist > 0.5 && !any_bumper_pressed) {
            angular = 0.0;
            linear = 0.25;
        } else {
            angular = 0.5;
            linear = 0.0;   
        }
        
        /*
        // Control logic after bumpers are being pressed.
        ROS_INFO("Postion: (%f, %f) Orientation: %f degrees Range: %f", posX, posY, RAD2DEG(yaw), minLaserDist);
        if (posX < 0.5 && yaw < M_PI / 12 && !any_bumper_pressed && minLaserDist > 0.7) {
            angular = 0.0;
            linear = 0.0;
            ROS_INFO("Turning is false!");
            ROS_INFO("Stopped turning...");

            break;

        }

        else //keep rotating as long as there is remaining yaw
        {
            angular = M_PI/6;
            linear = 0.0;
            ROS_INFO("Turning CCW... remaining: %f", remainingYaw);
        }*/

        //turn CW

        remainingYaw = yaw - targetYaw;
        ROS_INFO("Yaw: %f", yaw);
        ROS_INFO("Target Yaw: %f", targetYaw);
        ROS_INFO("Remaining Yaw: %f", remainingYaw);

        if(remainingYaw <= 0.0) //stop once remaining is less than 0
        {
            angular = 0.0;
            linear = 0.0;
            ROS_INFO("Turning is false!");
            ROS_INFO("Stopped turning...");

            //break;
        }

        else //keep rotating as long as there is remaining yaw
        {
            angular = -M_PI/6;
            linear = 0.0;
            ROS_INFO("Turning CW... remaining: %f", remainingYaw);
        }

        

        vel.linear.x = linear;
        vel.angular.z = angular;
        vel_pub.publish(vel);
        
        // The last thing to do is to update the timer.
        //secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}
