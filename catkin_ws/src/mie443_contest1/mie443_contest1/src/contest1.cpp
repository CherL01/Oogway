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

//uint8_t leftState = bumper[kobuki_msgs::BumperEvent::LEFT];
//uint8_t centerState = bumper[kobuki_msgs::BumperEvent::CENTER];
//uint8_t rightState = bumper[kobuki_msgs::BumperEvent::RIGHT];

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

    ROS_INFO("Position: (%f, %f) Orientation: %f degrees Range: %f", posX, posY, yaw*180/pi, maxLaserRange);

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

    float angular = 0.0;
    float linear = 0.0;

    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();
        //fill with your code
        
        bool any_bumper_pressed = false;
        for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
            any_bumper_pressed |=(bumper[b_idx]==kobuki_msgs::BumperEvent::PRESSED);
        }

        //Control logic after bumpers are being pressed
        if (posX < 0.5 && yaw < M_PI / 12 && !any_bumper_pressed) {
            angular = 0.0;
            linear = 0.2;
        }

        else if (posX > 0.5 && yaw < M_PI / 12 && !any_bumper_pressed) {
            angular = M_PI / 6;
            linear = 0.0;
        }

        else {
            angular = 0.0;
            linear = 0.0;
            break;
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
