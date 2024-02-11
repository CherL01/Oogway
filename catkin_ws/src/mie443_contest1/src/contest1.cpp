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

#include "EnvironmentSearch.h"

///////////////////////////////////
////////   MAIN CODE   ////////////
///////////////////////////////////

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    // ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    // ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
    // ros::Subscriber odom = nh.subscribe("odom", 1, &odomCallback);

    // ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    ros::Rate loop_rate(50);
    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;
    
    EnvironmentSearch ES;

    while(ros::ok() && secondsElapsed <= 480) {
        
        // bool any_bumper_pressed = false;
        // for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
        //     any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
        // }

        ES.search(secondsElapsed);
        ROS_INFO("Seconds Elapsed: %u", secondsElapsed);

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
        
    }

    return 0;
}


