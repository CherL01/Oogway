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

double posX = 0.0, posY = 0.0, yaw = 0.0;

uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};

float minLaserDist = std::numeric_limits<float>::infinity();
int32_t nLasers=0, desiredNLasers=0, desiredAngle=5;

// uint8_t leftState = bumper[kobuki_msgs::BumperEvent::LEFT];
// uint8_t centerState = bumper[kobuki_msgs::BumperEvent::CENTER];
// uint8_t rightState = bumper[kobuki_msgs::BumperEvent::RIGHT];

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
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
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    // ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, RAD2DEG(yaw));
}

float RandomFloat(float a, float b) {
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = b - a;
    float r = random * diff;
    return a + r;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "maze_explorer");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    ros::Subscriber odom = nh.subscribe("odom", 1, &odomCallback);

    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;


    // initialize loop variables

    // angle tracking uses degrees!!
    int angle_tracker = 0;
    int yaw_deg = RAD2DEG(yaw);
    int prev_yaw_deg = yaw_deg;
    int initial_yaw = yaw_deg;
    int final_yaw = (initial_yaw + 360) % (360);

    // travelling conditions
    bool scan_360 = true;
    bool change_turn_direction = false;
    bool obstacle_avoidance = true;
    float collision_turn = M_PI / 9;
    int collision_turn_deg = RAD2DEG(collision_turn);
    float reg_turn = M_PI / 3;
    int reg_turn_deg = RAD2DEG(reg_turn);

    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();

        // yaw preprocessing
        // make everything positive!
        if (yaw < 0) {
            yaw += 2 * M_PI;
        }
        
        // change yaw to degrees
        yaw_deg = int RAD2DEG(yaw);

        ROS_INFO("yaw (deg): ", yaw_deg);

        // get difference between current and previous yaw
        int yaw_diff = abs(yaw_deg - prev_yaw_deg);
        ROS_INFO("%f", yaw_diff);

        // // Control logic after bumpers are being pressed
        // ROS_INFO("Position: (%f, %f) Orientation: %f degrees Range: %f", posX, posY, RAD2DEG(yaw), minLaserDist);

        // check if bumper pressed
        bool any_bumper_pressed = false;
        int bumper_num = -1;
        for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
            any_bumper_pressed |=(bumper[b_idx]==kobuki_msgs::BumperEvent::PRESSED);
            if (bumper[b_idx]==kobuki_msgs::BumperEvent::PRESSED) {
                bumper_num = b_idx;
            }
        }

        // spin 360 to scan area
        if (scan_360) {
            ROS_INFO("scanning 360 deg");
            angular = reg_turn;
            linear = 0.0;

            angle_tracker += yaw_diff;

            // stop spinning after 360
            if (yaw_deg > final_yaw && angle_tracker > 2 * M_PI) {
                ROS_INFO("done scanning 360 deg");
                scan_360 = false;
                angular = 0.0;
                linear = 0.0;
                angle_tracker = 0;
            }
        }

        // not spinning anymore, proceed to regular mapping conditions
        else {

            // if bumper pressed, move backwards and turn 20 deg
            if (any_bumper_pressed) {
                ROS_INFO("bumper pressed, too close! (attempt to back up and turn slightly)");

                switch (bumper_num) {

                    case 0:
                        angular = -collision_turn;
                        break;

                    case 2:
                        angular = collision_turn;
                        break;
                }

                // angular = collision_turn;
                // linear = 0.01;
                linear = 0.01;
            }

            // if min laser distance less than 1m, turn 
            else if (minLaserDist < 1) {
                ROS_INFO("laser distance < 1m, need to turn! (attempt to turn left/right to avoid hitting obstacles)");
                if (obstacle_avoidance) {
                    initial_yaw = yaw_deg;
                    final_yaw = (initial_yaw + int RAD2DEG(reg_turn)) % (360);
                    obstacle_avoidance = false;
                }

                linear = 0.0;
                angular = reg_turn;

                angle_tracker += yaw_diff;
            }

            // no obstacles, go straight
            else if (yaw_deg > final_yaw && angle_tracker > reg_turn_deg) {
                ROS_INFO("cruising");
                linear = 0.25;
                angular = 0.0;
                obstacle_avoidance = true;
            }

            if (secondsElapsed % 30 == 0) {
                ROS_INFO("it's been 30 seconds, time to scan");
                scan_360 = true;
                initial_yaw = yaw_deg;
                final_yaw = (initial_yaw + 360) % (360);
            }

            if (change_turn_direction) {
                ROS_INFO("turning right");
                angular *= -1;
            }

            else if (posX < 0.5 && posY < 0.5 && secondsElapsed > 120) {
                ROS_INFO("changed to turning right");
                change_turn_direction = true;
            } 

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
