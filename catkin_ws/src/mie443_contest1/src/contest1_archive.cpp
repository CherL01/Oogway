#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad)*180. /M_PI)
#define DEG2RAD(deg) ((deg)* M_PI /180.)

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

//uint8_t is a boolean variable
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};

uint8_t leftState = bumper[kobuki_msgs::BumperEvent::LEFT];
uint8_t rightState = bumper[kobuki_msgs::BumperEvent::RIGHT];
uint8_t centerState = bumper[kobuki_msgs::BumperEvent::CENTER];

//float is 32-bit decimal 
float angular = 0.0;
float linear = 0.0;
float posX = 0.0, posY = 0.0, yaw = 0.0;
float minLaserDist = std::numeric_limits<float>::infinity();
float leftLaserDist = std::numeric_limits<float>::infinity();
float rightLaserDist = std::numeric_limits<float>::infinity();

//int32 is an integer
int32_t nLasers=0, desiredNLasers=0, desiredAngle=15;
int32_t nRanges=0;

uint64_t timeToTurn = 0;


void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
    bumper[msg->bumper] = msg->state;
    leftState = bumper[0];
    centerState = bumper[1];
    rightState = bumper[2];

    //ROS_INFO("Left Bumper: %i, Center Bumper: %i, Right Bumper: %i", leftState, centerState, rightState);
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    std::cout << "laser calling..." << std::endl;

    minLaserDist = std::numeric_limits<float>::infinity();
    leftLaserDist = std::numeric_limits<float>::infinity();
    rightLaserDist = std::numeric_limits<float>::infinity();
    
    nLasers = (msg->angle_max-msg->angle_min)/ msg->angle_increment; //639
    desiredNLasers = DEG2RAD(desiredAngle)/msg->angle_increment; //5=53, 10=106, 15=159, 20=212

    //ROSTOPIC INFO: angle_max=0.524, angle_min=-0.5215, angle_increment=0.001636
    //angle can fit in between +-30 degrees
    //index 0 is right
    //index nLasers(max) is left


    //code for playing with laserscan
    /*
    nRanges = (msg->range_max);
    ROS_INFO("MAX Range: %i", nRanges);
    nRanges = (msg->range_min);
    ROS_INFO("MIN Range: %i", nRanges);

    nRanges = msg->angle_increment;
    ROS_INFO("ANGLE INCREMENT: %f", nRanges);
    */
    

    //ROS_INFO("Size of laser scan array: %i and size of offset: %i", nLasers, desiredNLasers);
    
    // FRONT-FACING: 0 degrees, hence the midpoint of ranges' index
    
    if (desiredAngle*M_PI/180 < msg->angle_max && desiredAngle*M_PI/180 > msg->angle_min){ //if desiredAngle is within max/min angles

        //CHECK CENTRE minDistance
        for (uint32_t laser_idx = nLasers/2-desiredNLasers; laser_idx <nLasers/2 + desiredNLasers; ++laser_idx){ //index within desired range
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);

        }

        

        //CHECK RIGHT maxDistance
        for (uint32_t rLaser_idx = 0; rLaser_idx < nLasers/2-desiredNLasers; ++rLaser_idx){
            rightLaserDist = std::min(rightLaserDist, msg->ranges[rLaser_idx]);
        }

        

        //CHECK LEFT maxDIstance
        for (uint32_t lLaser_idx = nLasers/2+desiredNLasers; lLaser_idx < nLasers; ++lLaser_idx){
            leftLaserDist = std::min(leftLaserDist, msg->ranges[lLaser_idx]);
        }
        
        ROS_INFO("FRONT: %g", minLaserDist);
        ROS_INFO("RIGHT-END: %g", rightLaserDist);
        ROS_INFO("LEFT-END: %g", leftLaserDist);



    
    }
    else{
        for (uint32_t laser_idx=0; laser_idx<nLasers; ++laser_idx){
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
            ROS_INFO("!!!OUT OF RANGE!!! - minLaserDist = %f", minLaserDist);
        }
    }
}

void odomCallback (const nav_msgs::Odometry::ConstPtr& msg)
{

    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
	yaw = tf::getYaw(msg->pose.pose.orientation);
    if (yaw < 0) {
        yaw += 2*M_PI;
    }
    //ROS_INFO("Position: (%f, %f) Orientation: %f degrees.", posX, posY, RAD2DEG(yaw));
	//ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, RAD2DEG(yaw));

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

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    float angular = 0.0;
    float linear = 0.0;
    uint64_t loopCount = 0;

    float angSpeed = M_PI/9;

/*
    //CODE: do a 360 scan to check biggest opening

    //update angular 
    angular = M_PI/6;
    linear = 0.0;
    vel.angular.z = angular;
    vel.linear.x = linear;
    vel_pub.publish(vel);
    std::chrono::time_point<std::chrono::system_clock> turnStart;
    turnStart = std::chrono::system_clock::now();
    uint64_t turnSecondsElapsed = 0;

    uint64_t turnTime = 2*M_PI/angular;


    //loop until 360 degrees turn completes (time = 2*M_PI/angular)
    while(ros::ok() && turnSecondsElapsed <= turnTime) {
        ros::spinOnce();

        //update minLaserDistance with coordinates

        //update time
        turnSecondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-turnStart).count();
        loop_rate.sleep();
    }
*/

    while(ros::ok() && secondsElapsed <= 480) {
        /*
        if (loopCount == 5) {
            break;
        }*/
        float slowDownLimit = 0.8;
        float stopLimit = 0.5;
        float maxOpening = 0.95;
        float midLimit = 0.6;
        float leftLimit = 0.75;
        

        ros::spinOnce();
        ROS_INFO("Ros spinned once");

        //fill with your code
        bool any_bumper_pressed = false; //no bumpers pressed initially, true if pressed
        for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
            any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED); //iterate through bumper, check if pressed 
        }
        
        //bumper pressed -> eStop
        if (any_bumper_pressed) {
            angular = 0;
            linear = 0;
            vel.angular.z = angular;
            vel.linear.x = linear;
            vel_pub.publish(vel);

            ros::spinOnce();
            ROS_INFO("EMERGENCY STOP");
            ROS_INFO("linear speed: %f, rotate speed: %f", vel.linear.x, vel.angular.z);
            ROS_INFO("Current Position: (%f, %f", posX, posY);

            //record positions to get relative positions
            float eStop_posX = posX;
            float eStop_posY = posY;
        
            
            float relative_dist = sqrt(pow(posX - eStop_posX, 2) + pow(posY - eStop_posY, 2));
            ROS_INFO("Relative distance is %f", relative_dist);


            // back up for 0.5m
            while (sqrt(pow(posX - eStop_posX, 2) + pow(posY - eStop_posY, 2)) < 0.2) {
                ros::spinOnce();
                float relative_dist = sqrt(pow(posX - eStop_posX, 2) + pow(posY - eStop_posY, 2));
                ROS_INFO("Backing Up.... moved %f", relative_dist);
                linear = -0.1;
                angular = 0.0;
                vel.angular.z = angular;
                vel.linear.x = linear;
                vel_pub.publish(vel);
            }

            //Stop after backing up
            angular = 0;
            linear = 0;
            vel.angular.z = angular;
            vel.linear.x = linear;
            vel_pub.publish(vel);

            //record yaw to get relative yaw & set up target yaw
            ros::spinOnce();
            float eStop_yaw = yaw;

            float relative_yaw = DEG2RAD(90);

            float targetYaw = eStop_yaw + relative_yaw;

            ROS_INFO("Target Yaw SET!");

            if (eStop_yaw + relative_yaw >= 2*M_PI) {
                targetYaw -= 2*M_PI;
            }
            
            ROS_INFO("Target Yaw: %f", RAD2DEG(targetYaw));
            ROS_INFO("Current Yaw: %f", RAD2DEG(yaw));

            //rotate 90
            while (RAD2DEG(yaw) > RAD2DEG(targetYaw)+0.5 || RAD2DEG(yaw) < RAD2DEG(targetYaw)-0.5) {
                ros::spinOnce();
                ROS_INFO("Rotating...turned %f", yaw-eStop_yaw);
                ROS_INFO("Target yaw: %f", RAD2DEG(targetYaw));
                ROS_INFO("Current yaw: %f", RAD2DEG(yaw));
                angular = M_PI/6;
                linear = 0.0;
                vel.angular.z = angular;
                vel.linear.x = linear;
                vel_pub.publish(vel);
            }

        }

        else if (secondsElapsed % 120 == 0) {
            timeToTurn = secondsElapsed;
            ROS_INFO("Time to Turn!");

            angular = 0;
            linear = 0;
            vel.angular.z = angular;
            vel.linear.x = linear;
            vel_pub.publish(vel);

            //record yaw to get relative yaw & set up target yaw
            ros::spinOnce();
            float eStop_yaw = yaw;

            float relative_yaw = DEG2RAD(180);

            float targetYaw = eStop_yaw + relative_yaw;

            ROS_INFO("Target Yaw SET!");

            if (eStop_yaw + relative_yaw >= 2*M_PI) {
                targetYaw -= 2*M_PI;
            }
            ROS_INFO("Target Yaw: %f", RAD2DEG(targetYaw));
            ROS_INFO("Current Yaw: %f", RAD2DEG(yaw));

            //rotate 180
            while (RAD2DEG(yaw) > RAD2DEG(targetYaw)+5 || RAD2DEG(yaw) < RAD2DEG(targetYaw)-5) {
                ros::spinOnce();
                ROS_INFO("Rotating...turned %f", yaw-eStop_yaw);
                ROS_INFO("Target yaw: %f", RAD2DEG(targetYaw));
                ROS_INFO("Current yaw: %f", RAD2DEG(yaw));
                angular = M_PI/6;
                linear = 0.0;
                vel.angular.z = angular;
                vel.linear.x = linear;
                vel_pub.publish(vel);
            }

            angular = 0;
            linear = 0;
            vel.angular.z = angular;
            vel.linear.x = linear;
            vel_pub.publish(vel);

            //record yaw to get relative yaw & set up target yaw
            ros::spinOnce();
            eStop_yaw = yaw;

            relative_yaw = DEG2RAD(180);

            targetYaw = eStop_yaw + relative_yaw;

            ROS_INFO("Target Yaw SET!");

            if (eStop_yaw + relative_yaw >= 2*M_PI) {
                targetYaw -= 2*M_PI;
            }
            ROS_INFO("Target Yaw: %f", RAD2DEG(targetYaw));
            ROS_INFO("Current Yaw: %f", RAD2DEG(yaw));

            //rotate 180
            while (RAD2DEG(yaw) > RAD2DEG(targetYaw)+5 || RAD2DEG(yaw) < RAD2DEG(targetYaw)-5) {
                ros::spinOnce();
                ROS_INFO("Rotating...turned %f", yaw-eStop_yaw);
                ROS_INFO("Target yaw: %f", RAD2DEG(targetYaw));
                ROS_INFO("Current yaw: %f", RAD2DEG(yaw));
                angular = M_PI/6;
                linear = 0.0;
                vel.angular.z = angular;
                vel.linear.x = linear;
                vel_pub.publish(vel);
            }
            

        }

        ///*
        //No bumpers hit, use laserscan through maze

        else {
            ROS_INFO("Operating Normally");
            
            //rotate to find open space - NEED TO FIX
            // while (minLaserDist < stopLimit || leftLaserDist < stopLimit || rightLaserDist < stopLimit) {
            //     ros::spinOnce();
            //     angular = M_PI/9;
            //     vel.angular.z = angular;
            //     vel_pub.publish(vel);

            //     ROS_INFO("Rotating to find open space...");
            // }

            
            ros::spinOnce(); //update scan info and odometry info in case bumper pressed

            ROS_INFO("FRONT: %g", minLaserDist);
            ROS_INFO("RIGHT-END: %g", rightLaserDist);
            ROS_INFO("LEFT-END: %g", leftLaserDist);

            
            //Reduce speed if front distance is less than 0.7!
            if (minLaserDist <= slowDownLimit or rightLaserDist <= slowDownLimit or leftLaserDist <= slowDownLimit) {
                ROS_INFO("Reducing Speed...");
                linear = 0.1;
                angular = 0.0;
            }

            else {
                ROS_INFO("Normal Speed");
                linear = 0.25;
                angular = 0.0;
            }
            vel.angular.z = angular;
            vel.linear.x = linear;
            vel_pub.publish(vel);

            
            

            //
            //Stop if front or right or left is less than 0.5 (too close to wall)!
            if (minLaserDist <= stopLimit or rightLaserDist <= stopLimit or leftLaserDist <= stopLimit) {
            
                ros::spinOnce();

                linear = 0.0;
                angular = 0.0;
                vel.angular.z = angular;
                vel.linear.x = linear;
                vel_pub.publish(vel); 
                ROS_INFO("Stopped to Turn");


                //Narrow Path: right and left are small and front is greater
                if (leftLaserDist < minLaserDist && rightLaserDist < minLaserDist) {
                    
                    ROS_INFO("Narrow Path...");
                    while (minLaserDist <= 0.7 || rightLaserDist <= 0.7 || leftLaserDist <= 0.7) {

                            // run linearly to go through narrow path
                            ros::spinOnce();

                            linear = 0.25;
                            angular = 0.0;
                            vel.linear.x = linear;
                            vel.angular.z = angular;
                            vel_pub.publish(vel); 
                    }

                }

                //turn cw if only left is too close
                if (leftLaserDist <= stopLimit && minLaserDist > stopLimit && rightLaserDist > stopLimit) {
                    while (leftLaserDist <= leftLimit) {
                        ros::spinOnce();

                        ROS_INFO("FRONT: %g", minLaserDist);
                        ROS_INFO("RIGHT-END: %g", rightLaserDist);
                        ROS_INFO("LEFT-END: %g", leftLaserDist);

                        angular = -angSpeed;
                        vel.angular.z = angular;
                        vel_pub.publish(vel);
                        ROS_INFO("Turning CW... Speed: %f", vel.angular.z);
                    }
                }

                //turn ccw until right scan is greater than midlimit
                while (minLaserDist <= midLimit || rightLaserDist <= midLimit) {
                    

                    ros::spinOnce();

                    ROS_INFO("FRONT: %g", minLaserDist);
                    ROS_INFO("RIGHT-END: %g", rightLaserDist);
                    ROS_INFO("LEFT-END: %g", leftLaserDist);

                    angular = angSpeed;
                    vel.angular.z = angular;
                    vel_pub.publish(vel);
                    ROS_INFO("Turning CCW... Speed: %f", vel.angular.z);
                }

                

            }//

        }
        //ROS_INFO("SPEED UPDATE! linear speed: %f, rotate speed: %f", vel.linear.x, vel.angular.z);
        //*/
        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
        loopCount += 1;
    }

    return 0;
}
