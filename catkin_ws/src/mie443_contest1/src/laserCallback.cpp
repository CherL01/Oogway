
#include "laserCallback.h"

float minLaserDist = std::numeric_limits<float>::infinity();
float leftLaserDist = std::numeric_limits<float>::infinity();
float rightLaserDist = std::numeric_limits<float>::infinity();
int32_t nLasers=0, desiredNLasers=0, desiredAngle=15;


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
        
        /*
        ROS_INFO("FRONT: %g", minLaserDist);
        ROS_INFO("RIGHT-END: %g", rightLaserDist);
        ROS_INFO("LEFT-END: %g", leftLaserDist);
        */



    
    }
    else{
        for (uint32_t laser_idx=0; laser_idx<nLasers; ++laser_idx){
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
            ROS_INFO("!!!OUT OF RANGE!!! - minLaserDist = %f", minLaserDist);
        }
    }
}