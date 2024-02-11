#include "EnvironmentSearch.h"

// PUBLIC//

void EnvironmentSearch::search(uint64_t secondsElapsed) {
    // main search code
    ROS_INFO("Searching...");
    ros::spinOnce();
    envSearchMain(secondsElapsed);
}

// PRIVATE //

///////////////////////////////////
////////   MAIN CODE   ////////////
///////////////////////////////////

void EnvironmentSearch::envSearchMain(uint64_t secondsElapsed) {
    
    float angular = 0.0;
    float linear = 0.0;
    bool turning = false;

    // Spin at the start, spin once in a while
    if (secondsElapsed <= 20) {
        angular = 0.3;
        linear = 0.0;

    } else if (secondsElapsed >= 20 && secondsElapsed < 30) {
        angular = -.3;
        linear = 0.0;

    } else if (secondsElapsed >= 120 && secondsElapsed < 130) {
        angular = 0.225;
        linear = 0.0;
    } else if (secondsElapsed >= 130 && secondsElapsed < 140) {
        angular = -0.225;
        linear = 0.0;

    } else if (secondsElapsed >= 200 && secondsElapsed < 220) {
        angular = 0.225;
        linear = 0.0;
    } else if (secondsElapsed >= 220 && secondsElapsed < 230) {
        angular = -0.225;
        linear = 0.0;

    } else if (secondsElapsed >= 290 && secondsElapsed < 310) {
        angular = 0.225;
        linear = 0.0;
    } else if (secondsElapsed >= 310 && secondsElapsed < 320) {
        angular = -0.225;
        linear = 0.0;

    } else if (secondsElapsed >= 380 && secondsElapsed < 400) {
        angular = 0.225;
        linear = 0.0;
    } else if (secondsElapsed >= 400 && secondsElapsed < 410) {
        angular = -0.225;
        linear = 0.0;

    } else {
        
        if (minLaserDist >= wallLimit) {
            if (minLeftLaserDist > minRightLaserDist) {
                angular = 0.1;
                linear = 0.175;
            } else if (minLeftLaserDist <= minRightLaserDist) {
                angular = -0.125;
                linear = 0.175;
            } 
        

        /*
        ////////////////// YAW STUFF /////////////////////
        currYaw = yaw;
        if (currYaw > M_PI*2) {
            currYaw -= M_PI*2;
        }
        
        // Default movement
        if (minLaserDist >= wallLimit) {
            // Running as normal. Lots of space from wall.
            if (currYaw > directionArray[directionIndex]) {
                angular = -0.125;
                linear = 0.175;
            } else if (currYaw <= directionArray[directionIndex]) {
                angular = 0.1;
                linear = 0.175;
            } 
            //ROS_INFO("Sending to pController-> minL: %f, minR: %f, idxL: %i, idxR: %i, kp: %f",minLeftLaserDist, minRightLaserDist, leftIndex, rightIndex, kp);
            //angular = pController(minLeftLaserDist, minRightLaserDist, leftIndex, rightIndex, kp, 0.);
            
        } else if (minLaserDist < wallLimit) {
            
            directionIndex += 1;
            if (directionIndex > 3) {
                directionIndex = 0;
            }

            while (currYaw < directionArray[directionIndex]) {
                angular = 0.3;
                linear = 0.0;
                publishVelocity(angular, linear);
                ros::spinOnce();
            }
        }
        ////////////////// YAW STUFF ENDS /////////////////////
*/

        } else if (minLaserDist < wallLimit) {
            // Too close to wall. Check which side has more space.
            
            if (backingUpCounter >=5) {
                if (minLeftLaserDist > minRightLaserDist) {
                    angular = 2;
                    linear = 0.0;
                    backingUpCounter = 0;
                } else if (minLeftLaserDist <= minRightLaserDist) {
                    angular = -2;
                    linear = 0.0;
                    backingUpCounter = 0;
                }
            }

            if (minLeftLaserDist > minRightLaserDist) {
                angular = -0.75;
                linear = -0.1;
                backingUpCounter+=1;
            } else if (minLeftLaserDist < minRightLaserDist) {
                angular = 0.75;
                linear = -0.1;
                backingUpCounter+=1;
            } else {
                angular = -0.5;
                linear = -0.08;
            }

        }
    }

    //ROS_INFO("Sending to publishVelocity -> Angular: %f, Linear: %f", angular, linear);
    publishVelocity(angular, linear);

}

///////////////////////////////////
//////   MAIN CODE ENDS   /////////
///////////////////////////////////


// float EnvironmentSearch::randomScan() {
    
//     // Turn in place
//     float angular = 0.3;
//     float linear = 0.0;
//     return angular, linear;
// }

void EnvironmentSearch::publishVelocity(float angular, float linear) {
    
    // Publish velocity values
    geometry_msgs::Twist vel;
    vel.angular.z = angular;
    vel.linear.x = linear;
    vel_pub.publish(vel);
    ROS_INFO("Published: Angular Vel: %f, Linear Vel: %f", angular, linear);
    
    // Process a single round of callbacks
    ros::spinOnce();
}

/* does not work :(
float EnvironmentSearch::pController(float minLeftDist, float minRightDist, float leftIndex, float rightIndex, float kp, float prevAngular) {
    ros::spinOnce();
    float angular = prevAngular;
    float laserDiff = minLeftDist - minRightDist;
    float indexDiff = leftIndex - rightIndex;
    
    if (laserDiff > 0.2) {
        angular = kp * laserDiff/minLeftDist;
    } else if (-1*laserDiff > 0.2) {
        angular = kp * laserDiff/minRightDist;
    }

    ROS_INFO("pController angular: %f", angular);
    return angular;

}
*/

void EnvironmentSearch::bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	// Access using bumper[kobuki_msgs::BumperEvent::{}] LEFT, CENTER, or RIGHT
    bumper[msg->bumper] = msg->state;
}

void EnvironmentSearch::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	maxLaserDist = 0.0;
    minLaserDist = std::numeric_limits<float>::infinity();
    leftIndex = nLasers/2 + desiredNLasers;
    rightIndex = nLasers/2 - desiredNLasers;
    nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    desiredNLasers = desiredAngle*M_PI / (180*msg->angle_increment);
    ROS_INFO("Size of laser scan array: %i and size of offset: %i", nLasers, desiredNLasers);
    
    if (desiredAngle * M_PI / 180 < msg->angle_max && -desiredAngle * M_PI / 180 > msg->angle_min) {
        for (uint32_t laser_idx = nLasers / 2 - desiredNLasers; laser_idx < nLasers / 2 + desiredNLasers; ++laser_idx){
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
            maxLaserDist = std::max(maxLaserDist, msg->ranges[laser_idx]);
        }
// right
        for (uint32_t laser_idx = 0; laser_idx < rightIndex; ++laser_idx){
            minRightLaserDist = std::min(minRightLaserDist, msg->ranges[laser_idx]);
            maxRightLaserDist = std::max(maxRightLaserDist, msg->ranges[laser_idx]);
        }
// left
        for (uint32_t laser_idx = leftIndex; laser_idx < nLasers; ++laser_idx){
            minLeftLaserDist = std::min(minLeftLaserDist, msg->ranges[laser_idx]);
            maxLeftLaserDist = std::max(maxLeftLaserDist, msg->ranges[laser_idx]);
        }

    }
    else {
        for (uint32_t laser_idx = 0; laser_idx < nLasers; ++laser_idx) {
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
            maxLaserDist = std::max(maxLaserDist, msg->ranges[laser_idx]);
        }
    }
}

void EnvironmentSearch::odomCallback (const nav_msgs::Odometry::ConstPtr& msg)
{
    posX = msg->pose.pose.position.x;           // Set the robot’s (X, Y) position along with its orientation
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);       // Orientation is given as a quaternion to convert it to the yaw angle
    ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, RAD2DEG(yaw)); // Print the robot’s position and orientation for testing purposes
}

