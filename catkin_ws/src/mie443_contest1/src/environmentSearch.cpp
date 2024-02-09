#include "environmentSearch.h"

// PUBLIC//

void environmentSearch::setup() {
    // Setting up
    ROS_INFO("Setting up...");

}

void environmentSearch::search() {
    // main search code
    ROS_INFO("Searching...");
    ros::spinOnce();

}

// PRIVATE //

void environmentSearch::envSearchMain(uint64_t secondsElapsed) {
    
    float angular = 0.0;
    float linear = 0.0;
    bool turning = false;

    // Spin at the start, spin once in a while
    if (secondsElapsed <= 20) {
        angular = 0.25;
        linear = 0.0;
        turning = true;
    } else if (secondsElapsed >= 20 && secondsElapsed < 30) {
        angular = -0.25;
        linear = 0.0;
        turning = true;
    } else if (secondsElapsed >= 120 && secondsElapsed < 150) {
        angular = 0.3;
        linear = 0.0;
        turning = true;
    } else if (secondsElapsed >= 210 && secondsElapsed < 240) {
        angular = 0.3;
        linear = 0.0;
        turning = true;
    } else if (secondsElapsed >= 300 && secondsElapsed < 330) {
        angular = 0.3;
        linear = 0.0;
        turning = true;
        } else if (secondsElapsed >= 390 && secondsElapsed < 420) {
        angular = 0.3;
        linear = 0.0;
        turning = true;
    } 

    // Open space to the left
    if (leftMaxLaserDist > rightMaxLaserDist && minLaserDist > 0.5 && turning == false) {
        angular = 0.125;
        linear = 0.175;

    // Open space to the right    
    } else if (rightMaxLaserDist > leftMaxLaserDist && minLaserDist > 0.5 && turning == false) {
        angular = -0.125;
        linear = 0.175;

    } else if (minLaserDist <= 0.5 && turning == false) {
            
        // Too close to a wall, back up and slightly turn
        if (leftMaxLaserDist > rightMaxLaserDist) {
            angular = 0.5;
            linear = -0.03;
        } else if (rightMaxLaserDist > leftMaxLaserDist) {
            angular = -0.5;
            linear = -0.03;
        } 

    }

    publishVelocity(angular, linear);
}

void environmentSearch::publishVelocity(float angular, float linear) {
    // Publish velocity values
    vel.angular.z = angular;
    vel.linear.x = linear;
    vel_pub.publish(vel);

    // Process a single round of callbacks
    ros::spinOnce();
}

void environmentSearch::avoidWall() {
    float wallLimit = 0.5;
    float currentWallDist = minLaserDist;

    float yawAdjustment = 0.0;
    float linearAdjustment = 0.0;

    if (currentWallDist < wallLimit) {
        yawAdjustment = 1.0;
        linearAdjustment = 0.0;
    } else {
        yawAdjustment = 0.0;
        linearAdjustment = 0.1;
    }

    publishVelocity(yawAdjustment,linearAdjustment);

}

void environmentSearch::bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	// Access using bumper[kobuki_msgs::BumperEvent::{}] LEFT, CENTER, or RIGHT
    bumper[msg->bumper] = msg->state;
}

void environmentSearch::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
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
            // minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
            rightMaxLaserDist = std::max(maxLaserDist, msg->ranges[laser_idx]);
        }
// left
        for (uint32_t laser_idx = nLasers/2+ desiredNLasers; laser_idx < nLasers; ++laser_idx){
            // minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
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

void environmentSearch::odomCallback (const nav_msgs::Odometry::ConstPtr& msg)
{
    posX = msg->pose.pose.position.x;           // Set the robot’s (X, Y) position along with its orientation
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);       // Orientation is given as a quaternion to convert it to the yaw angle
    ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, RAD2DEG(yaw)); // Print the robot’s position and orientation for testing purposes
}

