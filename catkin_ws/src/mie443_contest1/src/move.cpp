#include "move.h"

float posX=0.0, posY=0.0, yaw=0.0;
float remainingYaw = 0.0;
bool turning = true;

float angular=0.0, linear = 0.0;


void odomCallback (const nav_msgs::Odometry::ConstPtr& msg)
{

    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
	yaw = tf::getYaw(msg->pose.pose.orientation);
    if (yaw < 0) {
        yaw += 2*M_PI;
    }

    yaw = RAD2DEG(yaw);
    //ROS_INFO("move.h working...!");

}

//function to turn counterclockwise
std::pair<float,float> turnCCW (float targetYaw)
{
    //correct any out of bounds for targetYaw

    if (targetYaw >= 360)
    {
        targetYaw = targetYaw - 360;
    }
    else if (targetYaw <= 0)
    {
        targetYaw = targetYaw + 360;
    } 


    if (yaw > targetYaw && abs(yaw-targetYaw) > 4) //need to tune threshold value 4
    {
        targetYaw += 360;
    }

    remainingYaw = targetYaw - yaw;
    
    ROS_INFO("Yaw: %f", yaw);
    ROS_INFO("Target Yaw: %f", targetYaw);
    ROS_INFO("Remaining Yaw: %f", remainingYaw);


    if(remainingYaw <= 0.0) //stop once remaining is less than 0
    {
        angular = 0.0;
        linear = 0.0;
        ROS_INFO("Turning is false!");
        ROS_INFO("Stopped turning...");

    }

    else //keep rotating as long as there is remaining yaw
    {
        angular = M_PI/6;
        linear = 0.0;
        ROS_INFO("Turning CCW... remaining: %f", remainingYaw);
    }

    return std::make_pair (angular, linear);
}

//function to turn clockwise
std::pair<float, float> turnCW (float targetYaw)
{

    //correct any out of bounds for targetYaw
    if (targetYaw >= 360)
    {
        targetYaw = targetYaw - 360;
    }
    else if (targetYaw <= 0)
    {
        targetYaw = targetYaw + 360;
    }


    if (yaw < targetYaw && abs(yaw-targetYaw) > 4)
    {
        targetYaw -= 360;
    }

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

    return std::make_pair (angular, linear);
}