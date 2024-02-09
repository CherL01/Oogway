#include "move.h"

float posX=0.0, posY=0.0, yaw=0.0;
float remainingYaw = std::numeric_limits<float>::infinity();
float targetYaw = std::numeric_limits<float>::infinity();

float remainingDist = std::numeric_limits<float>::infinity();
float dist = 0.0;




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
void turnCCW (float& targetYaw, float& angular, float& linear, float& remainingYaw)
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

    remainingYaw = targetYaw - yaw; //initialize remainingYaw
    
    //ROS_INFO("Yaw: %f", yaw);
    //ROS_INFO("Target Yaw: %f", targetYaw);
    //ROS_INFO("Remaining Yaw: %f", remainingYaw); 

    ///* logic to check ismovingis done 
    if(remainingYaw <= 0.0) //stop once remaining is less than 0
    {
        angular = 0.0;
        linear = 0.0;
        //ROS_INFO("Turning is false!");
        //ROS_INFO("Stopped turning...");

    }

    else //keep rotating as long as there is remaining yaw
    {
        angular = M_PI/6;
        linear = 0.0;
        //ROS_INFO("Turning CCW... remaining: %f", remainingYaw);
    } //logic to ~~ */

}

//function to turn clockwise
void turnCW (float& targetYaw, float& angular, float& linear, float& remainingYaw)
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

    if(remainingYaw <= 0.0) //stop once remaining is less than 0
    {
        angular = 0.0;
        linear = 0.0;

        //break;
    }

    else //keep rotating as long as there is remaining yaw
    {
        angular = -M_PI/6;
        linear = 0.0;
        //ROS_INFO("Turning CW... remaining: %f", remainingYaw);
    }
}

void checkTurnCCW (float& targetYaw, float& angular, float& linear, float& remainingYaw)
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

    remainingYaw = targetYaw - yaw; //initialize remainingYaw
    
    //ROS_INFO("Yaw: %f", yaw);
    //ROS_INFO("Target Yaw: %f", targetYaw);
    //ROS_INFO("Remaining Yaw: %f", remainingYaw); 

    ///* logic to check ismovingis done 
    if(remainingYaw <= 0.0) //stop once remaining is less than 0
    {
        angular = 0.0;
        linear = 0.0;
        //ROS_INFO("Turning is false!");
        //ROS_INFO("Stopped turning...");

    }

}

void checkTurnCW (float& targetYaw, float& angular, float& linear, float& remainingYaw)
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

    if(remainingYaw <= 0.0) //stop once remaining is less than 0
    {
        angular = 0.0;
        linear = 0.0;

        //break;
    }
}



/////MOVING
//functions to move forwards and backwards
void moveFront (float& targetDist, float& currentX, float& currentY, float& angular, float& linear)
{
    dist = sqrt(pow(currentX-posX, 2) + pow(currentY-posY, 2));

    remainingDist = targetDist - dist;

    if (remainingDist <= 0)
    {
        angular = 0.0;
        linear = 0.0;
        ROS_INFO("STOPPED MOVING!!");
    }

    else
    {
        angular = 0.0;
        linear = 0.25;
        
    }
}

void moveBack (float& targetDist, float& currentX, float& currentY, float& angular, float& linear)
{
    dist = sqrt(pow(currentX-posX, 2) + pow(currentY-posY, 2));

    remainingDist = targetDist - dist;

    if (remainingDist <= 0)
    {
        angular = 0.0;
        linear = 0.0;
        ROS_INFO("STOPPED MOVING!!");
    }

    else
    {
        angular = 0.0;
        linear = -0.25;
        
    }

}

//check functions to see if move completed - run while moving
void checkMoveFront (float& targetDist, float& currentX, float& currentY, float& angular, float& linear)
{
    dist = sqrt(pow(currentX-posX, 2) + pow(currentY-posY, 2));

    remainingDist = targetDist - dist;

    if (remainingDist <= 0)
    {
        angular = 0.0;
        linear = 0.0;
        ROS_INFO("STOPPED MOVING!!");
    }

    else
    {
        angular = 0.0;
        linear = 0.25;
    }
}

void checkMoveBack (float& targetDist, float& currentX, float& currentY, float& angular, float& linear);
{
    dist = sqrt(pow(currentX-posX, 2) + pow(currentY-posY, 2));

    remainingDist = targetDist - dist;

    if (remainingDist <= 0)
    {
        angular = 0.0;
        linear = 0.0;
        ROS_INFO("STOPPED MOVING!!");
    }

    else
    {
        angular = 0.0;
        linear = -0.25;
    }
}