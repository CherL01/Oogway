#include "move.h"

float posX=0.0, posY=0.0, yaw=0.0;


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