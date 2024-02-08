#include "globals.h"
#include "laserCallback.cpp"
#include "move.cpp"
#include "bumper.cpp"

//state globals

//uint8_t is a boolean variable


//float is 32-bit decimal 
float angular = 0.0;
float linear = 0.0;

float targetYaw = 180.0;
float remainingYaw = 0.0;

bool turning = true;

//target yaw greater than current yaw -> turn ccw
 //check how much left by remaining: (target yaw - current yaw)
    //if remaining <=0, angular/linear=0


//target yaw less than current yaw -> turn cw
 // check how much left by (current yaw - target yaw)
    //if remaining <=0, angular/linear=0





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


    float angular = 0.0;
    float linear = 0.0;


    while(ros::ok()) {
        /*
        if (loopCount == 5) {
            break;
        }*/
        

        ros::spinOnce();
        //ROS_INFO("Ros spinned once");

        //fill with your code

/*
        //turn CCW

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

            break;

        }

        else //keep rotating as long as there is remaining yaw
        {
            angular = M_PI/6;
            linear = 0.0;
            ROS_INFO("Turning CCW... remaining: %f", remainingYaw);
        }*/

        //turn CW

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

        

        vel.linear.x = linear;
        vel.angular.z = angular;
        vel_pub.publish(vel);
        
        // The last thing to do is to update the timer.
        //secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}
