#include <header.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/CliffEvent.h>
#include <imageTransporter.hpp>
#include <chrono>
#include <unistd.h>

using namespace std;

int counter = 0;

uint8_t bumperState[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
uint8_t cliffState[3] = {kobuki_msgs::CliffEvent::FLOOR, kobuki_msgs::CliffEvent::FLOOR, kobuki_msgs::CliffEvent::FLOOR};
uint16_t cliffHeight = 0;    

bool bumperPressed;
bool cliffDetected;

geometry_msgs::Twist follow_cmd;
int world_state;

void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
}

void bumperCB(const kobuki_msgs::BumperEvent::ConstPtr& msg){
    bumperState[msg->bumper] = msg->state;

}

void cliffCB(const kobuki_msgs::CliffEvent::ConstPtr& msg){
	cliffState[msg->sensor] = msg->state;
	cliffHeight = msg->bottom;
}

//-------------------------------------------------------------

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	sound_play::SoundClient sc;
	string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
	teleController eStop;

	//publishers
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);

	//subscribers
	ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
	ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCB);
	ros::Subscriber cliff = nh.subscribe("mobile_base/events/cliff", 10, &cliffCB);

    // contest count down timer
	ros::Rate loop_rate(10);
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

	imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	//imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //--for turtlebot Camera
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

	int world_state = 2;

	double angular = 0.0;
	double linear = 0.0;

	geometry_msgs::Twist vel;
	vel.angular.z = angular;
	vel.linear.x = linear;

	while(ros::ok() && secondsElapsed <= 480){		
		ros::spinOnce();
		ros::Duration(0.5).sleep(); //necessary to tune reactions
		ROS_INFO("start");

		//---Check Sensory Inputs ---

		//check bumper

		bumperPressed = false;
		if (bumperState[0] == kobuki_msgs::BumperEvent::PRESSED) //left bumper
		{
			ROS_INFO("Left Bumper PRESSED!!");
			bumperPressed = true;
		}
		if (bumperState[1] == kobuki_msgs::BumperEvent::PRESSED) //centre bumper
		{
			ROS_INFO("Centre Bumper PRESSED!!");
			bumperPressed = true;
		}
		if (bumperState[2] == kobuki_msgs::BumperEvent::PRESSED)
		{
			ROS_INFO("Right Bumper PRESSED!!");
			bumperPressed = true;
		}

		//check cliff sensor
		cliffDetected = false;
		if (cliffState[0] == kobuki_msgs::CliffEvent::CLIFF)
		{
			ROS_INFO("CLIFF on left!!");
			cliffDetected = true;
		}
		if (cliffState[1] == kobuki_msgs::CliffEvent::CLIFF)
		{
			ROS_INFO("CLIFF in front!!");
			cliffDetected = true;
		}
		if (cliffState[2] == kobuki_msgs::CliffEvent::CLIFF)
		{
			ROS_INFO("CLIFF on right!!");
			cliffDetected = true;
		}



		//---Actions to trigger world states---
		// DEFAULT: state 0 (follower)

		// 1. lose track of person - sad...

			// 45 degree sweeps(slow), dramatic sad music

		// 2. obstacle - surprise...!
		if (bumperPressed && world_state == 0) world_state = 2;
		
			// move back 
			// STOP (freezing)

		// 3. pick up & put down - RAGE!!!!

			// 

		// 4. pick up & hug - positively excited

			// 

		if(world_state == 0)
		{
			//fill with your code
			//vel_pub.publish(vel);
			vel_pub.publish(follow_cmd);

		}
		else if(world_state == 1)
		{
			/*
			...
			...
			*/
		}
		
		else if (world_state == 2)
		{
			ROS_INFO("SURPRISE!!");
			sc.playWave(path_to_sounds + "surprise2.wav");
			ros::Duration(0.5).sleep();

			vel.angular.z = 0;
			vel.linear.x = -1.0;
			vel_pub.publish(vel);
			ros::spinOnce();
			ROS_INFO("Backing up!!");

 			ros::Duration(0.5).sleep();


			vel.angular.z = 0;
			vel.linear.x = 0;
			vel_pub.publish(vel);
			ros::spinOnce();
			ROS_INFO("FROZEN...!");

			ros::Duration(5).sleep();

			break;


		}

		else if (world_state == 3)
		{
			
		}

		else if (world_state == 4)
		{
			
		}
		secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
		loop_rate.sleep();
	}

	return 0;
}
