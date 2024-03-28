#include <header.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/CliffEvent.h>
#include <kobuki_msgs/WheelDropEvent.h>
#include <imageTransporter.hpp>
#include <chrono>
#include <inttypes.h>
#include <unistd.h>

using namespace std;

int counter = 0;

uint8_t bumperState[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
uint8_t cliffState[3] = {kobuki_msgs::CliffEvent::FLOOR, kobuki_msgs::CliffEvent::FLOOR, kobuki_msgs::CliffEvent::FLOOR};
uint8_t wheelState[2] = {kobuki_msgs::WheelDropEvent::DROPPED,kobuki_msgs::WheelDropEvent::DROPPED};
uint16_t cliffHeight = 0;    

bool bumperPressed;
bool cliffDetected;

std::chrono::time_point<std::chrono::system_clock> airTimeStart;
uint64_t airTime = 0;
uint64_t airTimeTotal = 0;
uint64_t maxAirTime = 20; // maximum air time in seconds

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

void wheelCB(const kobuki_msgs::WheelDropEvent::ConstPtr& msg){ //left is 0, right is 1
	wheelState[msg->wheel] = msg->state;

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
	ros::Subscriber wheelDrop = nh.subscribe("mobile_base/events/wheel_drop", 10, &wheelCB);

    // contest count down timer
	ros::Rate loop_rate(10);
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

	imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	//imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //--for turtlebot Camera
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

	int world_state = 3; //3 for testing cliff height

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
		ROS_INFO("Cliff Height: %" PRIu16, cliffHeight);

		/*
		if (cliffHeight > 50)
		{
			if (!cliffDetected) airTimeStart = std::chrono::system_clock::now();
			cliffDetected = true;
			ROS_INFO("LIFTED UP!");
		}

		else
		{
			if (cliffDetected) airTimeTotal = airTime;
			cliffDetected = false;
			airTime = 0;
			ROS_INFO("On ground, safe to proceed...");
		}*/

		//check wheel sensor
		bool robotRaised = false;
		if (wheelState[0] == kobuki_msgs::WheelDropEvent::RAISED && wheelState[1]==kobuki_msgs::WheelDropEvent::RAISED) 
		{
			robotRaised = true;
			ROS_INFO("LIFTED UP!!!");
			break;
		}

		else robotRaised = false;



		//---Actions to trigger world states---
		// DEFAULT: state 0 (follower)

		// 1. lose track of person - sad...

			// 45 degree sweeps(slow), dramatic sad music

		// 2. obstacle - surprise...!
		if (bumperPressed && world_state == 0) world_state = 2;
		
			// move back 
			// STOP (freezing)

		// 3. pick up & put down - ANGRY...
		if (cliffDetected>0 && (airTimeTotal<maxAirTime)) world_state = 3; //total air time is less than some period
			// wait until placed down -> 

		// 4. pick up & shake - RAGE!!!
		if (cliffDetected>0 && (airTimeTotal>=maxAirTime)) world_state = 4; //second time lifted up, 

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

			break; //world_state = 0;


		}

		else if (world_state == 3)
		{
			ROS_INFO("Time in air: %" PRIu64, airTime);
			ROS_INFO("Total time in air: %" PRIu64, airTimeTotal);

			// if cliffHeight -wait
			// else (on ground)
				// music = angry music & image 
				// 
				
			// exit to state 0


		}

		else if (world_state == 4)
		{
			
		}

		cliffHeight = 0; // reset cliffHeight to 0 every loop

		if (cliffDetected) airTime = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-airTimeStart).count();
		secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
		loop_rate.sleep();
	}

	return 0;
}
