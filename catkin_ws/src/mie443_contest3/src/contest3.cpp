#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <chrono>
#include "bumper.h"

using namespace std;

geometry_msgs::Twist follow_cmd;
int world_state;

uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
uint8_t bumperLeftState = bumper[kobuki_msgs::BumperEvent::LEFT];
uint8_t bumperRightState = bumper[kobuki_msgs::BumperEvent::RIGHT];
uint8_t bumperCenterState = bumper[kobuki_msgs::BumperEvent::CENTER];

uint8_t cliff[3] = {kobuki_msgs::CliffEvent::RELEASED, kobuki_msgs::CliffEvent::RELEASED, kobuki_msgs::CliffEvent::RELEASED};
uint8_t cliffLeftState = cliff[kobuki_msgs::CliffEvent::LEFT];
uint8_t cliffRightState = cliff[kobuki_msgs::CliffEvent::RIGHT];
uint8_t cliffCenterState = cliff[kobuki_msgs::CliffEvent::CENTER];

void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
}

void bumperCB(const kobuki_msgs::BumperEvent::ConstPtr& msg){
    //Fill with code
	bumper[msg->bumper] = msg->state;
    bumperLeftState = bumper[0];
    bumperCenterState = bumper[1];
    bumperRightState = bumper[2];
    ROS_INFO("Left Bumper: %i, Center Bumper: %i, Right Bumper: %i", bumperLeftState, bumperCenterState, bumperRightState);

}

void cliffCB(const kobuki_msgs::CliffEvent::ConstPtr& msg) {
	cliff[msg->cliff] = msg->state;
	cliffLeftState = cliff[0];
	cliffCenterState = cliff[1];
	cliffRightState = cliff[2];
	ROS_INFO("L Cliff Sensor: %i, C Cliff Sensor: %i, R Cliff Sensor: %i", cliffLeftState, cliffCenterState, cliffRightState);

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

	int world_state = 0;

	double angular = 0.2;
	double linear = 0.0;

	geometry_msgs::Twist vel;
	vel.angular.z = angular;
	vel.linear.x = linear;

	sc.playWave(path_to_sounds + "sound.wav");
	ros::Duration(0.5).sleep();

	while(ros::ok() && secondsElapsed <= 480){		
		ros::spinOnce();

		// left and right bumpers activated, and cliff sensors -> world state 4
		// need to subscribe to cliff topic

		if(world_state == 0){
			//fill with your code
			//vel_pub.publish(vel);
			vel_pub.publish(follow_cmd);

		}else if(world_state == 1){
			/*
			...
			...
			*/
		}else if (world_state == 4){
			// left and right bumpers activated
			// positively excited
			// play some sounds

		}
		secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
		loop_rate.sleep();
	}

	return 0;
}
