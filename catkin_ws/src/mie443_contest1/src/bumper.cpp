#include "bumper.h"

uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};

uint8_t leftState = bumper[kobuki_msgs::BumperEvent::LEFT];
uint8_t rightState = bumper[kobuki_msgs::BumperEvent::RIGHT];
uint8_t centerState = bumper[kobuki_msgs::BumperEvent::CENTER];
bool anyBumperPressed = false;

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
    bumper[msg->bumper] = msg->state;
    leftState = bumper[0];
    centerState = bumper[1];
    rightState = bumper[2];

    ROS_INFO("Left Bumper: %i, Center Bumper: %i, Right Bumper: %i", leftState, centerState, rightState);
}