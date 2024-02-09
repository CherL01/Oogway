#include "globals.h"
#include "laserCallback.cpp"
#include "move.cpp"
#include "bumper.cpp"

//state globals
//WORKING CODE
bool local = false;
bool isMoving = false;

int TRAVEL_STEP = 1;
int SCAN_STEP = 0;

int travelCount = 0;
int bumperCount = 0;

int stepsCount = 0;
int subStepsCount = 0;

//uint8_t is a boolean variable


//float is 32-bit decimal 

//float dist = 0.0;
float targetDist = 0.0;
//float remainingDist = 0.0;

float currentX = 0.0;
float currentY = 0.0;

float openYaw = 0.0;

float maxLaserDist = 0.0;

int travelLoop = 0;
int travelLoopLimit = 10;

float turtleSpeed = 0.0;

uint64_t travelElapsed = 0;

std::chrono::time_point<std::chrono::system_clock> travelStart;

float turnAngle = 10.0;






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

    //contest countdown timer

    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;


    float angular = 0.0;
    float linear = 0.0;

    int turning = 0;

    //print statement to differentiate local & oogway
        
    if (local)
    {
        ROS_INFO("This is Local, not uploaded to github");
    }
    else
    {
        ROS_INFO("This is Oogway, uploaded to github");
    }


    while(ros::ok() && secondsElapsed <= 480) {

        ros::spinOnce();

        ROS_INFO("substep: %d", subStepsCount);


        //PRELIMINARY CHECKINGS
        //check if movement is complete

        isMoving = (angular != 0.0 || linear != 0.0);
        ROS_INFO("Angular speed: %f, Linear speed: %f", angular, linear);

        //check if bumper is hit -> might need to add this to the moving function

        bool anyBumperPressed = false;
        for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
            anyBumperPressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED); //iterate through bumper, check if pressed 
        }

        ROS_INFO("Bumper is pressed: %d", anyBumperPressed);

        //check for speed limit
        if (leftLaserDist < slowDownLimit || rightLaserDist < slowDownLimit || minLaserDist < slowDownLimit)
        {
            turtleSpeed = slowDown;
            ROS_INFO("SLOWING DOWN...");
        }

        else 
        {
            turtleSpeed = normal;
            ROS_INFO("NORMAL SPEED");
        }

        //ADDING RANDOMNESS TO MOVEMENT -> SCAN every interval, after travelling for certain intervals, gittering at corners
        //scan at 300 & 450
        if (secondsElapsed % 150 == 0 && secondsElapsed != 0 && secondsElapsed != 150)
        {
            subStepsCount = 0;
            stepsCount = SCAN_STEP;
            turnAngle = 15.0;
        }

        //When Oogway is stationary, give directions
        if (!isMoving)
        {
            

            if (stepsCount == SCAN_STEP)
            {
                ROS_INFO("SCANNING...");

                if (subStepsCount == 0) 
                [
                    ROS_INFO("Correcting Yaw! Current Yaw: %f", yaw); // correcting yaw:it initially is 0
                    subStepsCount++;
                ]
                
                
                else if (subStepsCount == 1) // initially turn 180 ccw
                {
                    //targetYaw = yaw +180;
                    //turnCCW(targetYaw, angular, linear, remainingYaw);

                    ///* comment block for 
                    //test 1
                    currentX = posX;
                    currentY = posY;
                    targetDist = 0.01; //0.01, 0.05, 0.1, 0.2
                    moveFront(targetDist, currentX, currentY, angular, linear, turtleSpeed);

                    //test 2
                    targetYaw = yaw +90; //90, 450, 10, 15 
                    turnCCW(targetYaw, angular, linear, remainingYaw);
                    
                    //*/
                    subStepsCount++;
                    
                } 

                else if (subStepsCount == 2) // turn another 180 ccw for 1 full loop
                {
                    //*/
                    targetYaw = yaw +180;
                    turnCCW(targetYaw, angular, linear, remainingYaw);
//*/

                    subStepsCount++;
                }

                else if (subStepsCount == 3)
                {
                    ///* comment block for test
                    targetYaw = openYaw;
                    turnCCW(targetYaw, angular, linear, remainingYaw);

                    //*/
                    subStepsCount = 0;
                    stepsCount = TRAVEL_STEP;
                    maxLaserDist = 0.0;
                    ROS_INFO("Entering Travel mode");

                    std::chrono::time_point<std::chrono::system_clock> travelStart;
                    travelStart = std::chrono::system_clock::now();
                    travelLoop = 0;
                    
                }


            }

            else if (stepsCount == TRAVEL_STEP)
            {
                

                ROS_INFO("TRAVELLING...");
                std::cout << "Time Elapsed:" << std:: endl;
                std::cout << travelElapsed;
                if (leftLaserDist > stopLimit && minLaserDist > stopLimit && rightLaserDist > stopLimit)
                // cleared from stoplimit
                {
                    currentX = posX;
                    currentY = posY;
                    targetDist = 0.01;
                    moveFront(targetDist, currentX, currentY, angular, linear, turtleSpeed);
                    travelLoop = 0;
                    subStepsCount = 0;
                }
                else 
                {
                    ROS_INFO("STOP MOTION OCCURED!");
                    ROS_INFO("Travel Loop: %d", travelLoop);
                    //break;
                    if (travelLoop >= travelLoopLimit)
                    {
                        ROS_INFO("Too much Gittering");
                        subStepsCount = 0;
                        stepsCount = SCAN_STEP;
                    }

                    if (subStepsCount == 0)
                    {
                        linear = 0;
                        angular = 0;
                        subStepsCount++;
                    }
                    
                    else
                    {
                        if (leftLaserDist > rightLaserDist)
                        {
                            targetYaw = yaw +turnAngle;
                            turnCCW(targetYaw, angular, linear, remainingYaw);
                            travelLoop++;
                        }

                        else
                        {
                            targetYaw = yaw -turnAngle;
                            turnCW(targetYaw, angular, linear, remainingYaw);
                            travelLoop++;
                        }
                    }

                    
                    ROS_INFO("Travel Loop: %d", travelLoop);
                }

            }
        }

        else if (anyBumperPressed) //when bumper is hit, stop and move back. (bumper usually hit when moving)
            {
                ROS_INFO("BANG!!! Bumper Pressed");
                
                currentX = posX;
                currentY = posY;
                targetDist = 0.2;

                moveBack(targetDist, currentX, currentY, angular, linear, turtleSpeed);
                stepsCount = SCAN_STEP;
                subStepsCount = 0;

            }

        //laser limit in else ifelse if ()


        else //isMoving
        {
            //ROS_INFO("Remaining yaw outside checkturn: %f", remainingYaw);
            //CHECK IF MOVEMENT IS DONE
            ROS_INFO("Moving...");

            //record yaw with most opening during 360 scan

            if (stepsCount == SCAN_STEP && subStepsCount != 0)
            {
                ROS_INFO("recording yaw...");
                if (maxLaserDist < minLaserDist && minLaserDist != std::numeric_limits<float>::infinity())
                {
                    maxLaserDist = minLaserDist;
                    openYaw = yaw;
                    ROS_INFO("Max yaw is: %f", openYaw);
                }
            }

            //Checking if movement is done
            
            if (linear != 0)
            {
                if (linear > 0) 
                {
                    checkMoveFront(targetDist, currentX, currentY, angular, linear, turtleSpeed);
                    
                }
                else 
                {
                    checkMoveBack(targetDist, currentX, currentY, angular, linear, turtleSpeed);

                }

                
            }

            if (angular != 0)
            {
                if (angular > 0) checkTurnCCW(targetYaw, angular, linear, remainingYaw);
                else checkTurnCW(targetYaw, angular, linear, remainingYaw);

            }
        
        
        }
      

        vel.linear.x = linear;
        vel.angular.z = angular;
        vel_pub.publish(vel);


        //Update timer of elapsed travel time
        if (stepsCount == TRAVEL_STEP)
        {
            travelElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-travelStart).count();
        }
        
        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        std::cout << "Time Elapsed:" << std:: endl;
        std::cout << secondsElapsed;
        loop_rate.sleep();
        }

    return 0;
}
