#include "globals.h"
#include "laserCallback.cpp"
#include "move.cpp"
#include "bumper.cpp"

bool local = true;

//// Initialize states & loop counts
bool isMoving = false;

int stepsCount = 0;
int subStepsCount = 0;
int travelLoop = 0;
uint64_t travelTimeLimit = 12;


//// Initialize variables for movement
float targetDist = 0.0;

float currentX = 0.0;
float currentY = 0.0;

float openYaw = 0.0;
float prevYaw = 0.0;

float turnAngle = 10.0; 

float turtleSpeed = 0.0;
float turtleAngle = 0.0;

float maxLaserDist = 0.0; //variable to store recorded yaw during travel

//// Timer variables to record travel time
uint64_t travelElapsed = 0;
std::chrono::time_point<std::chrono::system_clock> travelStart;

 

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

    // Contest countdown timer

    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    // Initiallize angular and linear velocities
    float angular = 0.0;
    float linear = 0.0;

    // Initialize to scan state & substep 0
    stepsCount = SCAN_STEP;
    subStepsCount = 0;

    // Print statement to differentiate local and github repositories
        
    if (local) ROS_INFO("This is Local, not uploaded to github");
    else ROS_INFO("This is Oogway, uploaded to github");


    while(ros::ok() && secondsElapsed <= 480) {

        ros::spinOnce();
        ROS_INFO("Substep: %d", subStepsCount);
        ROS_INFO("FRONT: %g", minLaserDist);
        ROS_INFO("RIGHT-END: %g", rightLaserDist);
        ROS_INFO("LEFT-END: %g", leftLaserDist);

        //PRELIMINARY CHECKINGS

        //// Check if movement is complete

        isMoving = (angular != 0.0 || linear != 0.0);
        ROS_INFO("Angular speed: %f, Linear speed: %f", angular, linear);

        //// Check if bumper is hit

        bool anyBumperPressed = false;
        for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
            anyBumperPressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED); //iterate through bumper, check if pressed 
        }

        ROS_INFO("Bumper is pressed: %d", anyBumperPressed);

        //// Check for speed limit

        if (leftLaserDist < slowDownLimit || rightLaserDist < slowDownLimit || minLaserDist < slowDownLimit)
        {
            turtleSpeed = slowDown;
            turtleAngle = slowDownAngular;
            travelTimeLimit = 30;
            turnAngle = 20;

            ROS_INFO("SLOWING DOWN...");
        }

        else 
        {
            turtleSpeed = normal;
            turtleAngle = normalAngular;
            travelTimeLimit = 15;
            turnAngle = 10;
            ROS_INFO("NORMAL SPEED");
        }



        //When Oogway is stationary, give directions
        if (!isMoving)
        {
            
            // SCANNING STEP
            if (stepsCount == SCAN_STEP)
            {

                ROS_INFO("SCANNING...");

                //skip first loop as yaw isn't updated yet
                if (subStepsCount == 0) 
                {
                    subStepsCount++;
                } 

                // initially turn 180 ccw                
                else if (subStepsCount == 1) 
                {
                    ROS_INFO("YAW::::: %f", yaw);

                    targetYaw = yaw +180;
                    turnCCW(targetYaw, angular, linear, remainingYaw, turtleAngle);

                    subStepsCount++;
                    
                } 
                
                // turn another 180 ccw for 1 full loop
                else if (subStepsCount == 2) 
                {

                    targetYaw = yaw +180;
                    turnCCW(targetYaw, angular, linear, remainingYaw, turtleAngle);

                    subStepsCount++;
                }

                // direct to desired yaw and exit to travel mode
                else if (subStepsCount == 3)
                {
                    targetYaw = openYaw;

                    if (targetYaw > yaw)
                    {
                        if (yaw < 180 && yaw+360 - targetYaw < 180) turnCW(targetYaw, angular, linear, remainingYaw, turtleAngle);
                        else turnCCW(targetYaw, angular, linear, remainingYaw, turtleAngle);
                    }

                    else 
                    {
                        if (yaw >= 180 && targetYaw+360 - yaw < 180) turnCCW(targetYaw, angular, linear, remainingYaw, turtleAngle);
                        else turnCW(targetYaw, angular, linear, remainingYaw, turtleAngle);
                    }
                    
                    

                    subStepsCount = 0;
                    stepsCount = TRAVEL_STEP;
                    maxLaserDist = 0.0;
                    ROS_INFO("Entering Travel mode");


                    travelLoop = 0;
                    
                }


            }

            // TRAVELLING STEP
            else if (stepsCount == TRAVEL_STEP)
            {

                ROS_INFO("TRAVELLING (NOT MOVING YET)...");

                //Iniitate clock for timeout
                if (subStepsCount == 0)
                {
                    travelStart = std::chrono::system_clock::now();
                    subStepsCount++;
                }

                //give travel instructions - go straight
                else if (subStepsCount == 1) 
                {
                    linear = turtleSpeed;
                    angular = 0.0;
                    
                }

                // STOP LIMIT REACHED: instruct to turn left or right
                else if (subStepsCount == 2) 
                {
                    if (travelLoop >= travelLoopLimit) // put this last
                    {
                        ROS_INFO("Too much Gittering");
                        subStepsCount = 0;
                        travelLoop = 0;
                        stepsCount = SCAN_STEP;
                        turnAngle = 10;
                    }
                    if (travelLoop >= 6)
                    {
                        ROS_INFO("Too much Gittering");
                        turnAngle = 5;
                    }
                    /// rotate if travel loop limit not reached
                    /*
                    if (minLaserDist > rightLaserDist && minLaserDist > leftLaserDist)
                    {
                        
                    }*/
                    if (leftLaserDist > rightLaserDist)
                    {
                        targetYaw = yaw +turnAngle;
                        turnCCW(targetYaw, angular, linear, remainingYaw, turtleAngle);
                        travelLoop++;
                        
                    }

                    else
                    {
                        targetYaw = yaw -turnAngle;
                        turnCW(targetYaw, angular, linear, remainingYaw, turtleAngle);
                        travelLoop++;
                    }

                }

                // turn 360 if timeout limit
                else if (subStepsCount == 3)
                {
                    ROS_INFO("!!!!!!!!!!!!!!!TIME LIMIT!!!!!!!!!!!!!");

                    if (rightLaserDist > leftLaserDist)
                    {
                        targetYaw = yaw -90;
                        turnCW(targetYaw, angular, linear, remainingYaw, turtleAngle);
                    }

                    else 
                    {
                        targetYaw = yaw +90;
                        turnCCW(targetYaw, angular, linear, remainingYaw, turtleAngle);

                    }
                    
                    subStepsCount++;
                }

            

            }
        }

        //When bumper is hit while moving, stop and move back
        else if (anyBumperPressed)
            {
                ROS_INFO("BANG!!! Bumper Pressed");
                
                currentX = posX;
                currentY = posY;
                targetDist = 0.2;

                moveBack(targetDist, currentX, currentY, angular, linear, turtleSpeed);
                stepsCount = SCAN_STEP;
                subStepsCount = 0;

            }

        //Monitor Travelling
        else if (stepsCount == TRAVEL_STEP)
        {
            ROS_INFO("Travelling...");

            //if timeout reached & after 300s passed, turn 360
            if (travelElapsed > travelTimeLimit && secondsElapsed > 240)
                {
                    travelElapsed = 0.0;
                    subStepsCount = 3;
                    linear = 0.0;
                    angular = 0.0;
                }


            //complete scanning motion after transitioning from scanning state
            if (subStepsCount == 0)
            {  
                if (angular > 0) checkTurnCCW(targetYaw, angular, linear, remainingYaw, turtleAngle);
                else checkTurnCW(targetYaw, angular, linear, remainingYaw, turtleAngle);
            }

            // Check for laserScan detection (obstacle avoidance)
            if (subStepsCount == 1)
            {
                if (leftLaserDist < slowDownLimit || minLaserDist < slowDownLimit || rightLaserDist < slowDownLimit) 
                {
                    
                    angular = 0.0;
                    linear = turtleSpeed;
                }

                if (leftLaserDist < stopLimit || minLaserDist < stopLimit || rightLaserDist < stopLimit) 
                {
                    subStepsCount++;
                    angular = 0.0;
                    linear = 0.0;
                }
            }
            
            //obstacle avoidance - check if turning is complete
            else if (subStepsCount == 2)
            {

                if (angular != 0)
                {
                    if (angular > 0) checkTurnCCW(targetYaw, angular, linear, remainingYaw, turtleAngle);
                    else checkTurnCW(targetYaw, angular, linear, remainingYaw, turtleAngle);
                } 


                // Go back to travelling state if laser scan clears
                if (leftLaserDist > clearLimit && minLaserDist > clearLimit && rightLaserDist > clearLimit)
                {
                    angular = 0.0;
                    linear = 0.0;
                    subStepsCount--;
                    travelLoop = 0;
                    travelStart = std::chrono::system_clock::now();
                }

                
            }

            //Complete turning motion during 360 sweep
            else if (subStepsCount == 4)
            {
                ROS_INFO("TURNING AFTER TIME LIMIT REACHED");
                if (angular>0) checkTurnCCW(targetYaw, angular, linear, remainingYaw, turtleAngle);
                else checkTurnCW(targetYaw, angular, linear, remainingYaw, turtleAngle);
                
                if (angular==0) subStepsCount=0;
            }



        }

        //isMoving
        else 
        {
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
                if (linear > 0) checkMoveFront(targetDist, currentX, currentY, angular, linear, turtleSpeed);
                else checkMoveBack(targetDist, currentX, currentY, angular, linear, turtleSpeed);
            }

            if (angular != 0)
            {
                if (angular > 0) checkTurnCCW(targetYaw, angular, linear, remainingYaw, turtleAngle);
                else checkTurnCW(targetYaw, angular, linear, remainingYaw, turtleAngle);
            }       
        }
      

        vel.linear.x = linear;
        vel.angular.z = angular;
        vel_pub.publish(vel);


        //Update timer of elapsed travel time
        if (stepsCount == TRAVEL_STEP && subStepsCount > 0 && subStepsCount < 3)
        {
            travelElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-travelStart).count();
        }
        if (stepsCount == TRAVEL_STEP && subStepsCount == 2)
        {
            travelElapsed = 0;
        }
        
        //Update the timer
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        std::cout << "Time Elapsed:" << std:: endl;
        std::cout << secondsElapsed << std:: endl;
        std::cout << "Travel Time Elapsed:" << std:: endl;
        std::cout << travelElapsed << std::endl;
        std::cout << "Travel Loop:" << std:: endl;
        std::cout << travelLoop << std::endl;
        
        loop_rate.sleep();
        }

    return 0;
}
