#include "globals.h"
#include "laserCallback.cpp"
#include "move.cpp"
#include "bumper.cpp"

//state globals

bool local = false;
bool isMoving = false;

int BUMPER_STEP = 0;
int TRAVEL_STEP = 1;
int SCAN_STEP = 2;

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


    while(ros::ok()) {

        ros::spinOnce();

        //check if movement is complete

        isMoving = (angular != 0.0 || linear != 0.0);
        ROS_INFO("Angular speed: %f, Linear speed: %f", angular, linear);


        //check if bumper is hit -> might need to add this to the moving function
        for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
            anyBumperPressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED); //iterate through bumper, check if pressed 
        }

        //When Oogway is stationary, give directions
        if (!isMoving)
        {
            //move forward
            if (stepsCount == BUMPER_STEP || anyBumperPressed) //hitting bumpers
            {
                currentX = posX;
                currentY = posY;
                targetDist = 0.2;

                moveFront(targetDist, currentX, currentY, angular, linear);
                stepsCount = SCAN_STEP;
            }

            else if (stepsCount == TRAVEL_STEP) //travelling
            {
                ;
            }

            else if (steps_Count == SCAN_STEP)
            {

            }

            /*//PSEUDOCODE
            ROS_INFO("steps: %d", stepsCount);
            if (stepsCount == 0) //first time in isMoving loop
            {
                stepsCount++;
                
                
            }

            else if (stepsCount == 1)
            {
                stepsCount++;
            
            }
            
            else if (stepsCount == 2) ROS_INFO("COMPLETED MOVEMENT");
            PSEUDOCODE */
        }

        //When Oogawy is moving, monitor motion until it stops
        //linear & angular are referenced variables, which can be updated from each "Check" functions
        else //isMoving
        {
            ROS_INFO("Remaining yaw outside checkturn: %f", remainingYaw);
            //CHECK IF MOVEMENT IS DONE
            ROS_INFO("Moving...");
            
            if (linear != 0)
            {
                if (linear > 0) checkMoveFront(targetDist, currentX, currentY, angular, linear);
                else checkMoveBack(targetDist, currentX, currentY, angular, linear);
            }

            if (angular != 0)
            {
                if (angular > 0) checkTurnCCW(targetYaw, angular, linear, remainingYaw);
                else checkTurnCW(targetYaw, angular, linear, remainingYaw);

            }

            //UPDATE SPEED BASED ON LASER READINGS GO HERE
            //if linear or angular is not 0
        
        }

        

        /* PSEUDOCODE
        //check bumpers first
        for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
            any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED); //iterate through bumper, check if pressed 
        }

        if (anyBumperPressed)
        {
            bumperCount += 1;
            //BUMPER STATE (hit a bumper)
            

            //EXIT
            if ()
            {
                bumperState = false;
                bumperCount = 0;
            }
        }

        if (travelState)
        {
            travelCount += 1;
            //TRAVEL STATE (random exploring)

            

            //speed mod if laser too close

            //EXIT if bumper/ travelCount is too large
                //travelCount = 0;
                //scanState = true;
                //travelState = false;
        }

        //IF travelCount >x || bumperCount >x || timeElapsed 5min, 7min: scanState = true

        if (scanState)
        {
            //SCAN STATE (scan surroundings and redirect)
        }PSEUDOCODE*/
        



        
        

      

        vel.linear.x = linear;
        vel.angular.z = angular;
        vel_pub.publish(vel);
        
        // The last thing to do is to update the timer.
        //secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
        }

    return 0;
}
