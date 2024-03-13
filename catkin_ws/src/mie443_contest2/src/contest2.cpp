#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <chrono>
#include <cmath>

#include <nav_msgs/GetPlan.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_datatypes.h>

float start_x, start_y, start_z, x,y,z; 
float deltAngle=1, normDist=0.5; 

float incAngle = 1, incNorm = 0.1;
float angStart=1, angEnd=15;
float normStart=0.5, normEnd=0.8;

bool getPlan(float xStart, float yStart, float phiStart, float xGoal, float yGoal, float phiGoal){
	// Set up and wait for actionClient.

    // Initialize node
    ros::NodeHandle n;
    ros::ServiceClient check_path = n.serviceClient<nav_msgs::GetPlan>("/move_base/NavfnROS/make_plan");
    nav_msgs::GetPlan srv;

    // Set start
    geometry_msgs::Quaternion phistart = tf::createQuaternionMsgFromYaw(phiStart);
    srv.request.start.header.frame_id = "map";
    srv.request.start.pose.position.x = xStart;
    srv.request.start.pose.position.y = yStart;
    srv.request.start.pose.position.z = 0.0;
    srv.request.start.pose.orientation.x = 0;
    srv.request.start.pose.orientation.y = 0;
    srv.request.start.pose.orientation.z = phistart.z;
    srv.request.start.pose.orientation.w = phistart.w;

    geometry_msgs::Quaternion phigoal = tf::createQuaternionMsgFromYaw(phiGoal);
    srv.request.goal.header.frame_id = "map";
    srv.request.goal.pose.position.x = xGoal;
    srv.request.goal.pose.position.y = yGoal;
    srv.request.goal.pose.position.z = 0.0;
    srv.request.goal.pose.orientation.x = 0.0;
    srv.request.goal.pose.orientation.y = 0.0;
    srv.request.goal.pose.orientation.z = phigoal.z;
    srv.request.goal.pose.orientation.w = phigoal.w;
    srv.request.tolerance = 0.1;

    check_path.call(srv);
    
	
    
    return srv.response.plan.poses.size()>0;
}

bool results;

int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    // Robot pose object + subscriber.
    RobotPose robotPose(0,0,0);

    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    // Initialize box coordinates and templates
    Boxes boxes; 
    if(!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    for(int i = 0; i < boxes.coords.size(); ++i) {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: " 
                  << boxes.coords[i][2] << std::endl;
    }
    

    // Initialize image objectand subscriber.
    ImagePipeline imagePipeline(n);

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;
    int box_count = 0;


    // Initialize second set of box coordinates to check for valid coordinates
    Boxes newBoxes;
    if (!newBoxes.load_coords() || !boxes.load_templates())
    {
        std::cout << "ERROR: could not load new coords or templates"<< std::endl;
        return -1;
    }
    // Calculate valid coordinates
    for (int i=0; i < newBoxes.coords.size(); i++)
    {
        ROS_INFO("Starting Box coordinate %d", i);
        z = newBoxes.coords[i][2] - M_PI;
        x = newBoxes.coords[i][0] + normDist*std::cos(newBoxes.coords[i][2]);
        y = newBoxes.coords[i][1] + normDist*std::sin(newBoxes.coords[i][2]);

        
        while (!getPlan(robotPose.x, robotPose.y, robotPose.phi, x,y,z))
        //while (true)
        {
            
            z = newBoxes.coords[i][2]+deltAngle/180*M_PI - M_PI;
            x = newBoxes.coords[i][0] + normDist*std::cos(newBoxes.coords[i][2]+deltAngle/180*M_PI);
            y = newBoxes.coords[i][1] + normDist*std::sin(newBoxes.coords[i][2]+deltAngle/180*M_PI);

            if (getPlan(robotPose.x, robotPose.y, robotPose.phi, x,y,z))
            {
                ROS_INFO("Valid Path! Updating coordinates to %f, %f, %f...", x,y,z);
                ROS_INFO("Delta angle is: %f and normal distance is: %f", deltAngle, normDist);
                break;
            }
            else 
            {
                ROS_INFO("Invalid Path to %f, %f, %f, not updating coordinates", x,y,z);
                ROS_INFO("Delta angle is: %f and normal distance is: %f", deltAngle, normDist);
            }

            // break statement when delta angles exceeds 15 and normal exceeds 0.8
            if (deltAngle > angEnd) 
            {
                normDist += incNorm;
                deltAngle = angStart;
                ROS_INFO("Updating normal distance");
                
            }

            // increment delta angle
            else
            {
                if (abs(deltAngle)%2==1) deltAngle = (abs(deltAngle)+incAngle)*(-1); //odd, positive deltangle
                else deltAngle = (abs(deltAngle)+incAngle); //even, negative deltangle
            }

            if (deltAngle >angEnd && normDist >normEnd) 
            {
                deltAngle = angStart;
                normDist = normStart;
                break;
            }
            
        }

        // Update boxes.coords with valid poses
        newBoxes.coords[i][0] = x;
        newBoxes.coords[i][1] = y;
        newBoxes.coords[i][2] = z;            
    }

    // print updated box coordinates
    for(int i = 0; i < newBoxes.coords.size(); ++i) {
        std::cout << "Updated Box coordinates: " << std::endl;
        std::cout << i << " x: " << newBoxes.coords[i][0] << " y: " << newBoxes.coords[i][1] << " z: " 
                << newBoxes.coords[i][2] << std::endl;
    }
    


    // Execute strategy.
    while(ros::ok() && secondsElapsed <= 300) {
        
        ros::spinOnce();
        
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi

        if (box_count == 0){
            start_x = robotPose.x;
            start_y = robotPose.y;
            start_z = robotPose.phi;
            Navigation::moveToGoal(start_x, start_y, start_z+ M_PI);
        }
        
        z = newBoxes.coords[box_count][2];
        x = newBoxes.coords[box_count][0];
        y = newBoxes.coords[box_count][1];
        
        
        ROS_INFO("At: %f,%f,%f", robotPose.x, robotPose.y, robotPose.phi);
        ROS_INFO("GOING to: %f,%f,%f", x, y, z);

        bool success = Navigation::moveToGoal(x,y,z);
        if (success) box_count++;

        
        
        if (box_count == 5) 
        {   
            Navigation::moveToGoal(start_x, start_y, start_z);
            secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();

            std::cout << "Finished in:" << std:: endl;
            std::cout << secondsElapsed << std::endl;
            break;
        }
        //imagePipeline.getTemplateID(boxes);*/

        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        std::cout << "Time Elapsed:" << std:: endl;
        std::cout << secondsElapsed << std::endl;
        ros::Duration(0.01).sleep();
        
    }
    return 0;
}
