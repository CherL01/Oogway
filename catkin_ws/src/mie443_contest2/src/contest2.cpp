#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <chrono>
#include <cmath>

#include <nav_msgs/GetPlan.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_datatypes.h>

#include "path_planning.cpp"

#include <iostream>
#include <fstream>
#include <string>
using namespace std;

float start_x, start_y, start_z, x,y,z; 

bool first = true; // check if first iteration or not

std::string outputPath = "/home/tuesday2023/Oogway/catkin_ws/src/mie443_contest2/boxes_database/Contest_2_Submission_0318.txt";
//std::string outputPath = "/home/harryp/catkin_ws/src/mie443_contest2/boxes_database";

float incAngle = 1, incNorm = 0.1;
float angStart=1, angEnd=45;
float normStart=0.48, normEnd=0.8, normSide=0.7;
float checkRight=M_PI/6, checkLeft=-M_PI/6;
float deltAngle=angStart, normDist=normStart; 

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
string getFinalOutput(int id);
int rb_repeat=0, ct_repeat=0, rk_repeat=0;

// Converts template_id values into chosen template
string getFinalOutput(int id) {

    if (rb_repeat>0 && id == 0) {
        return "(REPEAT) RAISIN_BRAN";
    } else if (ct_repeat>0 && id == 1) {
        return "(REPEAT) CINNAMON_TOAST";
    } else if (rk_repeat>0 && id == 2) {
        return "(REPEAT) RICE_KRISPIES";
    } else if (rb_repeat==0 && id == 0) {
        rb_repeat++;
        return "RAISIN_BRAN";
    } else if (ct_repeat==0 && id == 1) {
        ct_repeat++;
        return "CINNAMON_TOAST";
    } else if (rk_repeat==0 && id == 2) {
        rk_repeat++;
        return "RICE_KRISPIES";
    } else if (id == -1) {
        return "BOX IS BLANK!";
    } else {
        return "unidentified...";
    }

}


int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    // Robot pose object + subscriber.
    RobotPose robotPose(0,0,0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    ros::spinOnce();

    // Setup output text file
    std::ofstream contest2_file(outputPath);

    
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
    int final_output[5];    // stores template_id of each box in order
    array<string,5> template_names = {"N/A", "N/A", "N/A", "N/A", "N/A"};

    // Initialize path vector
    std::vector<int> min_path;

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
            ros::spinOnce();
            
            z = newBoxes.coords[i][2]+deltAngle/180*M_PI - M_PI;
            x = newBoxes.coords[i][0] + normDist*std::cos(newBoxes.coords[i][2]+deltAngle/180*M_PI);
            y = newBoxes.coords[i][1] + normDist*std::sin(newBoxes.coords[i][2]+deltAngle/180*M_PI);

            if (getPlan(robotPose.x, robotPose.y, robotPose.phi, x,y,z))
            {
                ROS_INFO("Valid Path! Updating coordinates to %f, %f, %f...", x,y,z);
                ROS_INFO("Delta angle is: %f and normal distance is: %f", deltAngle, normDist);
                //break;
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
                if (int(abs(deltAngle))%2==1) deltAngle = (abs(deltAngle)+incAngle)*(-1); //odd, positive deltangle
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

        if (first){
            first = false;
            ROS_INFO("First Iteration!");
            start_x = robotPose.x;
            start_y = robotPose.y;
            start_z = robotPose.phi;
            ROS_INFO("Starting Coordinates: (%f, %f, %f)", start_x, start_y, start_z);

            ROS_INFO("Localizing...");
            ROS_INFO("At: %f,%f,%f", robotPose.x, robotPose.y, robotPose.phi);
            Navigation::moveToGoal(start_x, start_y, start_z+ M_PI);

            ros::spinOnce();

            start_x = robotPose.x;
            start_y = robotPose.y;
            start_z = robotPose.phi- M_PI;
            


            ROS_INFO("Finished Localizing");
            ROS_INFO("Actual Starting Coordinates: (%f, %f, %f)", start_x, start_y, start_z);

            //Calculate shortest path
            ROS_INFO("Calculating Shortest path");
            std::vector<float> temp_vec = {start_x, start_y, start_z};
            std::vector<std::vector<float>> view_coords = newBoxes.coords;
            view_coords.push_back(temp_vec);
            std::vector<vector<float>> sorted_graph = sortGraph(view_coords);
            int start_node = view_coords.size() - 1;
            min_path = travellingSalesmanProblem(sorted_graph, start_node);
            std::cout << "MIN PATH SIZE:" << min_path.size() << std::endl;

        }

        else if (box_count == 5)
        {
            ROS_INFO("At: %f,%f,%f", robotPose.x, robotPose.y, robotPose.phi);
            ROS_INFO("GOING to: %f,%f,%f", start_x, start_y, start_z);

            Navigation::moveToGoal(start_x, start_y, start_z);
            secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();

            std::cout << "Finished in:" << std:: endl;
            std::cout << secondsElapsed << std::endl;
            break;
        }

        else
        {
        
            // replace box_count with min_path[box_count] from here on

            // If available, check and create waypoint at right side of box
            normDist = normSide;
            deltAngle = M_PI/2;
            z = boxes.coords[min_path[box_count]][2];
            x = boxes.coords[min_path[box_count]][0] + normDist*std::cos(boxes.coords[min_path[box_count]][2]+deltAngle);
            y = boxes.coords[min_path[box_count]][1] + normDist*std::sin(boxes.coords[min_path[box_count]][2]+deltAngle);

            ROS_INFO("Checking right side of box...");
            if (getPlan(robotPose.x, robotPose.y, robotPose.phi, x, y, z))
            {
                ROS_INFO("Right side available!");
                Navigation::moveToGoal(x,y,z);
                ROS_INFO("Reached right side");
            }
            
            else // If right side not available, check left side of box
            {
                ROS_INFO("Right side not available, checking left side of box...");
                deltAngle = -M_PI/2;
                z = boxes.coords[min_path[box_count]][2];
                x = boxes.coords[min_path[box_count]][0] + normDist*std::cos(boxes.coords[min_path[box_count]][2]+deltAngle);
                y = boxes.coords[min_path[box_count]][1] + normDist*std::sin(boxes.coords[min_path[box_count]][2]+deltAngle);
                if (getPlan(robotPose.x, robotPose.y, robotPose.phi, x, y, z))
                {
                    ROS_INFO("Left side available!");
                    Navigation::moveToGoal(x,y,z);
                    ROS_INFO("Reached left side");
                }
                else
                {
                    ROS_INFO("Left side not available either");
                }
            }
            ros::spinOnce();
            ROS_INFO("Heading to normal of box...");

            z = newBoxes.coords[min_path[box_count]][2];
            x = newBoxes.coords[min_path[box_count]][0];
            y = newBoxes.coords[min_path[box_count]][1];

            ROS_INFO("At: %f,%f,%f", robotPose.x, robotPose.y, robotPose.phi);
            ROS_INFO("GOING to: %f,%f,%f", x, y, z);

            if (Navigation::moveToGoal(x,y,z))
            {
                // image pipeline
                ros::spinOnce();
                final_output[min_path[box_count]] = imagePipeline.getTemplateID(boxes);

                box_count++;

            }
        }
        

        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        std::cout << "Time Elapsed:" << std:: endl;
        std::cout << secondsElapsed << std::endl;
        ros::Duration(0.01).sleep();
        
    }
    // make text file of final output (coords and template_id
    std::cout<< "Final output 1: " << final_output[0] << std::endl;
    std::cout<< "Final output 2: " << final_output[1] << std::endl;
    std::cout<< "Final output 3: " << final_output[2] << std::endl;
    std::cout<< "Final output 4: " << final_output[3] << std::endl;
    std::cout<< "Final output 5: " << final_output[4] << std::endl;

    for (int i=0;i<5;i++) {
        template_names[i] = getFinalOutput(final_output[i]);
        
        // Checks for repeats by keeping count
        if (final_output[i]==0) {
            rb_repeat++;
        } else if (final_output[i]==1) {
            ct_repeat++;
        } else if (final_output[i]==2) {
            rk_repeat++;
        }
    }

    contest2_file << "Names: Henry, Harry, Cherry, Alastair\n" << std::endl;
    for (int i=0;i<5;i++) {
        contest2_file << "Box "<< i+1 << "Tag: " << template_names[i] << std::endl;
        contest2_file << "Coordinates (x,y, phi): (" << boxes.coords[i][0] << ", " << boxes.coords[i][1] << ", " << boxes.coords[i][2] << ") \n" << std::endl;
    }

    //contest2_file.close();


    return 0;
}

// Edit costmap parameters

//roscd turtlebot_navigation/param
//sudo gedit costmap_common_params.yaml
