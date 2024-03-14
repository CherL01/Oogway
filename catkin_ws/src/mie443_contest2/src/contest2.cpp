// #include "path_planning.cpp"

// #include <boxes.h>
// #include <navigation.h>
// #include <robot_pose.h>
// #include <imagePipeline.h>
// #include <chrono>
// #include <cmath>
// // #include <path_planning.h>


// int main(int argc, char** argv) {
//     // Setup ROS.
//     ros::init(argc, argv, "contest2");
//     ros::NodeHandle n;
//     // Robot pose object + subscriber.
//     RobotPose robotPose(0,0,0);

//     ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
//         // Initialize box coordinates and templates
//     Boxes boxes; 
//     if(!boxes.load_coords() || !boxes.load_templates()) {
//         std::cout << "ERROR: could not load coords or templates" << std::endl;
//         return -1;
//     }
//     for(int i = 0; i < boxes.coords.size(); ++i) {
//         std::cout << "Box coordinates: " << std::endl;
//         std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: " 
//                   << boxes.coords[i][2] << std::endl;
//     }
//     // Initialize image objectand subscriber.
//     ImagePipeline imagePipeline(n);
//     // contest count down timer
//     std::chrono::time_point<std::chrono::system_clock> start;
//     start = std::chrono::system_clock::now();
//     uint64_t secondsElapsed = 0;
//     int box_count = -1;
//     float start_x;
//     float start_y;
//     float start_z;
//     std::vector<int> min_path;

//     // calculate viewing location coordinates
//     std::vector<std::vector<float>> view_coords;
//     for (int box = 0; box < boxes.coords.size(); box++) {
//         float box_z = boxes.coords[box][2] - M_PI; // -2.3
//         float box_x = boxes.coords[box][0] + 0.5*std::cos(boxes.coords[box][2]); //2.42;
//         float box_y = boxes.coords[box][1] + 0.5*std::sin(boxes.coords[box][2]); //-0.894
//         std::vector<float> temp_vec = {box_x, box_y, box_z};

//         view_coords.push_back(temp_vec);
//     }
//     std::cout << "VIEW COORDS SIZE:" << view_coords.size() << std::endl;

//     // Execute strategy.
//     while(ros::ok() && secondsElapsed <= 300) {
//         ros::spinOnce();
//         /***YOUR CODE HERE***/
//         // Use: boxes.coords
//         // Use: robotPose.x, robotPose.y, robotPose.phi

//         // startup
//         if (box_count == -1){
//             ROS_INFO("STARTUP");
//             start_x = robotPose.x;
//             start_y = robotPose.y;
//             start_z = robotPose.phi;
//             Navigation::moveToGoal(start_x, start_y, start_z+ M_PI);
            
//             // calculate shortest path
//             std::vector<float> temp_vec = {start_x, start_y, start_z};
//             view_coords.push_back(temp_vec);
//             std::vector<vector<float>> sorted_graph = sortGraph(view_coords);
//             int start_node = view_coords.size() - 1;
//             std::vector<int> min_path = travellingSalesmanProblem(sorted_graph, start_node);
//             std::cout << "MIN PATH SIZE:" << min_path.size() << std::endl;
//             box_count = 0;
//         }
        
//         // float z = boxes.coords[box_count][2] - M_PI; // -2.3
//         // float x = boxes.coords[box_count][0] + 0.5*std::cos(boxes.coords[box_count][2]); //2.42;
//         // float y = boxes.coords[box_count][1] + 0.5*std::sin(boxes.coords[box_count][2]); //-0.894
        
//         // use predetermined viewing location coordinates as destination coordinates
//         float x = view_coords[min_path[box_count]][0];
//         float y = view_coords[min_path[box_count]][1];
//         float z = view_coords[min_path[box_count]][2];

//         ROS_INFO("At: %f,%f,%f", robotPose.x, robotPose.y, robotPose.phi);
//         ROS_INFO("GOING to: %f,%f,%f", x, y, z);

//         bool success = Navigation::moveToGoal(x, y, z);
//         if (success) box_count++;
//         if (box_count == 5) 
//         {   
//             Navigation::moveToGoal(start_x, start_y, start_z);
//             secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();

//             ROS_INFO("Finsihed in: %f", secondsElapsed);
//             break;}
//         // imagePipeline.getTemplateID(boxes);
//         secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
//         std::cout << "Time Elapsed:" << std:: endl;
//         std::cout << secondsElapsed;
//         ros::Duration(0.01).sleep();
        
//     }
//     return 0;
// }


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
    int final_output[5];    // stores template_id of each box in order

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;
    int box_count = 0;
    std::vector<int> min_path;

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
        //break;
        ros::spinOnce();
        
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi

        if (box_count == 0){
            start_x = robotPose.x;
            start_y = robotPose.y;
            start_z = robotPose.phi;
            if (Navigation::moveToGoal(start_x, start_y, start_z+ M_PI))
            {ros::spinOnce();}

            // calculate shortest path
            std::vector<float> temp_vec = {start_x, start_y, start_z};
            std::vector<std::vector<float>> view_coords = newBoxes.coords;
            view_coords.push_back(temp_vec);
            std::vector<vector<float>> sorted_graph = sortGraph(view_coords);
            int start_node = view_coords.size() - 1;
            min_path = travellingSalesmanProblem(sorted_graph, start_node);
            std::cout << "MIN PATH SIZE:" << min_path.size() << std::endl;
            
        }
        
        // z = newBoxes.coords[box_count][2];
        // x = newBoxes.coords[box_count][0];
        // y = newBoxes.coords[box_count][1];

        x = newBoxes.coords[min_path[box_count]][0];
        y = newBoxes.coords[min_path[box_count]][1];
        z = newBoxes.coords[min_path[box_count]][2];
        
        
        ROS_INFO("At: %f,%f,%f", robotPose.x, robotPose.y, robotPose.phi);
        ROS_INFO("GOING to: %f,%f,%f", x, y, z);
        
        // final_output[box_count] = imagePipeline.getTemplateID(boxes); // returns template_id integer (-1 for blank)
        // secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        
        if (Navigation::moveToGoal(x, y, z))
        {
            ros::spinOnce();
            final_output[box_count] = imagePipeline.getTemplateID(boxes); // returns template_id integer (-1 for blank)
            secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
            box_count++;
            
        }

        if (box_count == 5) 
        {   
            Navigation::moveToGoal(start_x, start_y, start_z);
            secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();

            std::cout << "Finished in:" << std:: endl;
            std::cout << secondsElapsed << std::endl;
            break;
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

    return 0;
}



// /* ----Henry's ----

// #include <boxes.h>
// #include <navigation.h>
// #include <robot_pose.h>
// #include <imagePipeline.h>
// #include <chrono>
// #include <cmath>

// bool first=true;
// int main(int argc, char** argv) {
//     // Setup ROS.
//     ros::init(argc, argv, "contest2");
//     ros::NodeHandle n;
//     // Robot pose object + subscriber.
//     RobotPose robotPose(0,0,0);

//     ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
//     // Initialize box coordinates and templates
//     Boxes boxes; 
//     if(!boxes.load_coords() || !boxes.load_templates()) {
//         std::cout << "ERROR: could not load coords or templates" << std::endl;
//         return -1;
//     }
//     for(int i = 0; i < boxes.coords.size(); ++i) {
//         std::cout << "Box coordinates: " << std::endl;
//         std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: " 
//                   << boxes.coords[i][2] << std::endl;
//     }
//     // Initialize image objectand subscriber.
//     ImagePipeline imagePipeline(n);
//     int final_output[5];    // stores template_id of each box in order

//     // contest count down timer
//     std::chrono::time_point<std::chrono::system_clock> start;
//     start = std::chrono::system_clock::now();
//     uint64_t secondsElapsed = 0;
//     int box_count = 0;
//     float start_x;
//     float start_y;
//     float start_z;

//     // Execute strategy.
//     while(ros::ok() && secondsElapsed <= 300) {
//         ros::spinOnce();
//         //**YOUR CODE HERE**
//         // Use: boxes.coords
//         // Use: robotPose.x, robotPose.y, robotPose.phi
//         if (box_count == 0){
//             start_x = robotPose.x;
//             start_y = robotPose.y;
//             start_z = robotPose.phi;
//             if (Navigation::moveToGoal(start_x, start_y, start_z+ M_PI))
//             {ros::spinOnce();}
            
//         }
        
//         float z = boxes.coords[box_count][2] - M_PI; // -2.3
//         float x = boxes.coords[box_count][0] + 0.5*std::cos(boxes.coords[box_count][2]); //2.42;
//         float y = boxes.coords[box_count][1] + 0.5*std::sin(boxes.coords[box_count][2]); //-0.894
        
//         ROS_INFO("At: %f,%f,%f", robotPose.x, robotPose.y, robotPose.phi);
//         ROS_INFO("GOING to: %f,%f,%f", x, y, z);

//         if (Navigation::moveToGoal(x, y, z))
//         {
//             ros::spinOnce();
//             final_output[box_count] = imagePipeline.getTemplateID(boxes); // returns template_id integer (-1 for blank)
//             secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
//             box_count++;
            
//         }

//         if (box_count == 5) 
//         {   
//             Navigation::moveToGoal(start_x, start_y, start_z);
//             uint64_t finalTime = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();

//             // ROS_INFO("Finished in: %f", finalTime);
//             std::cout << "Finished in:" << std:: endl;
//             std::cout << finalTime;
//             break;}
        
//         std::cout << "Time Elapsed:" << std:: endl;
//         std::cout << secondsElapsed;
//         ros::Duration(0.01).sleep();
        
//     }

//     // make text file of final output (coords and template_id
//     std::cout<< "Final output 1: " << final_output[0] << std::endl;
//     std::cout<< "Final output 2: " << final_output[1] << std::endl;
//     std::cout<< "Final output 3: " << final_output[2] << std::endl;
//     std::cout<< "Final output 4: " << final_output[3] << std::endl;
//     std::cout<< "Final output 5: " << final_output[4] << std::endl;
//     return 0;
// }
