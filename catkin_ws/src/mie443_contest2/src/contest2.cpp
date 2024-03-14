#include "path_planning.cpp"

#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <chrono>
#include <cmath>
// #include <path_planning.h>


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
    int box_count = -1;
    float start_x;
    float start_y;
    float start_z;
    std::vector<int> min_path;

    // calculate viewing location coordinates
    std::vector<std::vector<float>> view_coords;
    for (int box = 0; box < boxes.coords.size(); box++) {
        float box_z = boxes.coords[box][2] - M_PI; // -2.3
        float box_x = boxes.coords[box][0] + 0.5*std::cos(boxes.coords[box][2]); //2.42;
        float box_y = boxes.coords[box][1] + 0.5*std::sin(boxes.coords[box][2]); //-0.894
        std::vector<float> temp_vec = {box_x, box_y, box_z};

        view_coords.push_back(temp_vec);
    }
    std::cout << "VIEW COORDS SIZE:" << view_coords.size() << std::endl;

    // Execute strategy.
    while(ros::ok() && secondsElapsed <= 300) {
        ros::spinOnce();
        /***YOUR CODE HERE***/
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi

        // startup
        if (box_count == -1){
            ROS_INFO("STARTUP");
            start_x = robotPose.x;
            start_y = robotPose.y;
            start_z = robotPose.phi;
            Navigation::moveToGoal(start_x, start_y, start_z+ M_PI);
            
            // calculate shortest path
            std::vector<float> temp_vec = {start_x, start_y, start_z};
            view_coords.push_back(temp_vec);
            std::vector<vector<float>> sorted_graph = sortGraph(view_coords);
            int start_node = view_coords.size() - 1;
            std::vector<int> min_path = travellingSalesmanProblem(sorted_graph, start_node);
            std::cout << "MIN PATH SIZE:" << min_path.size() << std::endl;
            box_count = 0;
        }
        
        // float z = boxes.coords[box_count][2] - M_PI; // -2.3
        // float x = boxes.coords[box_count][0] + 0.5*std::cos(boxes.coords[box_count][2]); //2.42;
        // float y = boxes.coords[box_count][1] + 0.5*std::sin(boxes.coords[box_count][2]); //-0.894
        
        // use predetermined viewing location coordinates as destination coordinates
        float x = view_coords[min_path[box_count]][0];
        float y = view_coords[min_path[box_count]][1];
        float z = view_coords[min_path[box_count]][2];

        ROS_INFO("At: %f,%f,%f", robotPose.x, robotPose.y, robotPose.phi);
        ROS_INFO("GOING to: %f,%f,%f", x, y, z);

        bool success = Navigation::moveToGoal(x, y, z);
        if (success) box_count++;
        if (box_count == 5) 
        {   
            Navigation::moveToGoal(start_x, start_y, start_z);
            secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();

            ROS_INFO("Finsihed in: %f", secondsElapsed);
            break;}
        // imagePipeline.getTemplateID(boxes);
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        std::cout << "Time Elapsed:" << std:: endl;
        std::cout << secondsElapsed;
        ros::Duration(0.01).sleep();
        
    }
    return 0;
}