// --- Henry's ---

#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <chrono>
#include <cmath>
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
    int final_output[6];    // stores template_id of each box in order

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;
    int box_count = 0;
    float start_x;
    float start_y;
    float start_z;

    // Execute strategy.
    while(ros::ok() && secondsElapsed <= 300) {
        ros::spinOnce();
        //**YOUR CODE HERE**
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi
        if (box_count == 0){
            start_x = robotPose.x;
            start_y = robotPose.y;
            start_z = robotPose.phi;
            Navigation::moveToGoal(start_x, start_y, start_z+ M_PI);
            
        }
        
        float z = boxes.coords[box_count][2] - M_PI; // -2.3
        float x = boxes.coords[box_count][0] + 0.5*std::cos(boxes.coords[box_count][2]); //2.42;
        float y = boxes.coords[box_count][1] + 0.5*std::sin(boxes.coords[box_count][2]); //-0.894
        
        ROS_INFO("At: %f,%f,%f", robotPose.x, robotPose.y, robotPose.phi);
        ROS_INFO("GOING to: %f,%f,%f", x, y, z);

        bool success = Navigation::moveToGoal(x, y, z);
        
        final_output[box_count] = imagePipeline.getTemplateID(boxes); // returns template_id integer (-1 for blank)
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        if (success) box_count++;

        if (box_count == 5) 
        {   
            Navigation::moveToGoal(start_x, start_y, start_z);
            uint64_t finalTime = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
            final_output[box_count+1] = imagePipeline.getTemplateID(boxes);

            // ROS_INFO("Finished in: %f", finalTime);
            std::cout << "Finished in:" << std:: endl;
            std::cout << finalTime;
            break;}
        
        std::cout << "Time Elapsed:" << std:: endl;
        std::cout << secondsElapsed;
        ros::Duration(0.01).sleep();
        
    }

    // make text file of final output (coords and template_id
    std::cout<< "Final output 1: " << final_output[0] << std::endl;
    std::cout<< "Final output 2: " << final_output[1] << std::endl;
    std::cout<< "Final output 3: " << final_output[2] << std::endl;
    std::cout<< "Final output 4: " << final_output[3] << std::endl;
    std::cout<< "Final output 5: " << final_output[4] << std::endl;
    std::cout<< "Final output 6: " << final_output[5] << std::endl;
    return 0;
}
