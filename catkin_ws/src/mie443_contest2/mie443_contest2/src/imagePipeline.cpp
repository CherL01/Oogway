#include <imagePipeline.h>

// Henry added
#include <iostream>
#include "opencv2/core.hpp"
//#ifdef HAVE_OPENCV_XFEATURES2D
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

using namespace cv;
using namespace cv::xfeatures2d;
using std::cout;
using std::endl;

// Check for cereal box repeats
int rb_count=0;
int ct_count=0;
int rk_count=0;

//input1: box -> template, input2: boxinscene -> img
const char* keys =
        "{ help h |                          | Print help message. }"
        "{ input1 | ../data/box.png          | Path to input image 1. }"
        "{ input2 | ../data/box_in_scene.png | Path to input image 2. }";

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"

ImagePipeline::ImagePipeline(ros::NodeHandle& n) {
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    isValid = false;
}

void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        if(isValid) {
            img.release();
        }
        img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();
        // imshow("test",img);
        isValid = true;
    } catch (cv_bridge::Exception& e) {
        std::cout << "ERROR: Could not convert from " << msg->encoding.c_str()
                  << " to " << IMAGE_TYPE.c_str() << "!" << std::endl;
        isValid = false;
    }    
}

int ImagePipeline::getTemplateID(Boxes& boxes) {
    int template_id = -1;
    if(!isValid) {
        std::cout << "ERROR: INVALID IMAGE!" << std::endl;
    } else if(img.empty() || img.rows <= 0 || img.cols <= 0) {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
    } else {
        /***YOUR CODE HERE***/

        // Initialize variables
        Mat img_object;
        Mat img_scene;
        //std::vector<KeyPoint> keypoints_object, keypoints_scene;
        std::vector<KeyPoint> keypoints_object_chosen, keypoints_scene_chosen;
        Mat descriptors_object, descriptors_scene;
            
        int prev_good_matches=0;

        //std::vector<DMatch> good_matches;
        std::vector<DMatch> chosen_matches;
        int matches_threshold = 25;

        // Iterate 3 times through all the templates, initialize some variables
        for (int counter=0;counter<3;counter++) {
            std::cout << "--template number "<< counter << std::endl;
        
            img_object = boxes.templates[counter];            // need to cycle through all the templates?
            img_scene = img;
            // Show the image it sees on the box
            // imshow("cereal", img);
            cv::waitKey(10);
            if ( img_object.empty() || img_scene.empty() )
            {
                cout << "Could not open or find the image!\n" << endl;
                //parser.printMessage();
                return -1;
            }

            //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
            int minHessian = 600;
            Ptr<SURF> detector = SURF::create( minHessian );
            std::vector<KeyPoint> keypoints_object, keypoints_scene;
            // Mat descriptors_object, descriptors_scene;
            detector->detectAndCompute( img_object, noArray(), keypoints_object, descriptors_object );
            detector->detectAndCompute( img_scene, noArray(), keypoints_scene, descriptors_scene );
            
            //-- Step 2: Matching descriptor vectors with a FLANN based matcher
            // Since SURF is a floating-point descriptor NORM_L2 is used
            Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
            std::vector< std::vector<DMatch> > knn_matches;
            matcher->knnMatch( descriptors_object, descriptors_scene, knn_matches, 5 );
            
            //-- Filter quality of matches using the Lowe's ratio test
            const float ratio_thresh = 0.70f;
            std::vector<DMatch> good_matches;
            for (size_t i = 0; i < knn_matches.size(); i++)
            {
                if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
                {
                    good_matches.push_back(knn_matches[i][0]);
                }
            }
            std::cout << "----matches size: "<< good_matches.size() << std::endl;

            // Template with most matches gets id saved, important variables saved
            if (good_matches.size() >= prev_good_matches) {
                prev_good_matches = good_matches.size();
                template_id = counter;
                chosen_matches = good_matches;
                keypoints_object_chosen=keypoints_object;
                keypoints_scene_chosen=keypoints_scene;
            }
        }

        if (chosen_matches.size() < matches_threshold) {
            std::cout << "----Box is BLANK!!---- " << std::endl;
            return -1;
        }

        std::cout << "----template id chosen: "<< template_id << std::endl;
        img_object = boxes.templates[template_id];
        //-- Draw matches - good keypoint matches visualized between object and scene
        Mat img_matches;
        drawMatches( img_object, keypoints_object_chosen, img_scene, keypoints_scene_chosen, chosen_matches, img_matches, Scalar::all(-1),
                Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        
        //-- Localize the object
        std::vector<Point2f> obj;
        std::vector<Point2f> scene;
        for( size_t i = 0; i < chosen_matches.size(); i++ )
        {
            //-- Get the keypoints from the good matches - separate into obj and scene
            obj.push_back( keypoints_object_chosen[ chosen_matches[i].queryIdx ].pt );
            scene.push_back( keypoints_scene_chosen[ chosen_matches[i].trainIdx ].pt );
        }
        Mat H = findHomography( obj, scene, RANSAC );
        
        //-- Get the corners from the image_1 ( the object to be "detected" )
        std::vector<Point2f> obj_corners(4);
        obj_corners[0] = Point2f(0, 0);
        obj_corners[1] = Point2f( (float)img_object.cols, 0 );
        obj_corners[2] = Point2f( (float)img_object.cols, (float)img_object.rows );
        obj_corners[3] = Point2f( 0, (float)img_object.rows );
        std::vector<Point2f> scene_corners(4);
        perspectiveTransform( obj_corners, scene_corners, H);
        
        //-- Draw lines between the corners (the mapped object in the scene - image_2 )
        line( img_matches, scene_corners[0] + Point2f((float)img_object.cols, 0),
            scene_corners[1] + Point2f((float)img_object.cols, 0), Scalar(0, 255, 0), 4 );
        line( img_matches, scene_corners[1] + Point2f((float)img_object.cols, 0),
            scene_corners[2] + Point2f((float)img_object.cols, 0), Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[2] + Point2f((float)img_object.cols, 0),
            scene_corners[3] + Point2f((float)img_object.cols, 0), Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[3] + Point2f((float)img_object.cols, 0),
            scene_corners[0] + Point2f((float)img_object.cols, 0), Scalar( 0, 255, 0), 4 );
        
        //-- Show detected matches
        // imshow("Good Matches & Object Detection", img_matches );
        // cv::waitKey(1000);
        //return 0;

        if (rb_count>0 && template_id==0) {
            imshow("(REPEAT) RAISIN BRAN - Good Matches & Object Detection", img_matches );
            cv::waitKey(1000);
        
        } else if (ct_count>0 && template_id==1) {
            imshow("(REPEAT) CINNAMON TOAST - Good Matches & Object Detection", img_matches );
            cv::waitKey(1000);
        
        } else if (rk_count>0 && template_id==2) {
            imshow("(REPEAT) RICE KRISPIES - Good Matches & Object Detection", img_matches );
            cv::waitKey(1000);
        
        } else if (template_id==0) {
            imshow("RAISIN BRAN - Good Matches & Object Detection", img_matches );
            //cv::imwrite("/home/tuesday2023/Oogway/catkin_ws/src/mie443_contest2/mie443_contest2/boxes_database/RAISIN_BRAN.jpg",img_matches);
            cv::waitKey(1000);
            rb_count++;
        
        } else if (template_id==1) {
            imshow("CINNAMON TOAST - Good Matches & Object Detection", img_matches );
            //cv::imwrite("/home/tuesday2023/Oogway/catkin_ws/src/mie443_contest2/mie443_contest2/boxes_database/CINNAMON_TOAST.jpg",img_matches);
            cv::waitKey(1000);
            ct_count++;
        
        } else if (template_id==2) {
            imshow("RICE KRISPIES - Good Matches & Object Detection", img_matches );
            //cv::imwrite("/home/tuesday2023/Oogway/catkin_ws/src/mie443_contest2/mie443_contest2/boxes_database/RICE_KRISPIES.jpg",img_matches);
            cv::waitKey(1000);
            rk_count++;

        }
        
        // Use: boxes.templates
        //cv::imshow("view", img);
        cv::waitKey(10);

    }  
    return template_id;
}


/*
1. Get image input
2. perform step 1: keypoint detection with SURF
3. perform step 2: match descriptor vectors using feature matching
4. use lowe's ratio to keep good matches
5. good matches are visualized with drawMatches
6. iterate good matches -> keypoints saved for obj and scene vectors respectively
7. obj/scene transform using findHomography
8. draw lines around it

launch: gazebo, acml, contest2, rviz (not needed)

todo:

figure out how to pop out the image we want, how to "select" template

mie443 laptop amcl line
roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/tuesday2023/Oogway/catkin_ws/src/mie443_contest2/mie443_contest2/maps/map_1.yaml
roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/tuesday2023/Oogway/catkin_ws/src/mie443_contest2/mie443_contest2/maps/contest_2.yaml



*/
