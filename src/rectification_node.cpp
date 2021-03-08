/**
 * \author Manjunath, Shreyas
 * \date 05.05.2021
 * \brief Implementation file for rectification node
 * */

// header files
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <string>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

/**
 * \brief Method for visualizing the stereo rectified images with epipolar lines. Suitable for debugging.
 * \param left opencv rectified image  from left camera
 * \param right opencv rectified image from right camera
 * \return void
 * */

void visualizeRectImgHelper(cv::Mat& left, cv::Mat& right);

// Rectified image ROS publishers 
ros::Publisher rect_img_left_publisher;
ros::Publisher rect_img_right_publisher;

/**
 * 
 *  2x2 matrix to store rectified mapping values
 * */ 
cv::Mat rectified_mapping[2][2];

/**
 * Structure to store stereo camera parameters 
 * */

struct StereoCameraParams{
    /*@{*/
    cv::Mat camMtx;/**< Camera Matrix */
    cv::Mat distCoeff;/**< Distorsion co efficients */
    cv::Mat rectMtx;/**< Rectification Matrix */
    cv::Mat projectionMtx;/**< Projection Matrix */
    /*@{*/
};

/**
 * Data structure to store left_camera_params
 * */
StereoCameraParams left_camera_params;
/**
 * Data structure to store right_camera_params
 * */
StereoCameraParams right_camera_params;

/**
 * Display flag to opt in or opt out of Open Window to be displayed.
 * Can be set to "true" using args to launch file. Default = false. 
 * */
bool DISPLAY = false;

/**
 * \brief Callback method for subscribing un-rectified stereo image topics, rectifying them and re-publish rectified stereo image topics.
 * \param img_left left camera image pointer, these are ros messages.
 * \param img_right  right camera image pointer, these are ros messages.
 * */
void stereoImageCallback(const sensor_msgs::ImageConstPtr &img_left, const sensor_msgs::ImageConstPtr &img_right){
    
    // opencv image pointers used for ros_message to opencv image coversion
    cv_bridge::CvImageConstPtr left_image_ptr;
    cv_bridge::CvImageConstPtr right_image_ptr;

    // OpenCV matrices to store rectified images
    cv::Mat cv_rectified_left_img;
    cv::Mat cv_rectified_right_img;


    try{
        // Convertion of image_ros_message to opencv image using cv_bridge::toCvShare
        left_image_ptr = cv_bridge::toCvShare(img_left);
        right_image_ptr = cv_bridge::toCvShare(img_right);
    }
    catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    // Rectifying the images using remap method
    cv::remap(left_image_ptr->image, cv_rectified_left_img, rectified_mapping[0][0], rectified_mapping[0][1], cv::INTER_LINEAR);
    cv::remap(right_image_ptr->image, cv_rectified_right_img, rectified_mapping[1][0], rectified_mapping[1][1], cv::INTER_LINEAR);

    // CvImage canvases for re-convertion from opencv image to ros message 
    cv_bridge::CvImage rect_img_left;
    cv_bridge::CvImage rect_img_right;

    // Left camera rectified image data structure which contains header, type of encoding and Image data.
    rect_img_left.header = img_left->header;
    rect_img_left.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    rect_img_left.image = cv_rectified_left_img;

    // Right camera rectified image data structure which contains header, type of encoding and Image data.
    rect_img_right.header = img_right->header;
    rect_img_right.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    rect_img_right.image = cv_rectified_right_img;

    // Publishing the rectified images as ros_messages
    rect_img_left_publisher.publish(rect_img_left.toImageMsg());
    rect_img_right_publisher.publish(rect_img_right.toImageMsg());

    if(DISPLAY){
        // If DISPLAY flag is "true", visualizeRectImgHelper method is called to display the opencv window
        visualizeRectImgHelper(cv_rectified_left_img, cv_rectified_right_img);
    }
    

}

/**
 * \brief This method reads the parameter_file or Intrinsic file provided by the user and stores the corresponding parameters like
 * CameraIntrinsicMatrix, Distorsion coeffs, Rectification Matrix and Projection Matrix into a data structure meant for left and right camera.
 * \param file_name absolute path to the parameter file (parameter file should be in opencv format and .yaml) 
 * */
void readIntrinsicFile(std::string& file_name){
    cv::FileStorage file(file_name.c_str(), cv::FileStorage::READ);

    file["leftCameraIntrinsicMatrix"] >> left_camera_params.camMtx;
    file["leftDistCoeffs"] >> left_camera_params.distCoeff;
    file["leftRectificationMatrix"] >> left_camera_params.rectMtx;
    file["leftProjectionMatrix"] >> left_camera_params.projectionMtx;

    file["rightCameraIntrinsicMatrix"] >> right_camera_params.camMtx;
    file["rightDistCoeffs"] >> right_camera_params.distCoeff;
    file["rightRectificationMatrix"] >> right_camera_params.rectMtx;
    file["rightProjectionMatrix"] >> right_camera_params.projectionMtx;

    ROS_INFO("camera params initialization completed with File: %s", file_name); 

    
}

/**
 * \brief initializes the stereo rectification using initUndistortRectifyMap method and generates rectified mapping values for rectification.
 * */

void initRectifyMaps(){
    cv::initUndistortRectifyMap(left_camera_params.camMtx, left_camera_params.distCoeff, 
                                left_camera_params.rectMtx, left_camera_params.projectionMtx, cv::Size(640, 480),
                                CV_32F, rectified_mapping[0][0], rectified_mapping[0][1]);
    cv::initUndistortRectifyMap(right_camera_params.camMtx, right_camera_params.distCoeff, 
                                right_camera_params.rectMtx, right_camera_params.projectionMtx, cv::Size(640, 480),
                                CV_32F, rectified_mapping[1][0], rectified_mapping[1][1]);

    ROS_INFO("initUndistortRectifyMap completed ");

}

void visualizeRectImgHelper(cv::Mat& left, cv::Mat& right){
    cv::Mat img_to_show;
    // concatinating left and right rectified stereo images on a single canvas
    cv::hconcat(left, right, img_to_show);
    cv::resize(img_to_show, img_to_show, cv::Size(1280,480), (0, 0), (0, 0), cv::INTER_LINEAR);

    // draw epipolar lines to visualize rectification
  for(int j = 0; j < img_to_show.rows; j += 24 ){
    line(img_to_show, cv::Point(0, j),
         cv::Point(img_to_show.cols, j),
         cv::Scalar(255, 0, 0, 255), 1, 8);
  }
  cv::imshow("Rectified Stereo Imgs with epipolar lines", img_to_show);
  cv::waitKey(1);
}

/**
 * Main Method
 * */


int main(int argc, char** argv){

    ros::init(argc, argv, "stereo_rectification_node");
    ros::NodeHandle nh;

    std::string paramFile, left_stereo_topic, right_stereo_topic; 
    // Retrieve node parameters
    nh.getParam("/stereo_rectification_node/stereo_params", paramFile);
    nh.getParam("/stereo_rectification_node/left_stereo_topic", left_stereo_topic);
    nh.getParam("/stereo_rectification_node/right_stereo_topic", right_stereo_topic);
    nh.getParam("/stereo_rectification_node/display", DISPLAY);
    
    // Reads the camera parameter file
    readIntrinsicFile(paramFile);

    // Initializes stereo rectification
    initRectifyMaps();
    
    rect_img_left_publisher = nh.advertise<sensor_msgs::Image>("/vid_rectify/rectified_vga_front_left_images", 10);
    rect_img_right_publisher = nh.advertise<sensor_msgs::Image>("/vid_rectify/rectified_vga_front_right_images", 10);

    message_filters::Subscriber<sensor_msgs::Image> img_left_sub;
    message_filters::Subscriber<sensor_msgs::Image> img_right_sub;
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>* topic_synchronizer;

    img_left_sub.subscribe(nh, left_stereo_topic, 1);
    img_right_sub.subscribe(nh, right_stereo_topic, 1);
    topic_synchronizer = new message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>(img_left_sub, img_right_sub, 10);

    // Subscribing to both the topics at the same time for synchronization
    topic_synchronizer->registerCallback(boost::bind(&stereoImageCallback, _1, _2));
    ros::spin();
	if(DISPLAY){
		cv::destroyAllWindows();
	}
    

    return 0;

}