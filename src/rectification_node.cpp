/**
 * Implementation file for rectification node
 * */

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <string>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

ros::Publisher rect_img_left_publisher;
ros::Publisher rect_img_right_publisher;

cv::Mat rectified_mapping[2][2];

struct StereoCameraParams{
    cv::Mat camMtx;
    cv::Mat distCoeff;
    cv::Mat rectMtx;
    cv::Mat projectionMtx;
};

StereoCameraParams left_camera_params;
StereoCameraParams right_camera_params;


void stereoImageCallback(const sensor_msgs::ImageConstPtr &img_left, const sensor_msgs::ImageConstPtr &img_right){

    cv_bridge::CvImageConstPtr left_image_ptr;
    cv_bridge::CvImageConstPtr right_image_ptr;

    cv::Mat cv_rectified_left_img;
    cv::Mat cv_rectified_right_img;

    try{
        left_image_ptr = cv_bridge::toCvShare(img_left);
        right_image_ptr = cv_bridge::toCvShare(img_right);
    }
    catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    cv::remap(left_image_ptr->image, cv_rectified_left_img, rectified_mapping[0][0], rectified_mapping[0][1], cv::INTER_LINEAR);
    cv::remap(right_image_ptr->image, cv_rectified_right_img, rectified_mapping[1][0], rectified_mapping[1][1], cv::INTER_LINEAR);

    cv_bridge::CvImage rect_img_left;
    cv_bridge::CvImage rect_img_right;

    rect_img_left.header = img_left->header;
    rect_img_left.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    rect_img_left.image = cv_rectified_left_img;

    rect_img_right.header = img_right->header;
    rect_img_right.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    rect_img_right.image = cv_rectified_right_img;


    rect_img_left_publisher.publish(rect_img_left.toImageMsg());
    rect_img_right_publisher.publish(rect_img_right.toImageMsg());

}

void readIntrinsicFile(std::string file_name){
    cv::FileStorage file(file_name.c_str(), cv::FileStorage::READ);

    file["leftCameraIntrinsicMatrix"] >> left_camera_params.camMtx;
    file["leftDistCoeffs"] >> left_camera_params.distCoeff;
    file["leftRectificationMatrix"] >> left_camera_params.rectMtx;
    file["leftProjectionMatrix"] >> left_camera_params.projectionMtx;

    file["rightCameraIntrinsicMatrix"] >> right_camera_params.camMtx;
    file["rightDistCoeffs"] >> right_camera_params.distCoeff;
    file["rightRectificationMatrix"] >> right_camera_params.rectMtx;
    file["rightProjectionMatrix"] >> right_camera_params.projectionMtx;

    std::cout << "camera params initialization completed with File: " << file_name << std::endl;

    
}

void initStereo(){
    cv::initUndistortRectifyMap(left_camera_params.camMtx, left_camera_params.distCoeff, 
                                left_camera_params.rectMtx, left_camera_params.projectionMtx, cv::Size(640, 480),
                                CV_32F, rectified_mapping[0][0], rectified_mapping[0][1]);
    cv::initUndistortRectifyMap(right_camera_params.camMtx, right_camera_params.distCoeff, 
                                right_camera_params.rectMtx, right_camera_params.projectionMtx, cv::Size(640, 480),
                                CV_32F, rectified_mapping[1][0], rectified_mapping[1][1]);

    std::cout << "initUndistortRectifyMap completed "  << std::endl;

}

/* 
    Main method

*/


int main(int argc, char** argv){

    ros::init(argc, argv, "Stereo_Rectification_Node");
    ros::NodeHandle nh;

    std::string file = "/home/manshr/catkin_ws/src/vid_rectify/yaml/m210_stereo_param.yaml";
    readIntrinsicFile(file);
    initStereo();
    

    rect_img_left_publisher = nh.advertise<sensor_msgs::Image>("/dji_osdk_ros_custom/rectified_vga_front_left_image", 10);
    rect_img_right_publisher = nh.advertise<sensor_msgs::Image>("/dji_osdk_ros_custom/rectified_vga_front_right_image", 10);

    message_filters::Subscriber<sensor_msgs::Image> img_left_sub;
    message_filters::Subscriber<sensor_msgs::Image> img_right_sub;
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>* topic_synchronizer;

    img_left_sub.subscribe(nh, "/dji_osdk_ros/stereo_vga_front_left_images", 1);
    img_right_sub.subscribe(nh, "/dji_osdk_ros/stereo_vga_front_right_images", 1);
    topic_synchronizer = new message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>(img_left_sub, img_right_sub, 10);

    img_left_sub. subscribe(nh, "/dji_osdk_ros/stereo_vga_front_left_images", 1);
    img_right_sub.subscribe(nh, "/dji_osdk_ros/stereo_vga_front_right_images", 1);
  
    topic_synchronizer = new message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>(img_left_sub, img_right_sub, 10);

    topic_synchronizer->registerCallback(boost::bind(&stereoImageCallback, _1, _2));
    ros::spin();

    return 0;

}