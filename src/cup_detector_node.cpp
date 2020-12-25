#include "ros/ros.h"
#include <opencv2/opencv.hpp> 
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

// store image messages
sensor_msgs::Image color_msg;
sensor_msgs::Image depth_msg;

void colorCallback(const sensor_msgs::ImageConstPtr& msg)
{
    color_msg = *msg;
}

void depthCallback(const sensor_msgs::ImageConstPtr& msg)
{
    depth_msg = *msg;
}

int main(int argc, char **argv)
{
    // ROS setup
    ros::init(argc, argv, "cup_detection_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(0.5);
    
    // Initialize subscribers
    image_transport::ImageTransport it(n);
    image_transport::Subscriber color_sub = it.subscribe("/camera/image_color", 1, colorCallback);
    image_transport::Subscriber depth_sub= it.subscribe("/camera/depth_raw", 1, depthCallback);

    cv::Mat color_frame;
    cv::Mat depth_frame;

    // main loop
    while (ros::ok())
    {
        // Make sure frames are in sync and valid
        if( depth_msg.header.stamp == color_msg.header.stamp && color_msg.header.stamp.toSec() != 0 )
        {
            // get openCV::Mat for each frame
            try
            {
                color_frame = cv_bridge::toCvCopy(color_msg, color_msg.encoding)->image;
                depth_frame = cv_bridge::toCvCopy(depth_msg, depth_msg.encoding)->image;
                ROS_INFO("Frames are valid");
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
            } 

            // run cup detection routine
            /*
            TODO: implement - blueprint from rs2 base version
            // Apply depth thresholding remove_background(color_frame, depth_frame, depth_scale, 3.f, 2.f); 
            cv::Mat blurred, gray, gray_thresh;
            //cv::GaussianBlur(threshold, blurred, cv::Size(3, 3), 0);
            cv::cvtColor( color_frame, gray, cv::COLOR_BGR2GRAY );
            cv::threshold( gray, gray_thresh, 150, 255, cv::THRESH_BINARY);

            // Find contours
            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hierarchy;
            cv::findContours( gray_thresh, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
            
            // Find circular contours
            std::vector<std::vector<cv::Point>> circles;
            */

            // TODO: publish PoseArray of cups
        }

        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
