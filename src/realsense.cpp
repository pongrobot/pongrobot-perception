#include "ros/ros.h"
#include <librealsense2/rs.hpp> 
#include <opencv2/opencv.hpp> 
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <tf2_ros/static_transform_broadcaster.h>


int main(int argc, char **argv)
{
    // ROS setup
    ros::init(argc, argv, "realsense");
    ros::NodeHandle n;
   
    // Read params 
    std::string camera_frame_id = "camera_link"; n.getParam("camera_frame_id", camera_frame_id);
    std::string image_frame_id = "image_frame"; n.getParam("image_frame_id", image_frame_id);
    bool enable_depth_map = true; n.getParam("enable_depth_map", enable_depth_map);

    // Publishers
    image_transport::ImageTransport it(n);
    image_transport::Publisher color_pub = it.advertise("image_color", 1);
    image_transport::Publisher depth_pub = it.advertise("depth", 1);
    image_transport::Publisher depth_u_pub = it.advertise("depth_raw", 1);
    ros::Publisher cam_info_pub = n.advertise<sensor_msgs::CameraInfo>("camera_info", 1000);
    image_transport::Publisher depth_map_pub;
    
    if( enable_depth_map )
    {
        depth_map_pub = it.advertise("depth_map", 1);
    }

    // Setup image frame -> camera frame transform broadcaster
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = camera_frame_id;
    static_transformStamped.child_frame_id = image_frame_id;
    static_transformStamped.transform.rotation.x = -0.5;
    static_transformStamped.transform.rotation.y = 0.5;
    static_transformStamped.transform.rotation.z = -0.5;
    static_transformStamped.transform.rotation.w = 0.5;
    static_broadcaster.sendTransform(static_transformStamped);

    // Create rs2 pipeline
    rs2::pipeline p;
    rs2::colorizer color_map;

    // Create rs2 configuration
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 0, 0, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 0, 0, RS2_FORMAT_Z16, 30);

    // Start the pipeline and align
    rs2::pipeline_profile profile = p.start(cfg);
    rs2::align align_to_depth(RS2_STREAM_DEPTH);
    rs2::align align_to_color(RS2_STREAM_COLOR);

    // Get depth scaling from device
    float depth_scale = 1.f;
    for (rs2::sensor& sensor : profile.get_device().query_sensors())
    {
        if ( rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>() )
        {
            depth_scale = dpt.get_depth_scale();
        }
    }
    
    // We only need the color frame info since depth is aligned to this
    sensor_msgs::CameraInfo color_cam_info; 

    // Let camera autoexposure calibrate for 30 frames
    rs2::frameset frames;
    for(int i=0; i<30; i++)
    {
        frames = p.wait_for_frames();
    }

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    rs2::points points;

    // main loop
    while (ros::ok())
    {
        // Get frames
        frames = p.wait_for_frames();
        frames = align_to_color.process(frames);
        rs2::frame color_frame = frames.get_color_frame();
        rs2::frame depth_frame = frames.get_depth_frame();

        // Convert frame data to openCV
        int w = color_frame.as<rs2::video_frame>().get_width();
        int h = color_frame.as<rs2::video_frame>().get_height();
        cv::Mat color_mat(cv::Size(w, h), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat depth_raw_mat(cv::Size(w,h), CV_16U, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat depth_mat;
        depth_raw_mat.convertTo(depth_mat, CV_32F); // Convert unsigned 16-bit depth to float32 in meters
        depth_mat = depth_mat * depth_scale;
        
        // Populate common header 
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = image_frame_id;

        // Populate camera_info
        rs2::video_stream_profile stream_profile = color_frame.get_profile().as<rs2::video_stream_profile>(); 
        rs2_intrinsics intrinsics= stream_profile.get_intrinsics(); 
        color_cam_info.header = header;
        color_cam_info.height = intrinsics.height;
        color_cam_info.width = intrinsics.width;
        
        // distortion model data
        color_cam_info.distortion_model = "plumb_bob";
        color_cam_info.D.resize(5);
        for (int i = 0; i < 5; i++)
            color_cam_info.D.at(i) = intrinsics.coeffs[i];
        
        // intrensic distorted projection matrix
        color_cam_info.K.at(0) = intrinsics.fx;
        color_cam_info.K.at(2) = intrinsics.ppx;   
        color_cam_info.K.at(4) = intrinsics.fy;   
        color_cam_info.K.at(5) = intrinsics.ppy;   
        color_cam_info.K.at(1) = 1;

        // rotation rectification matrix (identity)
        color_cam_info.R.at(0) = 1.0;
        color_cam_info.R.at(1) = 0.0;
        color_cam_info.R.at(2) = 0.0;
        color_cam_info.R.at(3) = 0.0;
        color_cam_info.R.at(4) = 1.0;
        color_cam_info.R.at(5) = 0.0;
        color_cam_info.R.at(6) = 0.0;
        color_cam_info.R.at(7) = 0.0;
        color_cam_info.R.at(8) = 1.0;

        // intrinsic projection matrix
        color_cam_info.P.at(0) = color_cam_info.K.at(0);
        color_cam_info.P.at(1) = 0;
        color_cam_info.P.at(2) = color_cam_info.K.at(2);
        color_cam_info.P.at(3) = 0;
        color_cam_info.P.at(4) = 0;
        color_cam_info.P.at(5) = color_cam_info.K.at(4);
        color_cam_info.P.at(6) = color_cam_info.K.at(5);
        color_cam_info.P.at(7) = 0;
        color_cam_info.P.at(8) = 0;
        color_cam_info.P.at(9) = 0;
        color_cam_info.P.at(10) = 1;
        color_cam_info.P.at(11) = 0;

        cam_info_pub.publish(color_cam_info);

        // Publish standard color img
        sensor_msgs::ImagePtr color_img = cv_bridge::CvImage(header, "bgr8", color_mat).toImageMsg();
        color_pub.publish(color_img);        


        // Publish floating point depth depth in meters
        sensor_msgs::ImagePtr depth_img = cv_bridge::CvImage(header, "32FC1", depth_mat).toImageMsg();
        depth_pub.publish(depth_img); 

        // Publish 16-bit depth in mm
        sensor_msgs::ImagePtr depth_raw_img = cv_bridge::CvImage(header, "16UC1", depth_raw_mat).toImageMsg();
        depth_u_pub.publish(depth_raw_img); 
        
        if ( enable_depth_map )
        {
            rs2::frame depth_map = frames.get_depth_frame().apply_filter(color_map);
            cv::Mat depth_map_mat(cv::Size(w, h), CV_8UC3, (void*)depth_map.get_data(), cv::Mat::AUTO_STEP);
            sensor_msgs::ImagePtr depth_map_img = cv_bridge::CvImage(header, "bgr8", depth_map_mat).toImageMsg();
            depth_map_pub.publish(depth_map_img);        
        }

        // Create pointcloud
        pc.map_to(color_frame);
        points = pc.calculate(depth_frame);
        // TODO: future expansion - rs2::pointcloud -> sensor_msgs::PointCloud2

        ros::spinOnce();
    }

    return 0;
}
