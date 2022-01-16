#ifndef CALIBRATOR_H
#define CALIBRATOR_H

// ROS headers
#include "ros/ros.h"
#include <std_msgs/Empty.h>
#include <geometry_msgs/PolygonStamped.h>

// PCL headers
#include <pcl_ros/point_cloud.h>
#include "pcl_ros/transforms.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>

class Calibrator
{
    public:
        Calibrator( ros::NodeHandle nh );
        void run();
        void loadParams();

    private:
        // Ros data
        ros::NodeHandle nh_;
        ros::Subscriber calibrate_sub_;
        ros::Subscriber cloud_sub_;
        ros::Publisher calibrate_complete_pub_;
        ros::Publisher surface_cloud_pub_;
        ros::Publisher table_poly_pub_;

        //Frame data
        std::string target_frame_id_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        // Filter/Segmentation params
        double passthrough_max_depth_;
        double passthrough_min_depth_;
        double eps_angle_;
        double distance_threshold_;
        double obj_max_height_;

        // Visualization
        bool publish_table_cloud_;
        bool publish_table_poly_;

        // Calibration
        bool run_calibration_;
        int num_iterations_;
        std::vector<pcl::PointXYZRGB> calibration_;

        // Point clouds
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr passthrough_cloud_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr marked_cloud_;

        // Callbacks
        void calibrateCallback(const std_msgs::Empty::ConstPtr& msg);
        void pointCloudCallback( const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg );

        // Helpers
        geometry_msgs::PolygonStamped buildTablePoly( pcl::PointXYZRGB minPt, pcl::PointXYZRGB maxPt);
        std::vector<pcl::PointXYZRGB> calibrate();
        void publishCalibration();
};

#endif
