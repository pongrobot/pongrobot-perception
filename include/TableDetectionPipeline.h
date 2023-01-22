#ifndef TABLE_DETECTION_PIPELINE_H
#define TABLE_DETECTION_PIPELINE_H

// ROS headers
#include "ros/ros.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Int8.h>
#include <visualization_msgs/MarkerArray.h>

// PCL headers
#include <pcl_ros/point_cloud.h>
#include "pcl_ros/transforms.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>    
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>

class TableDetectionPipeline
{
    public:
        struct PipelineParams
        {
            double frames_to_keep;
        };

        struct CropBoxParams
        {
            std::vector<float> min;
            std::vector<float> max;
        };

        struct SORParams
        {
            int mean_k;
            double std_dev_mul_thresh;
        };

        struct VGFParams
        {
            float leaf_size_x;
            float leaf_size_y;
            float leaf_size_z;
        };

        struct SACParams
        {
            double eps_angle;
            double distance_thresh;
        };

        struct DebugFlags
        {
            bool publish_table_cloud;
            bool publish_table_poly;
        };

    public:
        TableDetectionPipeline( ros::NodeHandle nh, std::string frame_id );
        void run();
        void loadParams();
        std::vector<pcl::PointXYZRGB> process( pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud );

    private:
        // ROS Data
        ros::NodeHandle nh_;
        ros::Subscriber camera_cloud_sub_;
        ros::Subscriber run_pipeline_sub_;
        ros::Publisher table_cloud_pub_;
        ros::Publisher table_poly_pub_;

        // Frame data
        std::string frame_id_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        // Pipeline Data
        std::list<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_frames_;

        // Params
        PipelineParams  pipeline_params_;
        CropBoxParams   crop_box_params_;
        SORParams       sor_params_;
        VGFParams       vgf_params_;
        SACParams       sac_params_;
        DebugFlags      debug_flags_;

        // Callback
        void pointCloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg);
        void runPipelineCallback(const std_msgs::Empty::ConstPtr& msg);

        // Utilities 
        geometry_msgs::PolygonStamped buildTablePoly( pcl::PointXYZRGB minPt, pcl::PointXYZRGB maxPt);

};

#endif