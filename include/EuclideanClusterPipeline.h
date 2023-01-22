#ifndef EUCLIDEAN_CLUSTER_PIPELINE_H
#define EUCLIDEAN_CLUSTER_PIPELINE_H

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
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

// Std Libs
#include <algorithm>
#include <list>

class EuclideanClusterPipeline
{
    public:
        struct PipelineParams
        {
            double frames_to_keep;
        };

        struct CropBoxParams
        {
            double max_object_height;
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

        struct ECEParams
        {
            double cluster_tolerance;
            int min_cluster_size;
            int max_cluster_size;
            double max_cluster_width;
        };

        struct DebugFlags
        {
            bool publish_filtered_cloud;
            bool publish_cluster_cloud;
            bool publish_cup_markers;
        };

    public:
        EuclideanClusterPipeline( ros::NodeHandle nh, std::string frame_id );
        void run();
        void loadParams();
        geometry_msgs::PoseArray process( pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud );

    private:
        // Ros Data
        ros::NodeHandle nh_;
        ros::Subscriber camera_cloud_sub_;
        ros::Publisher obj_cloud_pub_;
        ros::Publisher cluster_cloud_pub_;
        ros::Publisher marker_pub_;
        ros::Publisher cup_pose_pub_;

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
        ECEParams       ece_params_;
        DebugFlags      debug_flags_;
        
        // Utility
        std::vector<Eigen::Vector3i> colorwheel_;

        // Callback
        void pointCloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg);

    private:
        visualization_msgs::Marker buildCupMarker( int cup_index, Eigen::Vector4f centroid );
        Eigen::Vector3i getColor( int seed );

};

#endif
