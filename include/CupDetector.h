#ifndef CUP_DETECTOR_H
#define CUP_DETECTOR_H

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
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/search/kdtree.h>

enum DetectorState
{
    CALIBRATING,
    RESTARTING,
    RUNNING
};

class CupDetector
{
    public:
        CupDetector( ros::NodeHandle nh );
        void run();
        void detect();
        void load_params();

    private:
        // Ros data
        ros::NodeHandle nh_;
        ros::Subscriber cloud_sub_;
        ros::Publisher obj_cloud_pub_;
        ros::Publisher cluster_cloud_pub_;
        ros::Publisher marker_pub_;
        ros::Publisher cup_pose_pub_;
        ros::Publisher state_pub_;
        ros::Subscriber restart_sub_;
        ros::Subscriber calibrate_sub_;

        // Control Data
        DetectorState state_;
        bool restart_;
        bool calibrate_;
        ros::Time calibration_time_;
        ros::Duration calibration_timeout_;

        // Filter/Segmentation params
        double passthrough_max_depth_;
        double passthrough_min_depth_;
        double eps_angle_;
        double distance_threshold_;
        double obj_max_height_;
        double cluster_tolerance_;
        double min_cluster_size_;
        double max_cluster_size_;

        //Frame data
        std::string target_frame_id_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        // Point clouds
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr passthrough_cloud_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr marked_cloud_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr obj_cloud_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud_;

        // Visualization data
        std::vector<Eigen::Vector3i> colorwheel_;
        bool publish_table_cloud_;
        bool publish_table_poly_;
        bool publish_obj_cloud_;
        bool publish_cluster_cloud_;
        bool publish_cup_markers_;

        // Callbacks
        void pointCloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg);
        void restartCallback(const std_msgs::Empty::ConstPtr& msg);
        void calibrateCallback(const std_msgs::Empty::ConstPtr& msg);

        // Helpers
        geometry_msgs::PolygonStamped buildTablePoly( pcl::PointXYZRGB minPt, pcl::PointXYZRGB maxPt );
        visualization_msgs::Marker buildCupMarker( int cup_index, Eigen::Vector4f centroid );
        Eigen::Vector3i getColor( int seed );

};

#endif
