#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PolygonStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/crop_box.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>

// Detector params
double passthrough_max_depth;
double passthrough_min_depth;
double eps_angle; 
double distance_threshold;
double cube_box_min_height;
double cube_box_max_height;
std::string camera_frame_id = "camera_link"; 
std::string robot_frame_id = "brobot"; 

pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

void pointCloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr msg)
{
    cloud = msg;
}

int main(int argc, char **argv)
{
    // ROS setup
    ros::init(argc, argv, "3D_cup_detection_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(0.5);

    // Read in detector parameters
    nh.param<double>("passthrough_max_depth", passthrough_max_depth, 10.f);
    nh.param<double>("passthrough_min_depth", passthrough_min_depth, 0.f);
    nh.param<double>("eps_angle", eps_angle, 0.1);
    nh.param<double>("distance_threshold", distance_threshold, 0.1);
    nh.param<double>("cube_box_min_height", cube_box_min_height, 0.0);
    nh.param<double>("cube_box_max_height", cube_box_max_height, 1.0);

    // Handle publishers and subscibers
    ros::Subscriber cloud_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB>> ("/camera/depth/color/points", 1, pointCloudCallback);
    ros::Publisher filtered_cloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("/detector/filtered", 1); 
    ros::Publisher hull_pub = nh.advertise<sensor_msgs::PointCloud2> ("/detector/box_cloud", 1); 
    ros::Publisher table_pub = nh.advertise<geometry_msgs::PolygonStamped> ("/detector/table", 1); 

    // Hold processed cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    // main loop
    while (ros::ok())
    {
        // Process cloud
        if ( !cloud->empty() )
        {
            // Passthrough filter
            pcl::PassThrough<pcl::PointXYZRGB> pass;
            pass.setInputCloud(cloud);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(passthrough_min_depth, passthrough_max_depth);
            pass.filter(*filtered_cloud);

            if ( !filtered_cloud->empty() )
            { 
                // Plane detection
                pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud (new pcl::PointCloud<pcl::PointXYZ>);
                pcl::copyPointCloud(*filtered_cloud, *xyz_cloud);
                pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
                pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

                // Create the segmentation object
                pcl::SACSegmentation<pcl::PointXYZ> seg;
                seg.setOptimizeCoefficients (true);
                seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
                seg.setAxis(Eigen::Vector3f::UnitZ());
                seg.setEpsAngle(eps_angle);
                seg.setMethodType(pcl::SAC_RANSAC);
                seg.setDistanceThreshold(distance_threshold);
                seg.setInputCloud(xyz_cloud);
                seg.segment (*inliers, *coefficients);

                // Make sure a plane was found
                if (inliers->indices.size() == 0)
                {
                    ROS_ERROR ("Could not estimate a planar model for the given dataset.");
                }
                else
                {
                    // Set points on plane to red
                    for (const auto& idx: inliers->indices)
                    {
                        filtered_cloud->points[idx].r = 0xFF;
                        filtered_cloud->points[idx].g = 0x00;
                        filtered_cloud->points[idx].b = 0x00;
                    }

                    // Project the model inliers
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
                    pcl::ProjectInliers<pcl::PointXYZ> proj;
                    proj.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
                    proj.setInputCloud(xyz_cloud);
                    proj.setModelCoefficients(coefficients);
                    proj.filter(*cloud_projected);

                    // Create a Concave Hull representation of the projected inliers
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
                    pcl::ConvexHull<pcl::PointXYZ> chull;
                    chull.setInputCloud(cloud_projected);
                    chull.reconstruct(*cloud_hull);

                    // Convert hull to stamped polygon
                    geometry_msgs::PolygonStamped table_poly;
                    table_poly.header.stamp = ros::Time::now();
                    table_poly.header.frame_id = cloud->header.frame_id;

                    // Find extreme values
                    pcl::PointXYZ minPt, maxPt;
                    pcl::getMinMax3D (*cloud_hull, minPt, maxPt);

                    // build rectangle from extreme points 
                    geometry_msgs::Point32 vertex;
                    vertex.x = minPt.x;
                    vertex.y = maxPt.y;
                    vertex.z = minPt.z;
                    table_poly.polygon.points.push_back(vertex);

                    vertex.x = maxPt.x;
                    vertex.y = maxPt.y;
                    vertex.z = minPt.z;
                    table_poly.polygon.points.push_back(vertex);

                    vertex.x = maxPt.x;
                    vertex.y = maxPt.y;
                    vertex.z = maxPt.z;
                    table_poly.polygon.points.push_back(vertex);

                    vertex.x = minPt.x;
                    vertex.y = maxPt.y;
                    vertex.z = maxPt.z;
                    table_poly.polygon.points.push_back(vertex);

                    // Build cube filter
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr box_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
                    pcl::CropBox<pcl::PointXYZRGB> box_filter;
                    box_filter.setMin(Eigen::Vector4f(minPt.x, minPt.y - cube_box_max_height, minPt.z, 1.0));
                    box_filter.setMax(Eigen::Vector4f(maxPt.x, maxPt.y - cube_box_min_height, maxPt.z, 1.0));
                    box_filter.setInputCloud(cloud);
                    box_filter.filter(*box_cloud);

                    // Publish all outputs
                    filtered_cloud_pub.publish(*filtered_cloud);
                    hull_pub.publish(*box_cloud);
                    table_pub.publish(table_poly);

                }
            }
            else
            {
                ROS_WARN("Empty cloud after Passthough");
            }
        }

        //loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
