#include "CupDetector.h"

CupDetector::
CupDetector( ros::NodeHandle nh ):
    tf_listener_(tf_buffer_)
{
    nh_ = nh;
    cloud_sub_ = nh_.subscribe<pcl::PointCloud<pcl::PointXYZRGB>> ("/camera/depth/color/points", 1, &CupDetector::pointCloudCallback, this);
    surface_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2> ("/detector/surface", 1);
    obj_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2> ("/detector/obj", 1);
    cluster_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2> ("/detector/obj", 1);
    table_poly_pub_ = nh.advertise<geometry_msgs::PolygonStamped> ("/detector/table", 1);
    
    // Init clouds
    cloud_.reset( new pcl::PointCloud<pcl::PointXYZRGB>() ); 
    passthrough_cloud_.reset( new pcl::PointCloud<pcl::PointXYZRGB>() ); 
    marked_cloud_.reset( new pcl::PointCloud<pcl::PointXYZRGB>() ); 
    obj_cloud_.reset( new pcl::PointCloud<pcl::PointXYZRGB>() ); 
    cluster_cloud_.reset( new pcl::PointCloud<pcl::PointXYZRGB>() ); 

    // Read in detector parameters
    nh.param<double>("passthrough_max_depth", passthrough_max_depth_, 10.f);
    nh.param<double>("passthrough_min_depth", passthrough_min_depth_, 0.f);
    nh.param<double>("eps_angle", eps_angle_, 0.1);
    nh.param<double>("distance_threshold", distance_threshold_, 0.1);
    nh.param<double>("obj_max_height", obj_max_height_, 1.0);
    nh.param<std::string>("robot_frame_id", robot_frame_id_, "brobot");

    ROS_INFO("%f", passthrough_max_depth_);
}

void
CupDetector::
pointCloudCallback( const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg )
{
    try
    {
        pcl_ros::transformPointCloud(robot_frame_id_, *msg, *cloud_, tf_buffer_);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
    } 
}

void
CupDetector::
run()
{
    if ( !cloud_->empty() )
    {
        // Passthrough filter
        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud(cloud_);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(passthrough_min_depth_, passthrough_max_depth_);
        pass.filter(*passthrough_cloud_);

        if ( !passthrough_cloud_->empty() )
        {
            // Plane detection
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

            // Create the segmentation object
            pcl::SACSegmentation<pcl::PointXYZRGB> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
            seg.setAxis(Eigen::Vector3f::UnitX());
            seg.setEpsAngle(eps_angle_);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(distance_threshold_);
            seg.setInputCloud(passthrough_cloud_);
            seg.segment (*inliers, *coefficients);             

            // Make sure a plane was found
            if (inliers->indices.size() == 0)
            {
                ROS_ERROR ("Could not estimate a planar model for the given dataset.");
            }
            else
            {
                // Set points on plane to red for debugging
                marked_cloud_ = passthrough_cloud_->makeShared();                

                for (const auto& idx: inliers->indices)
                {
                    marked_cloud_->points[idx].r = 0xFF;
                    marked_cloud_->points[idx].g = 0x00;
                    marked_cloud_->points[idx].b = 0x00;
                }

                // Extract inliers from cloud
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr surface_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>());
                pcl::ExtractIndices<pcl::PointXYZRGB> extract;
                extract.setInputCloud(passthrough_cloud_);
                extract.setIndices(inliers);
                extract.setNegative(false);
                extract.filter (*surface_cloud_);

                // Build table polygon
                pcl::PointXYZRGB minPt, maxPt;
                pcl::getMinMax3D (*surface_cloud_, minPt, maxPt);
                geometry_msgs::PolygonStamped table_poly = buildTablePoly( minPt, maxPt );
                table_poly_pub_.publish(table_poly);

                // Pull out segmented objects on table
                pcl::CropBox<pcl::PointXYZRGB> box_filter;
                box_filter.setMin(Eigen::Vector4f(minPt.x, minPt.y, maxPt.z, 1.0));
                box_filter.setMax(Eigen::Vector4f(maxPt.x, maxPt.y, maxPt.z + obj_max_height_, 1.0));
                box_filter.setInputCloud(cloud_);
                box_filter.filter(*obj_cloud_);
                
                // Publish clouds 
                surface_cloud_pub_.publish(marked_cloud_);
                obj_cloud_pub_.publish(obj_cloud_);


                // Cluster the objects to find targets
                pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
                tree->setInputCloud(obj_cloud_);

                std::vector<pcl::PointIndices> cluster_indices;
                pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
                ec.setClusterTolerance(0.02); // 2cm
                ec.setMinClusterSize(100);
                ec.setMaxClusterSize(25000);
                ec.setSearchMethod(tree);
                ec.setInputCloud(obj_cloud_);
                ec.extract(cluster_indices);

                // clear cloud to store clusters
                cluster_cloud_->clear(); 
                ROS_INFO("Found %d clusters", cluster_indices.size());

                // Iterate through clusters
                for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
                {
                }

            }
        }
    }
}

geometry_msgs::PolygonStamped
CupDetector::
buildTablePoly( pcl::PointXYZRGB minPt, pcl::PointXYZRGB maxPt )
{
    // Initialize message
    geometry_msgs::PolygonStamped table_poly;
    table_poly.header.stamp = ros::Time::now();
    table_poly.header.frame_id = robot_frame_id_;
    
    // build rectangle from extreme points 
    geometry_msgs::Point32 vertex;
    vertex.x = minPt.x;
    vertex.y = minPt.y;
    vertex.z = maxPt.z;
    table_poly.polygon.points.push_back(vertex);

    vertex.x = maxPt.x;
    vertex.y = minPt.y;
    vertex.z = maxPt.z;
    table_poly.polygon.points.push_back(vertex);

    vertex.x = maxPt.x;
    vertex.y = maxPt.y;
    vertex.z = maxPt.z;
    table_poly.polygon.points.push_back(vertex);

    vertex.x = minPt.x;
    vertex.y = maxPt.y;
    vertex.z = maxPt.z;
    table_poly.polygon.points.push_back(vertex);

    return table_poly;
}
