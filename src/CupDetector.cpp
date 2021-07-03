#include "CupDetector.h"

CupDetector::
CupDetector( ros::NodeHandle nh ):
    tf_listener_(tf_buffer_)
{
    nh_ = nh;

    // Read in detector parameters
    load_params();        

    // Init Publishers and Subscribers
    cloud_sub_ = nh_.subscribe<pcl::PointCloud<pcl::PointXYZRGB>> ("/camera/depth/color/points", 1, &CupDetector::pointCloudCallback, this);
    if ( publish_table_cloud_ ) { surface_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2> ("/detector/surface", 1); }
    if ( publish_obj_cloud_ ) { obj_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2> ("/detector/obj", 1); }
    if ( publish_table_poly_ ) { table_poly_pub_ = nh.advertise<geometry_msgs::PolygonStamped> ("/detector/table", 1); }
    if ( publish_cluster_cloud_ ) { cluster_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2> ("/detector/cluster", 1); }
    if ( publish_cup_markers_ ) { marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>( "/detector/cup_marker", 1 ); };
    cup_pose_pub_ = nh.advertise<geometry_msgs::PoseArray>( "/detector/cup_array", 1 );
    
    // Init clouds
    cloud_.reset( new pcl::PointCloud<pcl::PointXYZRGB>() ); 
    passthrough_cloud_.reset( new pcl::PointCloud<pcl::PointXYZRGB>() ); 
    marked_cloud_.reset( new pcl::PointCloud<pcl::PointXYZRGB>() ); 
    obj_cloud_.reset( new pcl::PointCloud<pcl::PointXYZRGB>() ); 
    cluster_cloud_.reset( new pcl::PointCloud<pcl::PointXYZRGB>() ); 

    // Init colorwheel
    colorwheel_.push_back( Eigen::Vector3i(255, 0,     0) );
    colorwheel_.push_back( Eigen::Vector3i(255, 125,   0) );
    colorwheel_.push_back( Eigen::Vector3i(255, 255,   0) );
    colorwheel_.push_back( Eigen::Vector3i(125, 255,   0) );
    colorwheel_.push_back( Eigen::Vector3i(  0, 255,   0) );
    colorwheel_.push_back( Eigen::Vector3i(  0, 255, 125) );
    colorwheel_.push_back( Eigen::Vector3i(  0, 255, 255) );
    colorwheel_.push_back( Eigen::Vector3i(  0, 125, 255) );
    colorwheel_.push_back( Eigen::Vector3i(  0,   0, 255) );
    colorwheel_.push_back( Eigen::Vector3i(125,   0, 255) );
    colorwheel_.push_back( Eigen::Vector3i(255,   0, 255) );
    colorwheel_.push_back( Eigen::Vector3i(255,   0, 125) );
}

void
CupDetector::
pointCloudCallback( const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg )
{
    try
    {
        // wait for the correct transform for up to 5 sec (supress startup errors)
        tf_buffer_.lookupTransform(target_frame_id_, msg->header.frame_id, ros::Time::now(), ros::Duration(5.0));
        pcl_ros::transformPointCloud(target_frame_id_, *msg, *cloud_, tf_buffer_);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
    } 
}

void
CupDetector::
load_params()
{
    nh_.param<std::string>("target_frame_id", target_frame_id_, "world");
    nh_.param<double>("filter/passthrough_max_depth", passthrough_max_depth_, 10.f);
    nh_.param<double>("filer/passthrough_min_depth", passthrough_min_depth_, 0.f);
    nh_.param<double>("filter/object_max_height", obj_max_height_, 1.0);
    nh_.param<double>("segment/eps_angle", eps_angle_, 0.1);
    nh_.param<double>("segment/distance_threshold", distance_threshold_, 0.1);
    nh_.param<double>("cluster/tolerance", cluster_tolerance_, 0.02);
    nh_.param<double>("cluster/min_cluster_size", min_cluster_size_, 100);
    nh_.param<double>("cluster/max_cluster_size", max_cluster_size_, 25000);
    nh_.param<bool>("debug/publish_table_cloud", publish_table_cloud_, true);
    nh_.param<bool>("debug/publish_table_poly", publish_table_poly_, true);
    nh_.param<bool>("debug/publish_obj_cloud", publish_obj_cloud_, true);
    nh_.param<bool>("debug/publish_cluster_cloud", publish_cluster_cloud_, true);
    nh_.param<bool>("debug/publish_cup_markers", publish_cup_markers_, true);
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
                if ( publish_table_cloud_ )
                {
                    // Set points on plane to red for debugging
                    marked_cloud_ = passthrough_cloud_->makeShared();                

                    for (const auto& idx: inliers->indices)
                    {
                        marked_cloud_->points[idx].r = 0xFF;
                        marked_cloud_->points[idx].g = 0x00;
                        marked_cloud_->points[idx].b = 0x00;
                    }

                    surface_cloud_pub_.publish(marked_cloud_);
                }

                // Extract inliers from cloud
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr surface_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>());
                pcl::ExtractIndices<pcl::PointXYZRGB> extract;
                extract.setInputCloud(passthrough_cloud_);
                extract.setIndices(inliers);
                extract.setNegative(false);
                extract.filter (*surface_cloud_);
                pcl::PointXYZRGB minPt, maxPt;
                pcl::getMinMax3D (*surface_cloud_, minPt, maxPt);

                // Build table polygon
                if ( publish_table_poly_ )
                {
                    geometry_msgs::PolygonStamped table_poly = buildTablePoly( minPt, maxPt );
                    table_poly_pub_.publish(table_poly);
                }

                // Pull out segmented objects on table
                pcl::CropBox<pcl::PointXYZRGB> box_filter;
                box_filter.setMin(Eigen::Vector4f(minPt.x, minPt.y, maxPt.z, 1.0));
                box_filter.setMax(Eigen::Vector4f(maxPt.x, maxPt.y, maxPt.z + obj_max_height_, 1.0));
                box_filter.setInputCloud(cloud_);
                box_filter.filter(*obj_cloud_);
               
                if ( publish_obj_cloud_ )
                { 
                    obj_cloud_pub_.publish(obj_cloud_);
                }

                // Cluster the objects to find targets
                pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
                tree->setInputCloud(obj_cloud_);

                std::vector<pcl::PointIndices> cluster_indices;
                pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
                ec.setClusterTolerance(cluster_tolerance_);
                ec.setMinClusterSize(min_cluster_size_);
                ec.setMaxClusterSize(max_cluster_size_);
                ec.setSearchMethod(tree);
                ec.setInputCloud(obj_cloud_);
                ec.extract(cluster_indices);

                // Setup visualization markers
                visualization_msgs::MarkerArray cup_markers;
                visualization_msgs::Marker clear_markers;
                clear_markers.header.frame_id = target_frame_id_;    
                clear_markers.action = visualization_msgs::Marker::DELETEALL;
                cup_markers.markers.push_back(clear_markers);
    
                // Cup pose output
                geometry_msgs::PoseArray cup_pose_array;
                cup_pose_array.header.frame_id = target_frame_id_;
                cup_pose_array.header.stamp = ros::Time();

                // Iterate through clusters
                int cup_index =0;
                cluster_cloud_ = obj_cloud_->makeShared();
                cluster_cloud_->clear();

                for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
                {
                    if ( publish_cluster_cloud_ )
                    {
                        // Build cluster visualization
                        for (const auto& idx : it->indices)
                        {
                            pcl::PointXYZRGB point = (*obj_cloud_)[idx];
                            Eigen::Vector3i color = getColor( cup_index );
                            point.r = color[0];
                            point.g = color[1];
                            point.b = color[2];
                            
                            cluster_cloud_->push_back( point );
                        }
                    }
                    // Get centroid of cup
                    Eigen::Vector4f centroid;
                    if ( pcl::compute3DCentroid(*obj_cloud_, it->indices, centroid) )
                    {
                        if ( publish_cup_markers_ )
                        {
                            cup_markers.markers.push_back( buildCupMarker(cup_index, centroid) );
                        }

                        // create Pose
                        geometry_msgs::Pose cup_pose;
                        cup_pose.position.x = centroid[0];
                        cup_pose.position.y = centroid[1];
                        cup_pose.position.z = centroid[2];
                        cup_pose_array.poses.push_back(cup_pose);

                        cup_index ++;
                    }
                    
                }
                
                // Publish Cup data
                if ( cup_markers.markers.size() > 0 ) 
                {
                    if ( publish_cup_markers_ )
                    {
                        marker_pub_.publish(cup_markers); 
                    }
                    if ( publish_cluster_cloud_ )
                    {
                        cluster_cloud_pub_.publish(cluster_cloud_);
                    }

                    // Output cup positions
                    cup_pose_pub_.publish(cup_pose_array);
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
    table_poly.header.frame_id = target_frame_id_;
    
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

visualization_msgs::Marker
CupDetector::
buildCupMarker( int cup_index, Eigen::Vector4f centroid )
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = target_frame_id_;
    marker.header.stamp = ros::Time();
    marker.ns = "cups";
    marker.id = cup_index;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = centroid[0];
    marker.pose.position.y = centroid[1];
    marker.pose.position.z = centroid[2];
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 0.8;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    return marker;
}

Eigen::Vector3i
CupDetector::
getColor( int seed )
{
    // generate color from the colorwheel
    if (colorwheel_.size() > 0)
    {
        return colorwheel_[ seed % (int) colorwheel_.size() ]; 
    }
    else
    {
        ROS_WARN("Colorwheel uninitialized, returning RED");
        return Eigen::Vector3i( 255, 0, 0);
    }
}
