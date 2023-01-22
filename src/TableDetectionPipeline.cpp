#include "TableDetectionPipeline.h"

TableDetectionPipeline::
TableDetectionPipeline
( ros::NodeHandle nh, std::string frame_id ):
    frame_id_(frame_id),
    tf_listener_(tf_buffer_)
{
    nh_ = nh;
    loadParams();
    
    // Init Publishers and Subscribers
    camera_cloud_sub_ = nh_.subscribe<pcl::PointCloud<pcl::PointXYZRGB>> ("/camera/depth/color/points", 1, &TableDetectionPipeline::pointCloudCallback, this);
    run_pipeline_sub_ = nh_.subscribe<std_msgs::Empty> ("run_pipeline", 1, &TableDetectionPipeline::runPipelineCallback, this);
    if ( debug_flags_.publish_table_cloud ) { table_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2> ("table_surface", 1); }
    if ( debug_flags_.publish_table_poly ) { table_poly_pub_ = nh.advertise<geometry_msgs::PolygonStamped> ("table_polygon", 1); }

}

void
TableDetectionPipeline::
run()
{
    loadParams();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pipeline_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto& cloud : cloud_frames_)
    {
        *pipeline_cloud+=*cloud;
    }

    std::vector<pcl::PointXYZRGB> extremes  = process(pipeline_cloud);

    // Publish table detection
    std::vector<float> min;
    min.push_back(extremes[0].x);
    min.push_back(extremes[0].y);
    min.push_back(extremes[0].z);
    nh_.setParam("/table/min", min);

    std::vector<float> max;
    max.push_back(extremes[1].x);
    max.push_back(extremes[1].y);
    max.push_back(extremes[1].z);
    nh_.setParam("/table/max", max);
}

std::vector<pcl::PointXYZRGB>
TableDetectionPipeline::
process( pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud )
{
    pcl::PointXYZRGB min, max;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

    // Run through CropBox
    if ( !raw_cloud->empty() )
    {
        //////////////////////////////////////////////////
        // Run filtering step
        //////////////////////////////////////////////////
        pcl::CropBox<pcl::PointXYZRGB> box_filter;
        box_filter.setMin(Eigen::Vector4f(crop_box_params_.min[0], crop_box_params_.min[1], crop_box_params_.min[2], 1.0));
        box_filter.setMax(Eigen::Vector4f(crop_box_params_.min[0], crop_box_params_.min[1], crop_box_params_.min[2], 1.0));
        box_filter.setInputCloud(raw_cloud);
        box_filter.filter(*cloud);

        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud( cloud );
        sor.setMeanK(sor_params_.mean_k);
        sor.setStddevMulThresh(sor_params_.std_dev_mul_thresh);
        sor.filter(*cloud);

        pcl::VoxelGrid<pcl::PointXYZRGB> vgf;
        vgf.setInputCloud(cloud);
        vgf.setLeafSize(vgf_params_.leaf_size_x,
                        vgf_params_.leaf_size_y,
                        vgf_params_.leaf_size_z);
        vgf.filter(*cloud);
    
        //////////////////////////////////////////////////
        // Fit plane model
        //////////////////////////////////////////////////
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
        seg.setAxis(Eigen::Vector3f::UnitX());
        seg.setEpsAngle(sac_params_.eps_angle);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(sac_params_.distance_thresh);
        seg.setInputCloud(cloud);
        seg.segment (*inliers, *coefficients);

        //////////////////////////////////////////////////
        // Extract Inliers from segmentation
        //////////////////////////////////////////////////
        if (inliers->indices.size() != 0)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr table_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::ExtractIndices<pcl::PointXYZRGB> extract;
            extract.setInputCloud(cloud);
            extract.setIndices(inliers);
            extract.setNegative(false);
            extract.filter (*table_cloud);

            // Extract extreme values
            pcl::getMinMax3D (*table_cloud, min, max);

            // Publish debug cloud
            if ( debug_flags_.publish_table_cloud )
            {       
                for (auto& point : table_cloud->points)
                {
                    point.r = 0xFF;
                    point.g = 0x00;
                    point.b = 0x00;
                }

                table_cloud_pub_.publish(table_cloud);
            }

            // Build table polygon
            if ( debug_flags_.publish_table_poly )
            {
                geometry_msgs::PolygonStamped table_poly = buildTablePoly( min, max );
                table_poly_pub_.publish(table_poly);
            }            
        }
    }

    return std::vector<pcl::PointXYZRGB>{min, max};
}

/////////////////////////////////////////////////////////
// Callbacks
/////////////////////////////////////////////////////////

void
TableDetectionPipeline::
pointCloudCallback( const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg )
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr incoming_cloud;

    try
    {
        // wait for the correct transform for up to 5 sec (surpress startup errors)
        tf_buffer_.lookupTransform(frame_id_, msg->header.frame_id, ros::Time::now(), ros::Duration(5.0));
        pcl_ros::transformPointCloud(frame_id_, *msg, *incoming_cloud, tf_buffer_);
        cloud_frames_.push_front(incoming_cloud);

        // Flush out old frames
        while ( cloud_frames_.size() > pipeline_params_.frames_to_keep )
        {
            cloud_frames_.pop_back();
        }
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s: Could not find transform to \"%s\", rejecting pointcloud.", ros::this_node::getName().c_str(), frame_id_.c_str());
    } 
}

void
TableDetectionPipeline::
runPipelineCallback(const std_msgs::Empty::ConstPtr& msg)
{
    run();
}

/////////////////////////////////////////////////////////
// Utility Functions
/////////////////////////////////////////////////////////
void
TableDetectionPipeline::
loadParams()
{
    // Load Pipeline Params
    if ( !nh_.getParam("pipeline/frames_to_keep", pipeline_params_.frames_to_keep) )
    {
        ROS_ERROR("Table Detection Pipeline cannot load param: %s/pipeline/frames_to_keep", nh_.getNamespace().c_str() );
    }

    // Load CropBox
    if ( !nh_.getParam("crop_box/min", crop_box_params_.min) )
    {
        ROS_ERROR("Table Detection Pipeline cannot load param: %s/crop_box/min", nh_.getNamespace().c_str() );
    }
    if ( !nh_.getParam("crop_box/max", crop_box_params_.max) )
    {
        ROS_ERROR("Table Detection Pipeline cannot load param: %s/crop_box/max", nh_.getNamespace().c_str() );
    }

     // Load SOR params
    if ( !nh_.getParam("sor/mean_k", sor_params_.mean_k) )
    {
        ROS_ERROR("Table Detection Pipeline cannot load param: %s/sor/mean_k", nh_.getNamespace().c_str() );
    }
    if ( !nh_.getParam("sor/std_dev_mul_thresh", sor_params_.std_dev_mul_thresh) )
    {
        ROS_ERROR("Table Detection Pipeline cannot load param: %s/sor/std_dev_mul_thresh", nh_.getNamespace().c_str() );
    }

    // Load VGF params 
    if ( !nh_.getParam("vgf/leaf_size_x", vgf_params_.leaf_size_x) )
    {
        ROS_ERROR("Table Detection Pipeline cannot load param: %s/vgf/leaf_size_x", nh_.getNamespace().c_str() );
    }
    if ( !nh_.getParam("vgf/leaf_size_y", vgf_params_.leaf_size_y) )
    {
        ROS_ERROR("Table Detection Pipeline cannot load param: %s/vgf/leaf_size_y", nh_.getNamespace().c_str() );
    }
    if ( !nh_.getParam("vgf/leaf_size_z", vgf_params_.leaf_size_z) )
    {
        ROS_ERROR("Table Detection Pipeline cannot load param: %s/vgf/leaf_size_z", nh_.getNamespace().c_str() );
    }

    // Load SAC params
    if ( !nh_.getParam("sac/eps_angle", sac_params_.eps_angle) )
    {
        ROS_ERROR("Table Detection Pipeline cannot load param: %s/sac/eps_angle", nh_.getNamespace().c_str() );
    }
    if ( !nh_.getParam("sac/distance_thresh", sac_params_.distance_thresh) )
    {
        ROS_ERROR("Table Detection Pipeline cannot load param: %s/sac/distance_thresh", nh_.getNamespace().c_str() );
    }

    // Load debug params
    if ( !nh_.getParam("debug/publish_table_cloud", debug_flags_.publish_table_cloud) )
    {
        ROS_ERROR("Table Detection Pipeline cannot load param: %s/debug/publish_table_cloud", nh_.getNamespace().c_str() );
    }
    if ( !nh_.getParam("debug/publish_table_poly", debug_flags_.publish_table_poly) )
    {
        ROS_ERROR("Table Detection Pipeline cannot load param: %s/debug/publish_table_poly", nh_.getNamespace().c_str() );
    }
}

geometry_msgs::PolygonStamped
TableDetectionPipeline::
buildTablePoly( pcl::PointXYZRGB minPt, pcl::PointXYZRGB maxPt )
{
    // Initialize message
    geometry_msgs::PolygonStamped table_poly;
    table_poly.header.stamp = ros::Time::now();
    table_poly.header.frame_id = frame_id_;
    
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

/////////////////////////////////////////////////////////
// Main
/////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    // ROS setup
    ros::init(argc, argv, "table_detection_pipeline");
    ros::NodeHandle nh;

     // Load Params
    float loop_frq;
    std::string frame_id;
    if ( !nh.getParam("rate", loop_frq) )
    {
        ROS_ERROR("Table Detection Pipeline cannot load param: %s/rate", nh.getNamespace().c_str());
    }
    if ( !nh.getParam("frame_id", frame_id) )
    {
        ROS_ERROR("Table Detection Pipeline cannot load param: %s/frame_id", nh.getNamespace().c_str());
    }

    ros::Rate loop_rate(loop_frq);
    TableDetectionPipeline table_detection_pipeline(nh, frame_id);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}