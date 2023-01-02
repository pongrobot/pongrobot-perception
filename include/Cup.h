#ifndef CUP_H
#define CUP_H

#include "ros/ros.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <list>

class Cup
{
    public:
                                    Cup(std::string frame_id);
                                    Cup();
        bool                        assignDetection( const geometry_msgs::PoseStamped detection);
        ros::Time                   lastDetectionTime();
        geometry_msgs::PoseStamped  getPose();

    public:
            static const uint8_t MAX_DETECTIONS_PER_CUP = 10;
        
    private:
        ros::Time                               last_detection_;
        ros::Time                               first_detected_;

        std::list<geometry_msgs::PoseStamped>   detections_;
        geometry_msgs::PoseStamped              estimated_pose_;

}; 

#endif