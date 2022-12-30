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
                                    Cup( const uint16_t cup_id, const std::string frame_id);
        uint16_t                    getId();
        bool                        insertDetection( const geometry_msgs::PoseStamped detection);
        ros::Time                   lastDetectionTime();
        geometry_msgs::PoseStamped  getPose();
        
    private:
        uint16_t                                id_;
        ros::Time                               lastDetection_;
        std::list<geometry_msgs::PoseStamped>   detections_;
        geometry_msgs::PoseStamped              estimated_pose_;

    public:
            const uint8_t MAX_DETECTIONS_PER_CUP = 10;

};

#endif