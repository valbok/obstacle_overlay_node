/**
    @file
    @author  Val Doroshchuk, Willow Garage, Inc.
    @copyright  2021
*/

#pragma once

#include "point_cloud_common.h"

#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <tf2_ros/message_filter.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

namespace rviz
{
    class Renderer;
    class PointCloud2Display
    {
    public:
        void initialize(const std::string &topic, const Renderer &renderer, ros::NodeHandle *nh);
        void reset();
        void update();
        void cb_pcl(const sensor_msgs::PointCloud2ConstPtr& cloud);

        message_filters::Subscriber<sensor_msgs::PointCloud2> sub_;
        std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::PointCloud2>> tf_filter_;

        std::unique_ptr<PointCloudCommon> point_cloud_common_;
    };

} // namespace rviz

