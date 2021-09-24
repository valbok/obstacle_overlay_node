/**
    @file
    @author  Val Doroshchuk
    @copyright  2021
*/

#include "renderer.h"
#include "frame_manager.h"
#include "ogre_helpers/point_cloud.h"
#include "validate_floats.h"
#include "point_cloud2_display.h"

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <ros/time.h>

namespace rviz
{
    void PointCloud2Display::initialize(const std::string &topic, const Renderer &renderer, ros::NodeHandle *nh)
    {
        point_cloud_common_.reset(new PointCloudCommon(renderer));
        // PointCloudCommon sets up a callback queue with a thread for each
        // instance.  Use that for processing incoming messages.
        nh->setCallbackQueue(point_cloud_common_->getCallbackQueue());
        auto &frame_manager = point_cloud_common_->renderer_.frame_manager_;
        tf_filter_.reset(new tf2_ros::MessageFilter<sensor_msgs::PointCloud2>(*frame_manager->getTF2BufferPtr(),
                                                                              frame_manager->getFixedFrame(), 10, *nh));
        tf_filter_->connectInput(sub_);
        tf_filter_->registerCallback(boost::bind(&PointCloud2Display::cb_pcl, this, _1));

        sub_.subscribe(*nh, topic, 10);
    }

    inline int32_t findChannelIndex(const sensor_msgs::PointCloud2ConstPtr& cloud, const std::string& channel)
    {
        for (size_t i = 0; i < cloud->fields.size(); ++i)
        {
            if (cloud->fields[i].name == channel)
            {
                return i;
            }
        }

        return -1;
    }

    void PointCloud2Display::cb_pcl(const sensor_msgs::PointCloud2ConstPtr& cloud)
    {
        // Filter any nan values out of the cloud.  Any nan values that make it through to PointCloudBase
        // will get their points put off in lala land, but it means they still do get processed/rendered
        // which can be a big performance hit
        sensor_msgs::PointCloud2Ptr filtered(new sensor_msgs::PointCloud2);
        int32_t xi = findChannelIndex(cloud, "x");
        int32_t yi = findChannelIndex(cloud, "y");
        int32_t zi = findChannelIndex(cloud, "z");

        if (xi == -1 || yi == -1 || zi == -1)
        {
            return;
        }

        const uint32_t xoff = cloud->fields[xi].offset;
        const uint32_t yoff = cloud->fields[yi].offset;
        const uint32_t zoff = cloud->fields[zi].offset;
        const uint32_t point_step = cloud->point_step;
        const size_t point_count = cloud->width * cloud->height;

        if (point_count * point_step != cloud->data.size())
        {
            ROS_ERROR_STREAM("Data size (" << cloud->data.size() << " bytes) does not match width (" << cloud->width
               << ") times height (" << cloud->height << ") times point_step (" << point_step
               << ").  Dropping message.");
            return;
        }

        filtered->data.resize(cloud->data.size());
        uint32_t output_count;
        if (point_count == 0)
        {
            output_count = 0;
        }
        else
        {
            uint8_t* output_ptr = &filtered->data.front();
            const uint8_t *ptr = &cloud->data.front(), *ptr_end = &cloud->data.back(), *ptr_init; // NOLINT
            size_t points_to_copy = 0;
            for (; ptr < ptr_end; ptr += point_step) // NOLINT
            {
                float x = *reinterpret_cast<const float*>(ptr + xoff); // NOLINT
                float y = *reinterpret_cast<const float*>(ptr + yoff); // NOLINT
                float z = *reinterpret_cast<const float*>(ptr + zoff); // NOLINT
                if (validateFloats(x) && validateFloats(y) && validateFloats(z))
                {
                    if (points_to_copy == 0)
                    {
                        // Only memorize where to start copying from
                        ptr_init = ptr;
                        points_to_copy = 1;
                    }
                    else
                    {
                        ++points_to_copy;
                    }
                }
                else
                {
                    if (points_to_copy != 0u)
                    {
                        // Copy all the points that need to be copied
                        memcpy(output_ptr, ptr_init, point_step * points_to_copy);
                        output_ptr += point_step * points_to_copy; // NOLINT
                        points_to_copy = 0;
                    }
                }
            }
            // Don't forget to flush what needs to be copied
            if (points_to_copy != 0u)
            {
                memcpy(output_ptr, ptr_init, point_step * points_to_copy);
                output_ptr += point_step * points_to_copy; // NOLINT
            }
            output_count = (output_ptr - &filtered->data.front()) / point_step;
        }

        filtered->header = cloud->header;
        filtered->fields = cloud->fields;
        filtered->data.resize(output_count * point_step);
        filtered->height = 1;
        filtered->width = output_count;
        filtered->is_bigendian = cloud->is_bigendian;
        filtered->point_step = point_step;
        filtered->row_step = output_count;

        point_cloud_common_->addMessage(filtered);
    }

    void PointCloud2Display::update()
    {
        point_cloud_common_->update();
    }

    void PointCloud2Display::reset()
    {
        point_cloud_common_->reset();
    }

} // namespace rviz

