/**
    @file
    @author  Val Doroshchuk, Willow Garage, Inc.
    @copyright  2021
*/

#include "renderer.h"
#include "frame_manager.h"
#include "point_cloud_common.h"
#include "point_cloud_transformers.h"
#include "ogre_helpers/point_cloud.h"
#include "validate_floats.h"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreWireBoundingBox.h>

#include <ros/time.h>

namespace rviz
{
    PointCloudCommon::CloudInfo::~CloudInfo()
    {
        clear();
    }

    void PointCloudCommon::CloudInfo::clear()
    {
        if (scene_node_ != nullptr)
        {
            manager_->destroySceneNode(scene_node_);
            scene_node_ = nullptr;
        }
    }

    PointCloudCommon::PointCloudCommon(const Renderer &renderer)
        : spinner_(1, &cbqueue_), renderer_(renderer)
    {
        loadTransformers();

        updateStyle();
        updateBillboardSize();
        updateAlpha();

        spinner_.start();
    }

    PointCloudCommon::~PointCloudCommon()
    {
        spinner_.stop();
    }

    void PointCloudCommon::loadTransformers()
    {
        {
            PointCloudTransformerPtr trans(new AxisColorPCTransformer);
            TransformerInfo info;
            info.transformer = trans;
            info.readable_name = "AxisColor";
            info.lookup_name = "AxisColor";
            transformers_["AxisColor"] = info;
        }
        {
            PointCloudTransformerPtr trans(new XYZPCTransformer);
            TransformerInfo info;
            info.transformer = trans;
            info.readable_name = "XYZ";
            info.lookup_name = "XYZ";
            transformers_["XYZ"] = info;
        }
    }

    void PointCloudCommon::setAutoSize(bool auto_size)
    {
        auto_size_ = auto_size;
        for (unsigned i = 0; i < cloud_infos_.size(); i++) // NOLINT
        {
            cloud_infos_[i]->cloud_->setAutoSize(auto_size);
        }
    }


    void PointCloudCommon::updateAlpha()
    {
        for (unsigned i = 0; i < cloud_infos_.size(); i++) // NOLINT
        {
            bool per_point_alpha = findChannelIndex(cloud_infos_[i]->message_, "rgba") != -1;
            cloud_infos_[i]->cloud_->setAlpha(alpha_, per_point_alpha);
        }
    }


    void PointCloudCommon::updateStyle()
    {
        for (unsigned int i = 0; i < cloud_infos_.size(); i++) // NOLINT
        {
            cloud_infos_[i]->cloud_->setRenderMode(mode_);
        }
        updateBillboardSize();
    }

    void PointCloudCommon::updateBillboardSize()
    {
        float size;
        if (mode_ == PointCloud::RM_POINTS)
        {
            size = pixel_size_;
        }
        else
        {
            size = world_size_;
        }
        for (unsigned i = 0; i < cloud_infos_.size(); i++) // NOLINT
        {
            cloud_infos_[i]->cloud_->setDimensions(size, size, size);
        }
    }

    void PointCloudCommon::reset()
    {
        boost::mutex::scoped_lock lock(new_clouds_mutex_);
        cloud_infos_.clear();
        new_cloud_infos_.clear();
    }

    void PointCloudCommon::causeRetransform()
    {
        needs_retransform_ = true;
    }

    void PointCloudCommon::update()
    {
        if (needs_retransform_)
        {
            retransform();
            needs_retransform_ = false;
        }

        // instead of deleting cloud infos, we just clear them
        // and put them into obsolete_cloud_infos, so active selections
        // are preserved

        auto now_sec = ros::Time::now().toSec();

        // if decay time == 0, clear the old cloud when we get a new one
        // otherwise, clear all the outdated ones
        {
            boost::mutex::scoped_lock lock(new_clouds_mutex_);
            if (decay_time_ > 0.0 || !new_cloud_infos_.empty())
            {
                while (!cloud_infos_.empty() &&
                    now_sec - cloud_infos_.front()->receive_time_.toSec() >= decay_time_)
                {
                    cloud_infos_.front()->clear();
                    obsolete_cloud_infos_.push_back(cloud_infos_.front());
                    cloud_infos_.pop_front();
                    //context_->queueRender();
                }
            }
        }

        {
            boost::mutex::scoped_lock lock(new_clouds_mutex_);
            if (!new_cloud_infos_.empty())
            {
                float size;
                if (mode_ == PointCloud::RM_POINTS)
                {
                    size = pixel_size_;
                }
                else
                {
                    size = world_size_;
                }

                V_CloudInfo::iterator it = new_cloud_infos_.begin();
                V_CloudInfo::iterator end = new_cloud_infos_.end();
                for (; it != end; ++it)
                {
                    CloudInfoPtr cloud_info = *it;

                    V_CloudInfo::iterator next = it;
                    next++;
                    // ignore point clouds that are too old, but keep at least one
                    if (next != end && now_sec - cloud_info->receive_time_.toSec() >= decay_time_)
                    {
                        continue;
                    }

                    bool per_point_alpha = findChannelIndex(cloud_info->message_, "rgba") != -1;

                    cloud_info->cloud_.reset(new PointCloud());
                    cloud_info->cloud_->setRenderMode(mode_);
                    cloud_info->cloud_->addPoints(&(cloud_info->transformed_points_.front()),
                                                  cloud_info->transformed_points_.size());
                    cloud_info->cloud_->setAlpha(alpha_, per_point_alpha);
                    cloud_info->cloud_->setDimensions(size, size, size);
                    cloud_info->cloud_->setAutoSize(auto_size_);

                    cloud_info->manager_ = renderer_.scene_manager_;

                    cloud_info->scene_node_ =
                        renderer_.scene_node_->createChildSceneNode(cloud_info->position_, cloud_info->orientation_);

                    cloud_info->scene_node_->attachObject(cloud_info->cloud_.get());

                    cloud_infos_.push_back(*it);
                }

                new_cloud_infos_.clear();
            }
        }

        {
            boost::recursive_mutex::scoped_try_lock lock(transformers_mutex_);

            new_xyz_transformer_ = false;
            new_color_transformer_ = false;
        }
    }

    void PointCloudCommon::processMessage(const sensor_msgs::PointCloud2ConstPtr& cloud)
    {
        CloudInfoPtr info(new CloudInfo);
        info->message_ = cloud;
        info->receive_time_ = ros::Time::now();

        if (transformCloud(info))
        {
            boost::mutex::scoped_lock lock(new_clouds_mutex_);
            new_cloud_infos_.push_back(info);
        }
    }

    void PointCloudCommon::updateXyzTransformer()
    {
        boost::recursive_mutex::scoped_lock lock(transformers_mutex_);
        if (transformers_.count(xyz_name_) == 0)
        {
            return;
        }
        new_xyz_transformer_ = true;
        causeRetransform();
    }

    void PointCloudCommon::updateColorTransformer()
    {
        boost::recursive_mutex::scoped_lock lock(transformers_mutex_);
        if (transformers_.count(color_name_) == 0)
        {
            return;
        }
        new_color_transformer_ = true;
        causeRetransform();
    }

    PointCloudTransformerPtr
    PointCloudCommon::getXYZTransformer(const sensor_msgs::PointCloud2ConstPtr& cloud)
    {
        boost::recursive_mutex::scoped_lock lock(transformers_mutex_);
        M_TransformerInfo::iterator it = transformers_.find(xyz_name_);
        if (it != transformers_.end())
        {
            const PointCloudTransformerPtr& trans = it->second.transformer;
            if ((trans->supports(cloud) & PointCloudTransformer::Support_XYZ) != 0)
            {
                return trans;
            }
        }

        return PointCloudTransformerPtr();
    }

    PointCloudTransformerPtr
    PointCloudCommon::getColorTransformer(const sensor_msgs::PointCloud2ConstPtr& cloud)
    {
        boost::recursive_mutex::scoped_lock lock(transformers_mutex_);
        M_TransformerInfo::iterator it = transformers_.find(color_name_);
        if (it != transformers_.end())
        {
            const PointCloudTransformerPtr& trans = it->second.transformer;
            if ((trans->supports(cloud) & PointCloudTransformer::Support_Color) != 0)
            {
                return trans;
            }
        }

        return PointCloudTransformerPtr();
    }


    void PointCloudCommon::retransform()
    {
        boost::recursive_mutex::scoped_lock lock(transformers_mutex_);

        D_CloudInfo::iterator it = cloud_infos_.begin();
        D_CloudInfo::iterator end = cloud_infos_.end();
        for (; it != end; ++it)
        {
            const CloudInfoPtr& cloud_info = *it;
            transformCloud(cloud_info);
            cloud_info->cloud_->clear();
            cloud_info->cloud_->addPoints(&cloud_info->transformed_points_.front(),
                                          cloud_info->transformed_points_.size());
        }
    }

    bool PointCloudCommon::transformCloud(const CloudInfoPtr& cloud_info)
    {
        if (cloud_info->scene_node_ == nullptr)
        {
            if (!renderer_.frame_manager_->getTransform(cloud_info->message_->header, cloud_info->position_,
                    cloud_info->orientation_))
            {
                ROS_ERROR_STREAM("Failed to transform from frame [" << cloud_info->message_->header.frame_id << "] to frame ["
                    << renderer_.frame_manager_->getFixedFrame() << "]");
                return false;
            }
        }

        Ogre::Matrix4 transform;
        transform.makeTransform(cloud_info->position_, Ogre::Vector3(1, 1, 1), cloud_info->orientation_);

        V_PointCloudPoint& cloud_points = cloud_info->transformed_points_;
        cloud_points.clear();

        size_t size = cloud_info->message_->width * cloud_info->message_->height;
        PointCloud::Point default_pt;
        default_pt.color = Ogre::ColourValue(1, 1, 1);
        default_pt.position = Ogre::Vector3::ZERO;
        cloud_points.resize(size, default_pt);

        {
            boost::recursive_mutex::scoped_lock lock(transformers_mutex_);
            PointCloudTransformerPtr xyz_trans = getXYZTransformer(cloud_info->message_);
            PointCloudTransformerPtr color_trans = getColorTransformer(cloud_info->message_);

            if (!xyz_trans)
            {
                ROS_ERROR("No position transformer available for cloud");
                return false;
            }

            if (!color_trans)
            {
                ROS_ERROR("No color transformer available for cloud");
                return false;
            }

            xyz_trans->transform(cloud_info->message_, PointCloudTransformer::Support_XYZ, transform,
                             cloud_points);
            color_trans->transform(cloud_info->message_, PointCloudTransformer::Support_Color, transform,
                               cloud_points);
        }

        for (V_PointCloudPoint::iterator cloud_point = cloud_points.begin(); cloud_point != cloud_points.end(); ++cloud_point) // NOLINT
        {
            if (!validateFloats(cloud_point->position))
            {
                cloud_point->position.x = 999999.0f;
                cloud_point->position.y = 999999.0f;
                cloud_point->position.z = 999999.0f;
            }
        }

        return true;
    }

    bool convertPointCloudToPointCloud2(const sensor_msgs::PointCloud& input,
                                        sensor_msgs::PointCloud2& output)
    {
        output.header = input.header;
        output.width = input.points.size();
        output.height = 1;
        output.fields.resize(3 + input.channels.size());
        // Convert x/y/z to fields
        output.fields[0].name = "x";
        output.fields[1].name = "y";
        output.fields[2].name = "z";
        int offset = 0;
        // All offsets are *4, as all field data types are float32
        for (size_t d = 0; d < output.fields.size(); ++d, offset += 4)
        {
            output.fields[d].offset = offset;
            output.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
        }
        output.point_step = offset;
        output.row_step = output.point_step * output.width;
        // Convert the remaining of the channels to fields
        for (size_t d = 0; d < input.channels.size(); ++d)
        {
            output.fields[3 + d].name = input.channels[d].name;
        }
        output.data.resize(input.points.size() * output.point_step);
        output.is_bigendian = 0u; // @todo ?
        output.is_dense = 0u;

        // Copy the data points
        for (size_t cp = 0; cp < input.points.size(); ++cp)
        {
            memcpy(&output.data[cp * output.point_step + output.fields[0].offset], &input.points[cp].x,
               sizeof(float));
            memcpy(&output.data[cp * output.point_step + output.fields[1].offset], &input.points[cp].y,
               sizeof(float));
            memcpy(&output.data[cp * output.point_step + output.fields[2].offset], &input.points[cp].z,
               sizeof(float));
            for (size_t d = 0; d < input.channels.size(); ++d)
            {
                if (input.channels[d].values.size() == input.points.size())
                {
                    memcpy(&output.data[cp * output.point_step + output.fields[3 + d].offset],
                        &input.channels[d].values[cp], sizeof(float));
                }
            }
        }
        return (true);
    }

    void PointCloudCommon::addMessage(const sensor_msgs::PointCloudConstPtr& cloud)
    {
        sensor_msgs::PointCloud2Ptr out(new sensor_msgs::PointCloud2);
        convertPointCloudToPointCloud2(*cloud, *out);
        addMessage(out);
    }

    void PointCloudCommon::addMessage(const sensor_msgs::PointCloud2ConstPtr& cloud)
    {
        processMessage(cloud);
    }

    void PointCloudCommon::fixedFrameChanged()
    {
        reset();
    }

} // namespace rviz
