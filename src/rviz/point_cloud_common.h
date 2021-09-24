/**
    @file
    @author  Val Doroshchuk, Willow Garage, Inc.
    @copyright  2021
*/

#pragma once

#include <deque>
#include <queue>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include <ros/spinner.h>
#include <ros/callback_queue.h>

#include <message_filters/time_sequencer.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include "point_cloud_transformer.h"
#include "ogre_helpers/point_cloud.h"
#include "selection/forwards.h"
#include "frame_manager.h"

namespace rviz
{
    class Renderer;

    class PointCloudTransformer;
    typedef boost::shared_ptr<PointCloudTransformer> PointCloudTransformerPtr;

    typedef std::vector<std::string> V_string;

    class PointCloudCommon
    {
    public:
        struct CloudInfo
        {
            CloudInfo() = default;
            ~CloudInfo();

            // clear the point cloud, but keep selection handler around
            void clear();

            ros::Time receive_time_;

            Ogre::SceneManager* manager_ = nullptr;

            sensor_msgs::PointCloud2ConstPtr message_;

            Ogre::SceneNode* scene_node_ = nullptr;
            boost::shared_ptr<PointCloud> cloud_;

            std::vector<PointCloud::Point> transformed_points_;

            Ogre::Quaternion orientation_;
            Ogre::Vector3 position_;
        };

        typedef boost::shared_ptr<CloudInfo> CloudInfoPtr;
        typedef std::deque<CloudInfoPtr> D_CloudInfo;
        typedef std::vector<CloudInfoPtr> V_CloudInfo;
        typedef std::queue<CloudInfoPtr> Q_CloudInfo;
        typedef std::list<CloudInfoPtr> L_CloudInfo;

        explicit PointCloudCommon(const Renderer &renderer);
        ~PointCloudCommon();

        void fixedFrameChanged();
        void reset();
        void update();

        void addMessage(const sensor_msgs::PointCloudConstPtr& cloud);
        void addMessage(const sensor_msgs::PointCloud2ConstPtr& cloud);

        ros::CallbackQueueInterface* getCallbackQueue()
        {
            return &cbqueue_;
        }

        void setAutoSize(bool auto_size);

        void causeRetransform();

        void updateStyle();
        void updateBillboardSize();
        void updateAlpha();
        void updateXyzTransformer();
        void updateColorTransformer();

        bool transformCloud(const CloudInfoPtr& cloud);

        void processMessage(const sensor_msgs::PointCloud2ConstPtr& cloud);
        void updateStatus();

        PointCloudTransformerPtr getXYZTransformer(const sensor_msgs::PointCloud2ConstPtr& cloud);
        PointCloudTransformerPtr getColorTransformer(const sensor_msgs::PointCloud2ConstPtr& cloud);
        void retransform();
        void onTransformerOptions(V_string& ops, uint32_t mask);

        void loadTransformers();

        bool auto_size_ = false;
        ros::AsyncSpinner spinner_;
        ros::CallbackQueue cbqueue_;

        D_CloudInfo cloud_infos_;

        const Renderer &renderer_;

        V_CloudInfo new_cloud_infos_;
        boost::mutex new_clouds_mutex_;

        L_CloudInfo obsolete_cloud_infos_;

        struct TransformerInfo
        {
            PointCloudTransformerPtr transformer;
            std::string readable_name;
            std::string lookup_name;
        };
        typedef std::map<std::string, TransformerInfo> M_TransformerInfo;

        boost::recursive_mutex transformers_mutex_;
        M_TransformerInfo transformers_;
        bool new_xyz_transformer_ = false;
        bool new_color_transformer_ = false;
        bool needs_retransform_ = false;

        PointCloud::RenderMode mode_ = PointCloud::RM_BOXES;
        float world_size_ = 0.5;
        float pixel_size_ = 3;
        float alpha_ = 1.0;
        int decay_time_ = 0;
        std::string xyz_name_ = "XYZ";
        std::string color_name_ = "AxisColor";
  };

} // namespace rviz

