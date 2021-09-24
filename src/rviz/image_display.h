/**
    @file
    @author  Val Doroshchuk
    @copyright  2021
*/

#pragma once

#include "ros_image_texture.h"

#include <message_filters/subscriber.h>
#include <tf2_ros/message_filter.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_common.h>
#include <image_transport/subscriber_filter.h>

#include <OgreRenderTargetListener.h>

namespace Ogre
{
    class Camera;
    class Viewport;
    class SceneNode;
    class Rectangle2D;
    class RenderTexture;
}

namespace rviz
{
    class Renderer;
    class ImageDisplay : public Ogre::RenderTargetListener
    {
    public:
        ~ImageDisplay();

        void update();
        void initialize(const std::string &topic, const Renderer &renderer, ros::NodeHandle *nh);

        void preRenderTargetUpdate(const Ogre::RenderTargetEvent &) override;
        void postRenderTargetUpdate(const Ogre::RenderTargetEvent &) override;

    private:
        bool updateCamera();
        void cb_image(const sensor_msgs::Image::ConstPtr &msg);
        void cb_cameraInfo(const sensor_msgs::CameraInfo::ConstPtr& msg);

        const Renderer *renderer_ = nullptr;
        Ogre::SceneNode *scene_node_ = nullptr;
        Ogre::Camera *camera_ = nullptr;
        Ogre::Viewport *viewport_ = nullptr;
        std::unique_ptr<image_transport::ImageTransport> it_;
        std::unique_ptr<image_transport::SubscriberFilter> sub_;  
        std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::Image>> tf_filter_;
        message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_;
        sensor_msgs::CameraInfo::ConstPtr camera_info_;
        image_transport::CameraPublisher image_pub_;

        ROSImageTexture texture_;
        bool camera_ok_ = false;

        Ogre::SceneNode *bg_scene_node_ = nullptr;
        Ogre::SceneNode *fg_scene_node_ = nullptr;

        Ogre::Rectangle2D *bg_screen_rect_ = nullptr;
        Ogre::MaterialPtr bg_material_;

        Ogre::Rectangle2D *fg_screen_rect_ = nullptr;
        Ogre::MaterialPtr fg_material_;

        Ogre::TexturePtr rtt_texture_;
        Ogre::RenderTexture *render_texture_ = nullptr; 
    };

} // namespace rviz

