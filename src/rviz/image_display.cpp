/**
    @file
    @author  Val Doroshchuk
    @copyright  2021
*/

#include "image_display.h"
#include "frame_manager.h"
#include "renderer.h"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>
#include <OgreMaterialManager.h>
#include <OgreMaterial.h>
#include <OgreRenderWindow.h>
#include <OgreCamera.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreRectangle2D.h>
#include <OgreTechnique.h>
#include <OgreRenderTexture.h>
#include <OgreTextureManager.h>

#include <sensor_msgs/image_encodings.h>

namespace rviz
{
    void ImageDisplay::initialize(const std::string &topic, const Renderer &renderer, ros::NodeHandle *nh)
    {
        renderer_ = &renderer;
        it_.reset(new image_transport::ImageTransport(*nh));
        sub_.reset(new image_transport::SubscriberFilter);
        tf_filter_.reset(new tf2_ros::MessageFilter<sensor_msgs::Image>(
                         *sub_, *renderer_->frame_manager_->getTF2BufferPtr(), renderer_->frame_manager_->getFixedFrame(), 10, *nh));
        tf_filter_->registerCallback(boost::bind(&ImageDisplay::cb_image, this, _1));
        sub_->subscribe(*it_, topic, 10);
        std::string camera_info_topic = image_transport::getCameraInfoTopic(topic);
        sub_info_.subscribe(*nh, camera_info_topic, 1);
        sub_info_.registerCallback(boost::bind(&ImageDisplay::cb_cameraInfo, this, _1));
        image_pub_ = it_->advertiseCamera(topic + "/rendered/image", 1);

        camera_ = renderer_->scene_manager_->createCamera("ImageDisplayCamera");
        camera_->setNearClipDistance(0.01f);
        if (renderer_->render_window_ != nullptr)
        {
            viewport_ = renderer_->render_window_->addViewport(camera_);
            viewport_->setOverlaysEnabled(false);
            viewport_->setBackgroundColour(Ogre::ColourValue::Black);
            viewport_->setClearEveryFrame(true);
        }
        scene_node_ = renderer_->scene_node_->createChildSceneNode("ImageDisplay::scene_node_");
        scene_node_->attachObject(camera_);

        bg_scene_node_ = scene_node_->createChildSceneNode("ImageDisplay::bg_scene_node_");
        fg_scene_node_ = scene_node_->createChildSceneNode("ImageDisplay::fg_scene_node_");

        {
            static int count = 0;
            std::stringstream ss;
            ss << "ImageDisplay" << count++;

            // background rectangle
            bg_screen_rect_ = new Ogre::Rectangle2D(true);
            bg_screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);

            ss << "Material";
            bg_material_ = Ogre::MaterialManager::getSingleton().create(
                ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

            bg_material_->setDepthWriteEnabled(false);
            bg_material_->setReceiveShadows(false);
            bg_material_->setDepthCheckEnabled(false);

            bg_material_->getTechnique(0)->setLightingEnabled(false);
            Ogre::TextureUnitState* tu = bg_material_->getTechnique(0)->getPass(0)->createTextureUnitState();
            tu->setTextureName(texture_.getTexture()->getName());
            tu->setTextureFiltering(Ogre::TFO_NONE);
            tu->setAlphaOperation(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, 0.0);

            bg_material_->setCullingMode(Ogre::CULL_NONE);
            bg_material_->setSceneBlending(Ogre::SBT_REPLACE);

            Ogre::AxisAlignedBox aabInf;
            aabInf.setInfinite();

            bg_screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_BACKGROUND);
            bg_screen_rect_->setBoundingBox(aabInf);
#if (OGRE_VERSION_MAJOR >= 1 && OGRE_VERSION_MINOR < 12)
            bg_screen_rect_->setMaterial(bg_material_->getName());
#else
            bg_screen_rect_->setMaterial(bg_material_);
#endif

            bg_scene_node_->attachObject(bg_screen_rect_);
            bg_scene_node_->setVisible(false);

            // overlay rectangle
            fg_screen_rect_ = new Ogre::Rectangle2D(true);
            fg_screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);

            fg_material_ = bg_material_->clone(ss.str() + "fg");
            fg_screen_rect_->setBoundingBox(aabInf);
#if (OGRE_VERSION_MAJOR >= 1 && OGRE_VERSION_MINOR < 12)
            fg_screen_rect_->setMaterial(fg_material_->getName());
#else
            fg_screen_rect_->setMaterial(fg_material_);
#endif

            fg_material_->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
            fg_screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);

            fg_scene_node_->attachObject(fg_screen_rect_);
            fg_scene_node_->setVisible(false);
        }

        const float alpha = 0.5;
        Ogre::Pass* pass = fg_material_->getTechnique(0)->getPass(0);
        if (pass->getNumTextureUnitStates() > 0)
        {
            Ogre::TextureUnitState* tex_unit = pass->getTextureUnitState(0);
            tex_unit->setAlphaOperation(Ogre::LBX_MODULATE, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, alpha);
        }
        else
        {
            fg_material_->setAmbient(Ogre::ColourValue(0.0f, 1.0f, 1.0f, alpha));
            fg_material_->setDiffuse(Ogre::ColourValue(0.0f, 1.0f, 1.0f, alpha));
        }

        if (renderer_->render_window_ != nullptr)
        {
            renderer_->render_window_->addListener(this);
        }
    }

    ImageDisplay::~ImageDisplay()
    {
        if (bg_scene_node_ != nullptr)
        {
            bg_scene_node_->getParentSceneNode()->removeAndDestroyChild(bg_scene_node_->getName());
        }
        if (fg_scene_node_ != nullptr)
        {
            fg_scene_node_->getParentSceneNode()->removeAndDestroyChild(fg_scene_node_->getName());
        }
        if (scene_node_ != nullptr)
        {
            scene_node_->getParentSceneNode()->removeAndDestroyChild(scene_node_->getName());
        }
        if (renderer_ != nullptr)
        {
            if (renderer_->render_window_ != nullptr)
            {
                renderer_->render_window_->removeListener(this);
            }
            renderer_->scene_manager_->destroyCamera(camera_);
        }
    }

    void ImageDisplay::preRenderTargetUpdate(const Ogre::RenderTargetEvent &/*evt*/)
    {
        bg_scene_node_->setVisible(camera_ok_);
        fg_scene_node_->setVisible(camera_ok_);
    }

    void ImageDisplay::postRenderTargetUpdate(const Ogre::RenderTargetEvent &/*evt*/)
    {
        bg_scene_node_->setVisible(false);
        fg_scene_node_->setVisible(false);

        if (camera_info_ == nullptr || render_texture_ == nullptr)
        {
            return;
        }

        int height = render_texture_->getHeight();
        int width = render_texture_->getWidth();

        sensor_msgs::ImagePtr image(new sensor_msgs::Image);
        image->encoding = sensor_msgs::image_encodings::RGB8;
        Ogre::PixelFormat pf = Ogre::PF_BYTE_RGB;
        uint pixelsize = Ogre::PixelUtil::getNumElemBytes(pf);
        uint datasize = width * height * pixelsize;
        unsigned char *data = OGRE_ALLOC_T(unsigned char, datasize, Ogre::MEMCATEGORY_RENDERSYS);
        Ogre::PixelBox pb(width, height, 1, pf, data);
#if (OGRE_VERSION_MAJOR >= 1 && OGRE_VERSION_MINOR < 12)
        render_texture_->copyContentsToMemory(pb, Ogre::RenderTarget::FB_AUTO);
#else
        render_texture_->copyContentsToMemory(Ogre::Box(0, 0, width, height), pb, Ogre::RenderTarget::FB_AUTO);
#endif

        image->header = camera_info_->header;
        image->height = height;
        image->width = width;
        image->step = pixelsize * width;
        image->is_bigendian = static_cast<int>(OGRE_ENDIAN == OGRE_ENDIAN_BIG);
        image->data.resize(datasize);
        memcpy(&image->data[0], data, datasize);

        image_pub_.publish(*image, *camera_info_);

        OGRE_FREE(data, Ogre::MEMCATEGORY_RENDERSYS);
    }

    void ImageDisplay::cb_image(const sensor_msgs::Image::ConstPtr &msg)
    {
        texture_.addMessage(msg);
    }

    void ImageDisplay::cb_cameraInfo(const sensor_msgs::CameraInfo::ConstPtr& msg)
    {
        camera_info_ = msg;
    }

    bool ImageDisplay::updateCamera()
    {
        sensor_msgs::Image::ConstPtr image = texture_.getImage();
        if (!camera_info_ || !image)
        {
            return (false);
        }
#if (OGRE_VERSION_MAJOR >= 1 && OGRE_VERSION_MINOR < 12)
        bool texture_ok = rtt_texture_.isNull() == false;
#else
        bool texture_ok = static_cast<bool>(rtt_texture_);
#endif
        if (texture_ok == false || camera_info_->width != render_texture_->getWidth() || camera_info_->height != render_texture_->getHeight())
        {
            rtt_texture_ = Ogre::TextureManager::getSingleton().createManual(
              "RttTex",
              Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
              Ogre::TEX_TYPE_2D,
              camera_info_->width, camera_info_->height,
              0,
              Ogre::PF_R8G8B8,
              Ogre::TU_RENDERTARGET);
            render_texture_ = rtt_texture_->getBuffer()->getRenderTarget();
            render_texture_->addViewport(camera_);
            //render_texture_->getViewport(0)->setVisibilityMask(vis_bit_);
            render_texture_->getViewport(0)->setClearEveryFrame(true);
            render_texture_->getViewport(0)->setBackgroundColour(Ogre::ColourValue::Black);
            render_texture_->setAutoUpdated(false);
            render_texture_->setActive(true);
            render_texture_->addListener(this);
        }

        Ogre::Vector3 position;
        Ogre::Quaternion orientation;
        if (!renderer_->frame_manager_->getTransform(image->header, position, orientation))
        {
            ROS_ERROR("Could not find image transform");
            return (false);
        }

        orientation = orientation * Ogre::Quaternion(Ogre::Degree(180), Ogre::Vector3::UNIT_X);
        float img_width = camera_info_->width;
        float img_height = camera_info_->height;

        double fx = camera_info_->P[0];
        double fy = camera_info_->P[5];

        float zoom_x = 1;
        float zoom_y = zoom_x;
        double tx = -1 * (camera_info_->P[3] / fx);
        Ogre::Vector3 right = orientation * Ogre::Vector3::UNIT_X;
        position = position + (right * tx);

        double ty = -1 * (camera_info_->P[7] / fy);
        Ogre::Vector3 down = orientation * Ogre::Vector3::UNIT_Y;
        position = position + (down * ty);

        scene_node_->setPosition(position);
        scene_node_->setOrientation(orientation);

        // calculate the projection matrix
        double cx = camera_info_->P[2];
        double cy = camera_info_->P[6];

        double far_plane = 100;
        double near_plane = 0.01;

        Ogre::Matrix4 proj_matrix;
        proj_matrix = Ogre::Matrix4::ZERO;

        proj_matrix[0][0] = 2.0 * fx / img_width * zoom_x; // NOLINT
        proj_matrix[1][1] = 2.0 * fy / img_height * zoom_y; // NOLINT

        proj_matrix[0][2] = 2.0 * (0.5 - cx / img_width) * zoom_x; // NOLINT
        proj_matrix[1][2] = 2.0 * (cy / img_height - 0.5) * zoom_y; // NOLINT

        proj_matrix[2][2] = -(far_plane + near_plane) / (far_plane - near_plane); // NOLINT
        proj_matrix[2][3] = -2.0 * far_plane * near_plane / (far_plane - near_plane); // NOLINT

        proj_matrix[3][2] = -1; // NOLINT

        camera_->setCustomProjectionMatrix(true, proj_matrix);

        // adjust the image rectangles to fit the zoom & aspect ratio
        double x_corner_start;
        double y_corner_start;
        double x_corner_end;
        double y_corner_end;

        if (camera_info_->roi.height != 0 || camera_info_->roi.width != 0)
        {
            // corners are computed according to roi
            x_corner_start = (2.0 * camera_info_->roi.x_offset / camera_info_->width - 1.0) * zoom_x;
            y_corner_start = (-2.0 * camera_info_->roi.y_offset / camera_info_->height + 1.0) * zoom_y;
            x_corner_end = x_corner_start + (2.0 * camera_info_->roi.width / camera_info_->width) * zoom_x;
            y_corner_end = y_corner_start - (2.0 * camera_info_->roi.height / camera_info_->height) * zoom_y;
        }
        else
        {
            x_corner_start = -1.0f * zoom_x;
            y_corner_start = 1.0f * zoom_y;
            x_corner_end = 1.0f * zoom_x;
            y_corner_end = -1.0f * zoom_y;
        }

        bg_screen_rect_->setCorners(x_corner_start, y_corner_start, x_corner_end, y_corner_end);
        fg_screen_rect_->setCorners(x_corner_start, y_corner_start, x_corner_end, y_corner_end);

        Ogre::AxisAlignedBox aabInf;
        aabInf.setInfinite();
        bg_screen_rect_->setBoundingBox(aabInf);
        fg_screen_rect_->setBoundingBox(aabInf);

        return (true);
    }

    void ImageDisplay::update()
    {
        if (texture_.update())
        {
            camera_ok_ = updateCamera();
        }

        if (render_texture_ != nullptr)
        {
            render_texture_->update();
        }
    }

} // namespace rviz

