/**
    @file
    @author  Val Doroshchuk
    @copyright  2021
*/

#include "renderer.h"
#include "frame_manager.h"
#include "env_config.h"

#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreRenderWindow.h>
#include <OgreRenderTexture.h>
#include <OgreTextureManager.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreViewport.h>
#include <OgreGpuProgramManager.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <RTShaderSystem/OgreRTShaderSystem.h>

#include <ros/ros.h>
#include <ros/package.h>

static void detectGlVersion(Ogre::RenderSystem *render_sys, int *gl_version, int *glsl_version)
{
    std::unique_ptr<Ogre::RenderSystemCapabilities> caps(render_sys->createRenderSystemCapabilities());
    ROS_INFO("OpenGL device: %s", caps->getDeviceName().c_str());
    const int major = caps->getDriverVersion().major;
    const int minor = caps->getDriverVersion().minor;
    *gl_version = major * 100 + minor * 10;
    bool mesa_workaround = caps->getDeviceName().find("Mesa ") != std::string::npos && *gl_version >= 320;

    switch (*gl_version)
    {
      case 200:
        *glsl_version = 110;
        break;
      case 210:
        *glsl_version = 120;
        break;
      case 300:
        *glsl_version = 130;
        break;
      case 310:
        *glsl_version = 140;
        break;
      case 320:
        *glsl_version = 150;
        break;
      default:
        if (*gl_version > 320)
        {
          *glsl_version = *gl_version;
        }
        else
        {
          *glsl_version = 0;
        }
        break;
    }

    if (mesa_workaround)
    { // https://github.com/ros-visualization/rviz/issues/1508
        ROS_INFO("OpenGl version: %.1f (GLSL %.1f) limited to GLSL 1.4 on Mesa system.",
             (float)*gl_version / 100.0, (float)*glsl_version / 100.0);

        *gl_version = 310;
        *glsl_version = 140;
        return;
    }

    ROS_INFO("OpenGl version: %.1f (GLSL %.1f).", (float)*gl_version / 100.0, (float)*glsl_version / 100.0);
}

static Ogre::RenderSystem* rendererSystem(Ogre::Root *ogre_root)
{
    Ogre::RenderSystem* render_sys = nullptr;
    const Ogre::RenderSystemList *rs_list = nullptr;

    // Get the list of available renderers.
#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR == 6
    rs_list = ogre_root->getAvailableRenderers();
#else
    rs_list = &(ogre_root->getAvailableRenderers());
#endif

    // Look for the OpenGL one, which we require.
    auto it = std::find_if(rs_list->begin(), rs_list->end(), [](const Ogre::RenderSystem *sys) { return (sys->getName() == "OpenGL Rendering Subsystem"); });
    if (it != rs_list->end())
    {
        render_sys = *it;
    }

    return (render_sys);
}

static void loadResources(const std::string &rviz_path, int glsl_version)
{
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation(rviz_path + "/src/rviz/ogre_media", "FileSystem", ROS_PACKAGE_NAME);
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation(rviz_path + "/src/rviz/ogre_media/textures", "FileSystem", ROS_PACKAGE_NAME);
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation(rviz_path + "/src/rviz/ogre_media/materials", "FileSystem", ROS_PACKAGE_NAME);
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation(rviz_path + "/src/rviz/ogre_media/materials/scripts", "FileSystem", ROS_PACKAGE_NAME);
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation(rviz_path + "/src/rviz/ogre_media/materials/glsl120", "FileSystem", ROS_PACKAGE_NAME);
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation(rviz_path + "/src/rviz/ogre_media/materials/glsl120/include", "FileSystem", ROS_PACKAGE_NAME);
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation(rviz_path + "/src/rviz/ogre_media/materials/glsl120/nogp", "FileSystem", ROS_PACKAGE_NAME);

    if (glsl_version >= 150)
    {
        Ogre::ResourceGroupManager::getSingleton().addResourceLocation(rviz_path + "/src/rviz/ogre_media/materials/glsl150", "FileSystem", ROS_PACKAGE_NAME);
        Ogre::ResourceGroupManager::getSingleton().addResourceLocation(rviz_path + "/src/rviz/ogre_media/materials/scripts150", "FileSystem", ROS_PACKAGE_NAME);
    }
    else if (glsl_version >= 120)
    {
        Ogre::ResourceGroupManager::getSingleton().addResourceLocation(rviz_path + "/src/rviz/ogre_media/materials/scripts120", "FileSystem", ROS_PACKAGE_NAME);
    }
    else
    {
        throw std::runtime_error("Your graphics driver does not support OpenGL 2.1");
    }
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
}

static void loadPlugins(Ogre::Root *ogre_root)
{
    const std::string plugin_prefix = rviz::get_ogre_plugin_path() + "/";
    //ogre_root->loadPlugin(plugin_prefix + "RenderSystem_GLES2");
    ogre_root->loadPlugin(plugin_prefix + "RenderSystem_GL");
    ogre_root->loadPlugin(plugin_prefix + "Plugin_OctreeSceneManager");
    ogre_root->loadPlugin(plugin_prefix + "Plugin_ParticleFX");
#if (OGRE_VERSION_MAJOR >= 1 && OGRE_VERSION_MINOR >= 12)
    ogre_root->loadPlugin(plugin_prefix + "Codec_STBI");
#endif
}

namespace rviz
{
    Renderer::~Renderer()
    {
        if (scene_manager_ != nullptr)
        {
            scene_manager_->destroySceneNode(scene_node_);
        }
    }

    void Renderer::initialize(const std::string &fixed_frame)
    {
        frame_manager_.reset(new rviz::FrameManager);
        frame_manager_->setFixedFrame(fixed_frame);
        const std::string rviz_path = ros::package::getPath(ROS_PACKAGE_NAME);
        ogre_root_ = new Ogre::Root(rviz_path + "/src/rviz/ogre_media/plugins.cfg");

        loadPlugins(ogre_root_);

        Ogre::RenderSystem* render_sys = rendererSystem(ogre_root_);
        if (render_sys == nullptr)
        {
            throw std::runtime_error("Could not find the opengl rendering subsystem!\n");
        }

        ogre_root_->setRenderSystem(render_sys);
        ogre_root_->initialise(false);

        Ogre::NameValuePairList params;
        params["externalGLControl"] = "false";
        //params["hidden"] = "true";
        params["FSAA"] = "4";

        render_window_ = ogre_root_->createRenderWindow("Main", 1920, 1080, false, &params);

#if (OGRE_VERSION_MAJOR >= 1 && OGRE_VERSION_MINOR < 12)
        scene_manager_ = ogre_root_->createSceneManager(Ogre::ST_GENERIC);
#else
        scene_manager_ = ogre_root_->createSceneManager();
#endif
        scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
        detectGlVersion(render_sys, &gl_version_, &glsl_version_);
        loadResources(rviz_path, glsl_version_);
    }

    void Renderer::update()
    {
        ogre_root_->renderOneFrame();
        render_window_->update();
    }
} // namespace rviz
