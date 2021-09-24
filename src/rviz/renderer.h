/**
    @file
    @author  Val Doroshchuk
    @copyright  2021
*/

#pragma once

#include <string>
#include <memory>

namespace Ogre
{
    class Root;
    class RenderWindow;
    class SceneManager;
    class SceneNode;
}

namespace rviz
{
    class FrameManager;
    class Renderer
    {
    public:
        Renderer() = default;
        ~Renderer();

        void initialize(const std::string &fixed_frame);
        void update();

        Ogre::Root *ogre_root_ = nullptr;
        Ogre::RenderWindow *render_window_ = nullptr;
        Ogre::SceneManager *scene_manager_ = nullptr;
        Ogre::SceneNode *scene_node_ = nullptr;
        std::unique_ptr<rviz::FrameManager> frame_manager_;

        int glsl_version_ = 0;
        int gl_version_ = 0;

    };
} // namespace rviz

