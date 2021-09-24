/**
    @file
    @author  Val Doroshchuk
    @copyright  2021
*/

#include "rviz/renderer.h"
#include "rviz/point_cloud2_display.h"
#include "rviz/image_display.h"

#include <ros/ros.h>

class Renderer
{
public:
    void initialize(const std::string &pcl_topic, const std::string &image_topic, ros::NodeHandle *nh)
    {
        renderer_.reset(new rviz::Renderer);
        renderer_->initialize("/mapper_internal");

        pcl_display_.reset(new rviz::PointCloud2Display);
        pcl_display_->initialize(pcl_topic, *renderer_, nh);

        image_display_.reset(new rviz::ImageDisplay);
        image_display_->initialize(image_topic, *renderer_, nh);
    }

    void spin()
    {
        ros::Rate rate(30);

        while (ros::ok())
        {
            ros::spinOnce();
            pcl_display_->update();
            image_display_->update();
            renderer_->update();
            rate.sleep();
        }
    }

private:
    std::unique_ptr<rviz::Renderer> renderer_;
    std::unique_ptr<rviz::PointCloud2Display> pcl_display_;
    std::unique_ptr<rviz::ImageDisplay> image_display_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_overlay_node");

    ros::NodeHandle nh;
    Renderer renderer;
    if (argc < 2)
    {
        ROS_ERROR("Occupied topic is not defined");
        return (EXIT_FAILURE);
    }

    if (argc < 3)
    {
        ROS_ERROR("Image topic is not defined");
        return (EXIT_FAILURE);
    }

    std::string pcl_topic = argv[1]; // NOLINT
    std::string image_topic = argv[2]; // NOLINT
    try
    {
        renderer.initialize(pcl_topic, image_topic, &nh);
        renderer.spin();
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Exception: '%s'", e.what());
        return (EXIT_FAILURE);
    }

    return (EXIT_SUCCESS);
}
