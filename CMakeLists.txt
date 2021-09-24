cmake_minimum_required(VERSION 2.8.3)
project(obstacle_overlay_node LANGUAGES CXX)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    image_transport
    pcl_conversions
    pcl_ros    
)
find_package(OpenCV REQUIRED)
find_package(PCL 1.3 REQUIRED)

find_package(PkgConfig)
pkg_check_modules(OGRE_OV REQUIRED OGRE)

## Find OGRE Plugin path (not necessarily platform-independent, I guess)
if(NOT DEFINED OGRE_PLUGIN_PATH)
  execute_process(COMMAND
    ${PKG_CONFIG_EXECUTABLE} --variable=plugindir OGRE
    OUTPUT_VARIABLE OGRE_PLUGIN_PATH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )
endif()
message(STATUS OGRE_PLUGIN_PATH=${OGRE_PLUGIN_PATH})

set(ENV_CONFIG_FILE ${CMAKE_CURRENT_BINARY_DIR}/env_config.cpp)
configure_file(src/rviz/env_config.cpp.in ${ENV_CONFIG_FILE})

if(NOT OGRE_OV_LIBRARIES_ABS)
  set(OGRE_OV_LIBRARIES_ABS ${OGRE_OV_LIBRARIES})
endif()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#    INCLUDE_DIRS include
#    DEPENDS system_lib
)

###########
## Build ##
###########

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")
include_directories(
    SYSTEM ${catkin_INCLUDE_DIRS} ${OGRE_OV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_rviz src/rviz/renderer.cpp src/rviz/frame_manager.cpp src/rviz/image_display.cpp src/rviz/ros_image_texture.cpp src/rviz/point_cloud2_display.cpp src/rviz/point_cloud_transformers.cpp src/rviz/point_cloud_common.cpp src/rviz/ogre_helpers/point_cloud.cpp ${ENV_CONFIG_FILE})

add_executable(obstacle_overlay_node src/obstacle_overlay_node.cpp)
target_link_libraries(obstacle_overlay_node ${PROJECT_NAME}_rviz ${catkin_LIBRARIES} ${OGRE_OV_LIBRARIES_ABS})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS obstacle_overlay_node ${PROJECT_NAME}_rviz
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
