/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef POINT_CLOUD_TRANSFORMERS_H1
#define POINT_CLOUD_TRANSFORMERS_H1

#include <sensor_msgs/PointCloud2.h>

#include "point_cloud_transformer.h"

namespace rviz
{

typedef std::vector<std::string> V_string;

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

template <typename T>
inline T valueFromCloud(const sensor_msgs::PointCloud2ConstPtr& cloud,
                        uint32_t offset,
                        uint8_t type,
                        uint32_t point_step,
                        uint32_t index)
{
  const uint8_t* data = &cloud->data[(point_step * index) + offset];
  T ret = 0;

  switch (type)
  {
  case sensor_msgs::PointField::INT8:
  case sensor_msgs::PointField::UINT8:
  {
    uint8_t val = *reinterpret_cast<const uint8_t*>(data);
    ret = static_cast<T>(val);
    break;
  }

  case sensor_msgs::PointField::INT16:
  case sensor_msgs::PointField::UINT16:
  {
    uint16_t val = *reinterpret_cast<const uint16_t*>(data);
    ret = static_cast<T>(val);
    break;
  }

  case sensor_msgs::PointField::INT32:
  case sensor_msgs::PointField::UINT32:
  {
    uint32_t val = *reinterpret_cast<const uint32_t*>(data);
    ret = static_cast<T>(val);
    break;
  }

  case sensor_msgs::PointField::FLOAT32:
  {
    float val = *reinterpret_cast<const float*>(data);
    ret = static_cast<T>(val);
    break;
  }

  case sensor_msgs::PointField::FLOAT64:
  {
    double val = *reinterpret_cast<const double*>(data);
    ret = static_cast<T>(val);
    break;
  }
  default:
    break;
  }

  return ret;
}

class IntensityPCTransformer : public PointCloudTransformer
{
public:
  uint8_t supports(const sensor_msgs::PointCloud2ConstPtr& cloud) override;
  bool transform(const sensor_msgs::PointCloud2ConstPtr& cloud,
                 uint32_t mask,
                 const Ogre::Matrix4& transform,
                 V_PointCloudPoint& points_out) override;
  uint8_t score(const sensor_msgs::PointCloud2ConstPtr& cloud) override;
  void updateChannels(const sensor_msgs::PointCloud2ConstPtr& cloud);

private:
  V_string available_channels_;
};

class XYZPCTransformer : public PointCloudTransformer
{
public:
  uint8_t supports(const sensor_msgs::PointCloud2ConstPtr& cloud) override;
  bool transform(const sensor_msgs::PointCloud2ConstPtr& cloud,
                 uint32_t mask,
                 const Ogre::Matrix4& transform,
                 V_PointCloudPoint& points_out) override;
};


class RGB8PCTransformer : public PointCloudTransformer
{
public:
  uint8_t supports(const sensor_msgs::PointCloud2ConstPtr& cloud) override;
  bool transform(const sensor_msgs::PointCloud2ConstPtr& cloud,
                 uint32_t mask,
                 const Ogre::Matrix4& transform,
                 V_PointCloudPoint& points_out) override;
};


class MONO8PCTransformer : public RGB8PCTransformer
{
public:
  bool transform(const sensor_msgs::PointCloud2ConstPtr& cloud,
                 uint32_t mask,
                 const Ogre::Matrix4& transform,
                 V_PointCloudPoint& points_out) override;
};


class RGBF32PCTransformer : public PointCloudTransformer
{
public:
  uint8_t supports(const sensor_msgs::PointCloud2ConstPtr& cloud) override;
  bool transform(const sensor_msgs::PointCloud2ConstPtr& cloud,
                 uint32_t mask,
                 const Ogre::Matrix4& transform,
                 V_PointCloudPoint& points_out) override;
};


class FlatColorPCTransformer : public PointCloudTransformer
{
public:
  uint8_t supports(const sensor_msgs::PointCloud2ConstPtr& cloud) override;
  bool transform(const sensor_msgs::PointCloud2ConstPtr& cloud,
                 uint32_t mask,
                 const Ogre::Matrix4& transform,
                 V_PointCloudPoint& points_out) override;
  uint8_t score(const sensor_msgs::PointCloud2ConstPtr& cloud) override;
};

class AxisColorPCTransformer : public PointCloudTransformer
{
public:
  uint8_t supports(const sensor_msgs::PointCloud2ConstPtr& cloud) override;
  bool transform(const sensor_msgs::PointCloud2ConstPtr& cloud,
                 uint32_t mask,
                 const Ogre::Matrix4& transform,
                 V_PointCloudPoint& points_out) override;
  uint8_t score(const sensor_msgs::PointCloud2ConstPtr& cloud) override;

  enum Axis
  {
    AXIS_X,
    AXIS_Y,
    AXIS_Z
  };
};

} // namespace rviz

#endif // POINT_CLOUD_TRANSFORMERS_H
