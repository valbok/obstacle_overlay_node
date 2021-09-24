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

#include <OgreColourValue.h>
#include <OgreMatrix4.h>
#include <OgreVector3.h>

#include "validate_floats.h"

#include "point_cloud_transformers.h"

namespace rviz
{
static void getRainbowColor(float value, Ogre::ColourValue& color)
{
  // this is HSV color palette with hue values going only from 0.0 to 0.833333.

  value = std::min(value, 1.0f);
  value = std::max(value, 0.0f);

  float h = value * 5.0f + 1.0f;
  int i = floor(h);
  float f = h - float(i);
  if ((i & 1) == 0)
  {
    f = 1 - f; // if i is even
  }
  float n = 1 - f;

  if (i <= 1)
  {
    color[0] = n, color[1] = 0, color[2] = 1;
  }
  else if (i == 2)
  {
    color[0] = 0, color[1] = n, color[2] = 1;
  }
  else if (i == 3)
  {
    color[0] = 0, color[1] = 1, color[2] = n;
  }
  else if (i == 4)
  {
    color[0] = n, color[1] = 1, color[2] = 0;
  }
  else if (i >= 5)
  {
    color[0] = 1, color[1] = n, color[2] = 0;
  }
}

uint8_t IntensityPCTransformer::supports(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  updateChannels(cloud);
  return Support_Color;
}

uint8_t IntensityPCTransformer::score(const sensor_msgs::PointCloud2ConstPtr& /*cloud*/)
{
  return 255;
}

bool IntensityPCTransformer::transform(const sensor_msgs::PointCloud2ConstPtr& cloud,
                                       uint32_t mask,
                                       const Ogre::Matrix4& /*transform*/,
                                       V_PointCloudPoint& points_out)
{
  if ((mask & Support_Color) == 0u)
  {
    return false;
  }

  int32_t index = findChannelIndex(cloud, "intensity");

  if (index == -1)
  {
    const bool intensity = true;
    if (intensity)
    {
      index = findChannelIndex(cloud, "intensities");
      if (index == -1)
      {
        return false;
      }
    }
    else
    {
      return false;
    }
  }

  const uint32_t offset = cloud->fields[index].offset;
  const uint8_t type = cloud->fields[index].datatype;
  const uint32_t point_step = cloud->point_step;
  const uint32_t num_points = cloud->width * cloud->height;

  float min_intensity = 999999.0f;
  float max_intensity = -999999.0f;
  const bool auto_compute_intensity_bounds_property_ = true;
  if (auto_compute_intensity_bounds_property_)
  {
    for (uint32_t i = 0; i < num_points; ++i)
    {
      float val = valueFromCloud<float>(cloud, offset, type, point_step, i);
      min_intensity = std::min(val, min_intensity);
      max_intensity = std::max(val, max_intensity);
    }

    min_intensity = std::max(-999999.0f, min_intensity);
    max_intensity = std::min(999999.0f, max_intensity);
    //min_intensity_property_->setFloat(min_intensity);
    //max_intensity_property_->setFloat(max_intensity);
  }
  else
  {
    //min_intensity = min_intensity_property_->getFloat();
    //max_intensity = max_intensity_property_->getFloat();
  }
  float diff_intensity = max_intensity - min_intensity;
  if (diff_intensity == 0)
  {
    // If min and max are equal, set the diff to something huge so
    // when we divide by it, we effectively get zero.  That way the
    // point cloud coloring will be predictably uniform when min and
    // max are equal.
    diff_intensity = 1e20;
  }
  Ogre::ColourValue max_color;// = max_color_property_->getOgreColor();
  Ogre::ColourValue min_color;// = min_color_property_->getOgreColor();
  const bool use_rainbow_property = true;
  if (use_rainbow_property)
  {
    for (uint32_t i = 0; i < num_points; ++i)
    {
      float val = valueFromCloud<float>(cloud, offset, type, point_step, i);
      float value = 1.0f - (val - min_intensity) / diff_intensity;
      const bool invert_rainbow_property = false;
      if (invert_rainbow_property)
      {
        value = 1.0f - value;
      }
      getRainbowColor(value, points_out[i].color);
    }
  }
  else
  {
    for (uint32_t i = 0; i < num_points; ++i)
    {
      float val = valueFromCloud<float>(cloud, offset, type, point_step, i);
      float normalized_intensity = (val - min_intensity) / diff_intensity;
      normalized_intensity = std::min(1.0f, std::max(0.0f, normalized_intensity));
      points_out[i].color.r =
          max_color.r * normalized_intensity + min_color.r * (1.0f - normalized_intensity);
      points_out[i].color.g =
          max_color.g * normalized_intensity + min_color.g * (1.0f - normalized_intensity);
      points_out[i].color.b =
          max_color.b * normalized_intensity + min_color.b * (1.0f - normalized_intensity);
    }
  }

  return true;
}

void IntensityPCTransformer::updateChannels(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  V_string channels;
  for (size_t i = 0; i < cloud->fields.size(); ++i) // NOLINT
  {
    channels.push_back(cloud->fields[i].name);
  }
  std::sort(channels.begin(), channels.end());

  if (channels != available_channels_)
  {
    //channel_name_property_->clearOptions();
    for (V_string::const_iterator it = channels.begin(); it != channels.end(); ++it) // NOLINT
    {
      const std::string& channel = *it;
      if (channel.empty())
      {
        continue;
      }
      //channel_name_property_->addOptionStd(channel);
    }
    available_channels_ = channels;
  }
}

uint8_t XYZPCTransformer::supports(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  int32_t xi = findChannelIndex(cloud, "x");
  int32_t yi = findChannelIndex(cloud, "y");
  int32_t zi = findChannelIndex(cloud, "z");

  if (xi == -1 || yi == -1 || zi == -1)
  {
    return Support_None;
  }

  if (cloud->fields[xi].datatype == sensor_msgs::PointField::FLOAT32)
  {
    return Support_XYZ;
  }

  return Support_None;
}

bool XYZPCTransformer::transform(const sensor_msgs::PointCloud2ConstPtr& cloud,
                                 uint32_t mask,
                                 const Ogre::Matrix4& /*transform*/,
                                 V_PointCloudPoint& points_out)
{
  if ((mask & Support_XYZ) == 0u)
  {
    return false;
  }

  int32_t xi = findChannelIndex(cloud, "x");
  int32_t yi = findChannelIndex(cloud, "y");
  int32_t zi = findChannelIndex(cloud, "z");

  const uint32_t xoff = cloud->fields[xi].offset;
  const uint32_t yoff = cloud->fields[yi].offset;
  const uint32_t zoff = cloud->fields[zi].offset;
  const uint32_t point_step = cloud->point_step;
  uint8_t const* point_x = &cloud->data.front() + xoff; // NOLINT
  uint8_t const* point_y = &cloud->data.front() + yoff; // NOLINT
  uint8_t const* point_z = &cloud->data.front() + zoff; // NOLINT
  for (V_PointCloudPoint::iterator iter = points_out.begin(); iter != points_out.end();
       ++iter, point_x += point_step, point_y += point_step, point_z += point_step) // NOLINT
  {
    iter->position.x = *reinterpret_cast<const float*>(point_x); // NOLINT
    iter->position.y = *reinterpret_cast<const float*>(point_y); // NOLINT
    iter->position.z = *reinterpret_cast<const float*>(point_z); // NOLINT
  }

  return true;
}

uint8_t RGB8PCTransformer::supports(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  int32_t index = std::max(findChannelIndex(cloud, "rgb"), findChannelIndex(cloud, "rgba"));
  if (index == -1)
  {
    return Support_None;
  }

  if (cloud->fields[index].datatype == sensor_msgs::PointField::INT32 ||
      cloud->fields[index].datatype == sensor_msgs::PointField::UINT32 ||
      cloud->fields[index].datatype == sensor_msgs::PointField::FLOAT32)
  {
    return Support_Color;
  }

  return Support_None;
}

bool RGB8PCTransformer::transform(const sensor_msgs::PointCloud2ConstPtr& cloud,
                                  uint32_t mask,
                                  const Ogre::Matrix4& /*transform*/,
                                  V_PointCloudPoint& points_out)
{
  if ((mask & Support_Color) == 0u)
  {
    return false;
  }

  const int32_t rgb = findChannelIndex(cloud, "rgb");
  const int32_t rgba = findChannelIndex(cloud, "rgba");
  const int32_t index = std::max(rgb, rgba);

  const uint32_t off = cloud->fields[index].offset;
  uint8_t const* rgb_ptr = &cloud->data.front() + off; // NOLINT
  const uint32_t point_step = cloud->point_step;

  // Create a look-up table for colors
  float rgb_lut[256]; // NOLINT
  for (int i = 0; i < 256; ++i)
  {
    rgb_lut[i] = float(i) / 255.0f; // NOLINT
  }
  if (rgb != -1) // rgb
  {
    for (V_PointCloudPoint::iterator iter = points_out.begin(); iter != points_out.end();
         ++iter, rgb_ptr += point_step) // NOLINT
    {
      uint32_t rgb1 = *reinterpret_cast<const uint32_t*>(rgb_ptr); // NOLINT
      iter->color.r = rgb_lut[(rgb1 >> 16) & 0xff]; // NOLINT
      iter->color.g = rgb_lut[(rgb1 >> 8) & 0xff]; // NOLINT
      iter->color.b = rgb_lut[rgb1 & 0xff]; // NOLINT
      iter->color.a = 1.0f;
    }
  }
  else // rgba
  {
    for (V_PointCloudPoint::iterator iter = points_out.begin(); iter != points_out.end();
         ++iter, rgb_ptr += point_step) // NOLINT
    {
      uint32_t rgb1 = *reinterpret_cast<const uint32_t*>(rgb_ptr); // NOLINT
      iter->color.r = rgb_lut[(rgb1 >> 16) & 0xff]; // NOLINT
      iter->color.g = rgb_lut[(rgb1 >> 8) & 0xff]; // NOLINT
      iter->color.b = rgb_lut[rgb1 & 0xff]; // NOLINT
      iter->color.a = rgb_lut[rgb1 >> 24]; // NOLINT
    }
  }

  return true;
}

bool MONO8PCTransformer::transform(const sensor_msgs::PointCloud2ConstPtr& cloud,
                                   uint32_t mask,
                                   const Ogre::Matrix4& /*transform*/,
                                   V_PointCloudPoint& points_out)
{
  if ((mask & Support_Color) == 0u)
  {
    return false;
  }

  const int32_t rgb = findChannelIndex(cloud, "rgb");
  const int32_t rgba = findChannelIndex(cloud, "rgba");
  const int32_t index = std::max(rgb, rgba);

  const uint32_t off = cloud->fields[index].offset;
  uint8_t const* rgb_ptr = &cloud->data.front() + off; // NOLINT
  const uint32_t point_step = cloud->point_step;

  // Create a look-up table for colors
  float rgb_lut[256]; // NOLINT
  for (int i = 0; i < 256; ++i)
  {
    rgb_lut[i] = float(i) / 255.0f; // NOLINT
  }
  if (rgb != -1) // rgb
  {
    for (V_PointCloudPoint::iterator iter = points_out.begin(); iter != points_out.end();
         ++iter, rgb_ptr += point_step) // NOLINT
    {
      uint32_t rgb1 = *reinterpret_cast<const uint32_t*>(rgb_ptr); // NOLINT
      float r = rgb_lut[(rgb1 >> 16) & 0xff]; // NOLINT
      float g = rgb_lut[(rgb1 >> 8) & 0xff]; // NOLINT
      float b = rgb_lut[rgb1 & 0xff]; // NOLINT
      float mono = 0.2989 * r + 0.5870 * g + 0.1140 * b; // NOLINT
      iter->color.r = iter->color.g = iter->color.b = mono;
      iter->color.a = 1.0f;
    }
  }
  else // rgba
  {
    for (V_PointCloudPoint::iterator iter = points_out.begin(); iter != points_out.end();
         ++iter, rgb_ptr += point_step) // NOLINT
    {
      uint32_t rgb1 = *reinterpret_cast<const uint32_t*>(rgb_ptr); // NOLINT
      float r = rgb_lut[(rgb1 >> 16) & 0xff]; // NOLINT
      float g = rgb_lut[(rgb1 >> 8) & 0xff]; // NOLINT
      float b = rgb_lut[rgb1 & 0xff]; // NOLINT
      float mono = 0.2989 * r + 0.5870 * g + 0.1140 * b; // NOLINT
      iter->color.r = iter->color.g = iter->color.b = mono;
      iter->color.a = rgb_lut[rgb1 >> 24]; // NOLINT
    }
  }

  return true;
}

uint8_t RGBF32PCTransformer::supports(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  int32_t ri = findChannelIndex(cloud, "r");
  int32_t gi = findChannelIndex(cloud, "g");
  int32_t bi = findChannelIndex(cloud, "b");
  if (ri == -1 || gi == -1 || bi == -1)
  {
    return Support_None;
  }

  if (cloud->fields[ri].datatype == sensor_msgs::PointField::FLOAT32)
  {
    return Support_Color;
  }

  return Support_None;
}

bool RGBF32PCTransformer::transform(const sensor_msgs::PointCloud2ConstPtr& cloud,
                                    uint32_t mask,
                                    const Ogre::Matrix4& /*transform*/,
                                    V_PointCloudPoint& points_out)
{
  if ((mask & Support_Color) == 0u)
  {
    return false;
  }

  int32_t ri = findChannelIndex(cloud, "r");
  int32_t gi = findChannelIndex(cloud, "g");
  int32_t bi = findChannelIndex(cloud, "b");

  const uint32_t roff = cloud->fields[ri].offset;
  const uint32_t goff = cloud->fields[gi].offset;
  const uint32_t boff = cloud->fields[bi].offset;
  const uint32_t point_step = cloud->point_step;
  const uint32_t num_points = cloud->width * cloud->height;
  uint8_t const* point = &cloud->data.front();
  for (uint32_t i = 0; i < num_points; ++i, point += point_step) // NOLINT
  {
    float r = *reinterpret_cast<const float*>(point + roff); // NOLINT
    float g = *reinterpret_cast<const float*>(point + goff); // NOLINT
    float b = *reinterpret_cast<const float*>(point + boff); // NOLINT
    points_out[i].color = Ogre::ColourValue(r, g, b);
  }

  return true;
}

uint8_t FlatColorPCTransformer::supports(const sensor_msgs::PointCloud2ConstPtr& /*cloud*/)
{
  return Support_Color;
}

uint8_t FlatColorPCTransformer::score(const sensor_msgs::PointCloud2ConstPtr& /*cloud*/)
{
  return 0;
}

bool FlatColorPCTransformer::transform(const sensor_msgs::PointCloud2ConstPtr& cloud,
                                       uint32_t mask,
                                       const Ogre::Matrix4& /*transform*/,
                                       V_PointCloudPoint& points_out)
{
  if ((mask & Support_Color) == 0u)
  {
    return false;
  }

  Ogre::ColourValue color;

  const uint32_t num_points = cloud->width * cloud->height;
  for (uint32_t i = 0; i < num_points; ++i)
  {
    points_out[i].color = color;
  }

  return true;
}

uint8_t AxisColorPCTransformer::supports(const sensor_msgs::PointCloud2ConstPtr& /*cloud*/)
{
  return Support_Color;
}

uint8_t AxisColorPCTransformer::score(const sensor_msgs::PointCloud2ConstPtr& /*cloud*/)
{
  return 255;
}

bool AxisColorPCTransformer::transform(const sensor_msgs::PointCloud2ConstPtr& cloud,
                                       uint32_t mask,
                                       const Ogre::Matrix4& transform,
                                       V_PointCloudPoint& points_out)
{
  if ((mask & Support_Color) == 0u)
  {
    return false;
  }

  int32_t xi = findChannelIndex(cloud, "x");
  int32_t yi = findChannelIndex(cloud, "y");
  int32_t zi = findChannelIndex(cloud, "z");

  const uint32_t xoff = cloud->fields[xi].offset;
  const uint32_t yoff = cloud->fields[yi].offset;
  const uint32_t zoff = cloud->fields[zi].offset;
  const uint32_t point_step = cloud->point_step;
  const uint32_t num_points = cloud->width * cloud->height;
  uint8_t const* point = &cloud->data.front();

  // Fill a vector of floats with values based on the chosen axis.
  int axis = 0;//axis_property_->getOptionInt();
  std::vector<float> values;
  values.reserve(num_points);
  Ogre::Vector3 pos;
  const bool use_fixed_frame_property = true;
  if (use_fixed_frame_property)
  {
    for (uint32_t i = 0; i < num_points; ++i, point += point_step) // NOLINT
    {
      // TODO: optimize this by only doing the multiplication needed
      // for the desired output value, instead of doing all of them
      // and then throwing most away.
      pos.x = *reinterpret_cast<const float*>(point + xoff); // NOLINT
      pos.y = *reinterpret_cast<const float*>(point + yoff); // NOLINT
      pos.z = *reinterpret_cast<const float*>(point + zoff); // NOLINT

      pos = transform * pos;
      values.push_back(pos[axis]);
    }
  }
  else
  {
    const uint32_t offsets[3] = {xoff, yoff, zoff}; // NOLINT
    const uint32_t off = offsets[axis]; // NOLINT
    for (uint32_t i = 0; i < num_points; ++i, point += point_step) // NOLINT
    {
      values.push_back(*reinterpret_cast<const float*>(point + off)); // NOLINT
    }
  }
  float min_value_current = 9999.0f;
  float max_value_current = -9999.0f;
  const bool auto_compute_bounds_property = true;
  if (auto_compute_bounds_property)
  {
    for (uint32_t i = 0; i < num_points; i++)
    {
      float val = values[i];
      min_value_current = std::min(min_value_current, val);
      max_value_current = std::max(max_value_current, val);
    }
    //min_value_property_->setFloat(min_value_current);
    //max_value_property_->setFloat(max_value_current);
  }
  else
  {
    //min_value_current = min_value_property_->getFloat();
    //max_value_current = max_value_property_->getFloat();
  }

  float range = max_value_current - min_value_current;
  if (range == 0)
  {
    range = 0.001f;
  }
  for (uint32_t i = 0; i < num_points; ++i)
  {
    float value = 1.0f - (values[i] - min_value_current) / range;
    getRainbowColor(value, points_out[i].color);
  }

  return true;
}

} // end namespace rviz

