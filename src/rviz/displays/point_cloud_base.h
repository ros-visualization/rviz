/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef OGRE_VISUALIZER_POINT_CLOUD_BASE_H
#define OGRE_VISUALIZER_POINT_CLOUD_BASE_H

#include "display.h"
#include "helpers/color.h"
#include "ogre_tools/point_cloud.h"

#include "robot_msgs/PointCloud.h"

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <deque>
#include <queue>
#include <vector>

namespace ros
{
  class Node;
}

namespace tf
{
template<class Message> class MessageNotifier;
}

namespace ogre_vis
{

class IntProperty;
class FloatProperty;
class StringProperty;
class ROSTopicStringProperty;
class ColorProperty;
class EnumProperty;
class BoolProperty;

/**
 * \class PointCloudBase
 * \brief Displays a point cloud of type robot_msgs::PointCloud
 *
 * By default it will assume channel 0 of the cloud is an intensity value, and will color them by intensity.
 * If you set the channel's name to "rgb", it will interpret the channel as an integer rgb value, with r, g and b
 * all being 8 bits.
 */
class PointCloudBase : public Display
{
private:
  struct CloudInfo
  {
    CloudInfo();
    ~CloudInfo();

    float time_;

    boost::shared_ptr<robot_msgs::PointCloud> message_;
    uint32_t num_points_;
  };
  typedef boost::shared_ptr<CloudInfo> CloudInfoPtr;
  typedef std::deque<CloudInfoPtr> D_CloudInfo;
  typedef std::queue<CloudInfoPtr> Q_CloudInfo;

public:
  /**
   * \enum Style
   * \brief The different styles of pointcloud drawing
   */
  enum Style
  {
    Points,    ///< Points -- points are drawn as a fixed size in 2d space, ie. always 1 pixel on screen
    Billboards,///< Billboards -- points are drawn as camera-facing quads in 3d space

    StyleCount,
  };

  /**
   * \enum ChannelRender
   * \brief The different channels that we support rendering
   */
  enum ChannelRender
  {
    Intensity,    ///< Intensity data
    Curvature,    ///< Surface curvature estimates
    ColorRGBSpace,///< RGB Color
    NormalSphere, ///< Use the nx-ny-nz (normal coordinates) instead of x-y-z

    ChannelRenderCount,
  };

  PointCloudBase( const std::string& name, VisualizationManager* manager );
  ~PointCloudBase();
  /**
   * Set the primary color of this point cloud.  This color is used verbatim for the highest intensity points, and linearly interpolates
   * down to the min color for the lowest intensity points
   */
  void setMaxColor( const Color& color );
  /**
   * Set the primary color of this point cloud.  This color is used verbatim for the highest intensity points, and linearly interpolates
   * down to the min color for the lowest intensity points
   */
  void setMinColor( const Color& color );
  /**
   * \brief Set the rendering style
   * @param style The rendering style
   */
  void setStyle( int style );
  /**
   * \brief Sets the size each point will be when drawn in 3D as a billboard
   * @note Only applicable if the style is set to Billboards (default)
   * @param size The size
   */
  void setBillboardSize( float size );
  /**
   * \brief Set the amount of time each cloud should stick around for
   * @param time Decay time, in seconds
   */
  void setDecayTime( float time );

  void setMinIntensity(float val);
  void setMaxIntensity(float val);
  float getMinIntensity() { return min_intensity_; }
  float getMaxIntensity() { return max_intensity_; }
  void setAutoComputeIntensityBounds(bool compute);
  bool getAutoComputeIntensityBounds() { return auto_compute_intensity_bounds_; }

  float getBillboardSize() { return billboard_size_; }
  const Color& getMaxColor() { return max_color_; }
  const Color& getMinColor() { return min_color_; }
  int getStyle() { return style_; }
  float getDecayTime() { return point_decay_time_; }

  /**
   * \brief Set the channel index to be rendered as color
   * @param channel_color_idx the index of the channel
   */
  void setChannelColorIndex (int channel_color_idx);
  int getChannelColorIndex () { return (channel_color_idx_); }

  /** \brief Get the index of a specified dimension/channel in a point cloud
    * \param points the point cloud
    * \param channel_name the string defining the channel name
    */
  inline int getROSCloudChannelIndex (const boost::shared_ptr<robot_msgs::PointCloud>& points, std::string channel_name)
  {
    for (unsigned int d = 0; d < points->chan.size (); d++)
      if (points->chan[d].name == channel_name)
        return (d);
    return (-1);
  }

  float getAlpha() { return alpha_; }
  void setAlpha( float alpha );

  // Overrides from Display
  virtual void fixedFrameChanged();
  virtual void createProperties();
  virtual void reset();
  virtual void update(float dt);

protected:
  virtual void onEnable();
  virtual void onDisable();

  /**
   * \brief Transforms the cloud into the correct frame, and sets up our renderable cloud
   */
  void transformCloud(const CloudInfoPtr& cloud);
  void transformThreadFunc();

  void processMessage(const boost::shared_ptr<robot_msgs::PointCloud>& cloud);
  void addMessage(const boost::shared_ptr<robot_msgs::PointCloud>& cloud);

  D_CloudInfo clouds_;
  boost::mutex clouds_mutex_;
  bool new_cloud_;

  ogre_tools::PointCloud* cloud_;

  float alpha_;
  Color min_color_;
  Color max_color_;
  float min_intensity_;
  float max_intensity_;
  bool auto_compute_intensity_bounds_;
  bool intensity_bounds_changed_;

  int style_;                                 ///< Our rendering style
  int channel_color_idx_;                     ///< Which channel to render as color
  float billboard_size_;                      ///< Size to draw our billboards
  float point_decay_time_;                    ///< How long clouds should stick around for before they are culled

  FloatProperty* billboard_size_property_;
  FloatProperty* alpha_property_;
  ColorProperty* min_color_property_;
  ColorProperty* max_color_property_;
  BoolProperty* auto_compute_intensity_bounds_property_;
  FloatProperty* min_intensity_property_;
  FloatProperty* max_intensity_property_;
  EnumProperty* style_property_;
  EnumProperty* channel_property_;
  FloatProperty* decay_time_property_;
};

} // namespace ogre_vis

#endif // OGRE_VISUALIZER_POINT_CLOUD_BASE_H
