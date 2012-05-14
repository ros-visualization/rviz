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

#ifndef RVIZ_POINT_CLOUD_BASE_H
#define RVIZ_POINT_CLOUD_BASE_H

#include <pluginlib/class_loader.h>

#include "point_cloud_transformer.h"

#include "rviz/display.h"
#include "rviz/helpers/color.h"
#include "rviz/properties/forwards.h"
#include "rviz/selection/forwards.h"

#include "rviz/ogre_helpers/point_cloud.h"

#include <message_filters/time_sequencer.h>

#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"

#include <ros/spinner.h>
#include <ros/callback_queue.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include <deque>
#include <queue>
#include <vector>

namespace rviz
{

class PointCloudSelectionHandler;
typedef boost::shared_ptr<PointCloudSelectionHandler> PointCloudSelectionHandlerPtr;
class PointCloudTransformer;
typedef boost::shared_ptr<PointCloudTransformer> PointCloudTransformerPtr;

/**
 * \class PointCloudBase
 * \brief Displays a point cloud of type sensor_msgs::PointCloud
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

    Ogre::Matrix4 transform_;
    sensor_msgs::PointCloud2ConstPtr message_;
    uint32_t num_points_;

    V_PointCloudPoint transformed_points_;
  };
  typedef boost::shared_ptr<CloudInfo> CloudInfoPtr;
  typedef std::deque<CloudInfoPtr> D_CloudInfo;
  typedef std::vector<CloudInfoPtr> V_CloudInfo;
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
    BillboardSpheres, ///< Billboard "spheres" -- cam-facing tris with a pixel shader that causes them to look like spheres
    Boxes, ///< Boxes -- Actual 3d cube geometry

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

  PointCloudBase();
  ~PointCloudBase();

  void onInitialize();

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

  float getBillboardSize() { return billboard_size_; }
  int getStyle() { return style_; }
  float getDecayTime() { return point_decay_time_; }

  float getAlpha() { return alpha_; }
  void setAlpha( float alpha );

  bool getSelectable() { return selectable_; }
  void setSelectable(bool selectable);

  // Overrides from Display
  virtual void fixedFrameChanged();
  virtual void createProperties();
  virtual void reset();
  virtual void update(float wall_dt, float ros_dt);

  void causeRetransform();

  /** @brief Hides all visible parts of this display, so they do not show up when the scene is rendered. */
  virtual void hideVisible();

  /** @brief Restores the display to the state it was in before hideVisible() was called. */
  virtual void restoreVisible();

protected:
  virtual void onEnable();
  virtual void onDisable();

  typedef std::vector<PointCloud::Point> V_Point;
  typedef std::vector<V_Point> VV_Point;

  /**
   * \brief Transforms the cloud into the correct frame, and sets up our renderable cloud
   */
  bool transformCloud(const CloudInfoPtr& cloud, V_Point& points, bool fully_update_transformers);

  void processMessage(const sensor_msgs::PointCloud2ConstPtr& cloud);
  void addMessage(const sensor_msgs::PointCloudConstPtr& cloud);
  void addMessage(const sensor_msgs::PointCloud2ConstPtr& cloud);
  void updateStatus();

  void setXYZTransformer(const std::string& name);
  void setColorTransformer(const std::string& name);
  const std::string& getXYZTransformer() { return xyz_transformer_; }
  const std::string& getColorTransformer() { return color_transformer_; }
  PointCloudTransformerPtr getXYZTransformer(const sensor_msgs::PointCloud2ConstPtr& cloud);
  PointCloudTransformerPtr getColorTransformer(const sensor_msgs::PointCloud2ConstPtr& cloud);
  void updateTransformers(const sensor_msgs::PointCloud2ConstPtr& cloud, bool fully_update);
  void retransform();
  void onTransformerOptions(V_string& ops, uint32_t mask);

  void loadTransformers();

  ros::AsyncSpinner spinner_;
  ros::CallbackQueue cbqueue_;

  D_CloudInfo clouds_;
  boost::mutex clouds_mutex_;
  bool new_cloud_;

  PointCloud* cloud_;
  Ogre::SceneNode* scene_node_;

  VV_Point new_points_;
  V_CloudInfo new_clouds_;
  boost::mutex new_clouds_mutex_;

  float alpha_;

  struct TransformerInfo
  {
    PointCloudTransformerPtr transformer;
    V_PropertyBaseWPtr xyz_props;
    V_PropertyBaseWPtr color_props;

    std::string readable_name;
    std::string lookup_name;
  };
  typedef std::map<std::string, TransformerInfo> M_TransformerInfo;

  boost::recursive_mutex transformers_mutex_;
  M_TransformerInfo transformers_;
  std::string xyz_transformer_;
  std::string color_transformer_;
  bool new_xyz_transformer_;
  bool new_color_transformer_;
  bool needs_retransform_;

  int style_;                                 ///< Our rendering style
  float billboard_size_;                      ///< Size to draw our billboards
  float point_decay_time_;                    ///< How long clouds should stick around for before they are culled

  bool selectable_;
  CollObjectHandle coll_handle_;
  PointCloudSelectionHandlerPtr coll_handler_;

  uint32_t messages_received_;
  uint32_t total_point_count_;

  pluginlib::ClassLoader<PointCloudTransformer>* transformer_class_loader_;

  BoolPropertyWPtr selectable_property_;
  FloatPropertyWPtr billboard_size_property_;
  FloatPropertyWPtr alpha_property_;
  EditEnumPropertyWPtr xyz_transformer_property_;
  EditEnumPropertyWPtr color_transformer_property_;
  EnumPropertyWPtr style_property_;
  FloatPropertyWPtr decay_time_property_;

  bool hidden_;

  friend class PointCloudSelectionHandler;
};

} // namespace rviz

#endif // RVIZ_POINT_CLOUD_BASE_H
