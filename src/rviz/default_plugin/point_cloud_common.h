/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#ifndef RVIZ_POINT_CLOUD_COMMON_H
#define RVIZ_POINT_CLOUD_COMMON_H

#ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <deque>
#include <queue>
#include <vector>

#include <QObject>
#include <QList>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include <ros/spinner.h>
#include <ros/callback_queue.h>

#include <message_filters/time_sequencer.h>

#include <pluginlib/class_loader.hpp>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include "rviz/selection/selection_manager.h"
#include "rviz/default_plugin/point_cloud_transformer.h"
#include "rviz/properties/color_property.h"
#include "rviz/ogre_helpers/point_cloud.h"
#include "rviz/selection/forwards.h"
#endif

namespace rviz
{
class BoolProperty;
class Display;
class DisplayContext;
class EnumProperty;
class FloatProperty;
struct IndexAndMessage;
class PointCloudSelectionHandler;
typedef boost::shared_ptr<PointCloudSelectionHandler> PointCloudSelectionHandlerPtr;
class PointCloudTransformer;
typedef boost::shared_ptr<PointCloudTransformer> PointCloudTransformerPtr;

typedef std::vector<std::string> V_string;

/**
 * \class PointCloudCommon
 * \brief Displays a point cloud of type sensor_msgs::PointCloud
 *
 * By default it will assume channel 0 of the cloud is an intensity value, and will color them by
 * intensity.
 * If you set the channel's name to "rgb", it will interpret the channel as an integer rgb value, with r,
 * g and b
 * all being 8 bits.
 */
class PointCloudCommon : public QObject
{
  Q_OBJECT
public:
  struct CloudInfo
  {
    CloudInfo();
    ~CloudInfo();

    // clear the point cloud, but keep selection handler around
    void clear();

    ros::Time receive_time_;

    Ogre::SceneManager* manager_;

    sensor_msgs::PointCloud2ConstPtr message_;

    Ogre::SceneNode* scene_node_;
    boost::shared_ptr<PointCloud> cloud_;
    PointCloudSelectionHandlerPtr selection_handler_;

    std::vector<PointCloud::Point> transformed_points_;

    Ogre::Quaternion orientation_;
    Ogre::Vector3 position_;
  };

  typedef boost::shared_ptr<CloudInfo> CloudInfoPtr;
  typedef std::deque<CloudInfoPtr> D_CloudInfo;
  typedef std::vector<CloudInfoPtr> V_CloudInfo;
  typedef std::queue<CloudInfoPtr> Q_CloudInfo;
  typedef std::list<CloudInfoPtr> L_CloudInfo;

  PointCloudCommon(Display* display);
  ~PointCloudCommon() override;

  void initialize(DisplayContext* context, Ogre::SceneNode* scene_node);

  void fixedFrameChanged();
  void reset();
  void update(float wall_dt, float ros_dt);

  void addMessage(const sensor_msgs::PointCloudConstPtr& cloud);
  void addMessage(const sensor_msgs::PointCloud2ConstPtr& cloud);

  ros::CallbackQueueInterface* getCallbackQueue()
  {
    return &cbqueue_;
  }

  Display* getDisplay()
  {
    return display_;
  }

  bool auto_size_;

  BoolProperty* selectable_property_;
  FloatProperty* point_world_size_property_;
  FloatProperty* point_pixel_size_property_;
  FloatProperty* alpha_property_;
  EnumProperty* xyz_transformer_property_;
  EnumProperty* color_transformer_property_;
  EnumProperty* style_property_;
  FloatProperty* decay_time_property_;

  void setAutoSize(bool auto_size);

public Q_SLOTS:
  void causeRetransform();

private Q_SLOTS:
  void updateSelectable();
  void updateStyle();
  void updateBillboardSize();
  void updateAlpha();
  void updateXyzTransformer();
  void updateColorTransformer();
  void setXyzTransformerOptions(EnumProperty* prop);
  void setColorTransformerOptions(EnumProperty* prop);

private:
  /**
   * \brief Transforms the cloud into the correct frame, and sets up our renderable cloud
   */
  bool transformCloud(const CloudInfoPtr& cloud, bool fully_update_transformers);

  void processMessage(const sensor_msgs::PointCloud2ConstPtr& cloud);
  void updateStatus();

  PointCloudTransformerPtr getXYZTransformer(const sensor_msgs::PointCloud2ConstPtr& cloud);
  PointCloudTransformerPtr getColorTransformer(const sensor_msgs::PointCloud2ConstPtr& cloud);
  void updateTransformers(const sensor_msgs::PointCloud2ConstPtr& cloud);
  void retransform();
  void onTransformerOptions(V_string& ops, uint32_t mask);

  void loadTransformers();

  float getSelectionBoxSize();
  void setPropertiesHidden(const QList<Property*>& props, bool hide);
  void fillTransformerOptions(EnumProperty* prop, uint32_t mask);

  ros::AsyncSpinner spinner_;
  ros::CallbackQueue cbqueue_;

  D_CloudInfo cloud_infos_;

  Ogre::SceneNode* scene_node_;

  V_CloudInfo new_cloud_infos_;
  boost::mutex new_clouds_mutex_;

  L_CloudInfo obsolete_cloud_infos_;

  struct TransformerInfo
  {
    PointCloudTransformerPtr transformer;
    QList<Property*> xyz_props;
    QList<Property*> color_props;

    std::string readable_name;
    std::string lookup_name;
  };
  typedef std::map<std::string, TransformerInfo> M_TransformerInfo;

  boost::recursive_mutex transformers_mutex_;
  M_TransformerInfo transformers_;
  bool new_xyz_transformer_;
  bool new_color_transformer_;
  bool needs_retransform_;

  pluginlib::ClassLoader<PointCloudTransformer>* transformer_class_loader_;

  Display* display_;
  DisplayContext* context_;

  friend class PointCloudSelectionHandler;
};

class PointCloudSelectionHandler : public SelectionHandler
{
public:
  PointCloudSelectionHandler(float box_size,
                             PointCloudCommon::CloudInfo* cloud_info,
                             DisplayContext* context);
  ~PointCloudSelectionHandler() override;

  void createProperties(const Picked& obj, Property* parent_property) override;
  void destroyProperties(const Picked& obj, Property* parent_property) override;

  bool needsAdditionalRenderPass(uint32_t pass) override
  {
    return pass < 2;
  }

  void preRenderPass(uint32_t pass) override;
  void postRenderPass(uint32_t pass) override;

  void onSelect(const Picked& obj) override;
  void onDeselect(const Picked& obj) override;

  void getAABBs(const Picked& obj, V_AABB& aabbs) override;

  void setBoxSize(float size)
  {
    box_size_ = size;
  }

  bool hasSelections()
  {
    return !boxes_.empty();
  }

private:
  PointCloudCommon::CloudInfo* cloud_info_;
  QHash<IndexAndMessage, Property*> property_hash_;
  float box_size_;
};

} // namespace rviz

#endif // RVIZ_POINT_CLOUD_COMMON_H
