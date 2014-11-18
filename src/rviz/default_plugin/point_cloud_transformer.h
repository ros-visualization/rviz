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

#ifndef RVIZ_POINT_CLOUD_TRANSFORMER_H
#define RVIZ_POINT_CLOUD_TRANSFORMER_H

#include <QObject>

#include <ros/message_forward.h>

#ifndef Q_MOC_RUN
#include <OgreVector3.h>
#include <OgreColourValue.h>

#include <rviz/ogre_helpers/point_cloud.h>
#endif

namespace Ogre
{
class Matrix4;
}

namespace sensor_msgs
{
ROS_DECLARE_MESSAGE(PointCloud2);
}

namespace rviz
{
class Property;

typedef std::vector<PointCloud::Point> V_PointCloudPoint;

class PointCloudTransformer: public QObject
{
Q_OBJECT
public:
  virtual void init() {}

  /**
   * \brief Enumeration of support levels.  Basic levels (Support_None, Support_XYZ, Support_Color) can be
   * ored together to form a mask, Support_Both is provided as a convenience.
   */
  enum SupportLevel
  {
    Support_None = 0,
    Support_XYZ = 1 << 1,
    Support_Color = 1 << 2,
    Support_Both = Support_XYZ|Support_Color,
  };

  /**
   * \brief Returns a level of support for a specific cloud.  This level of support is a mask using the SupportLevel enum.
   */
  virtual uint8_t supports(const sensor_msgs::PointCloud2ConstPtr& cloud) = 0;
  /**
   * \brief Transforms a PointCloud2 into an rviz::PointCloud.  The rviz::PointCloud is assumed to have been preallocated into the correct
   * size.  The mask determines which part of the cloud should be output (xyz or color).  This method will only be called if supports() of the same
   * cloud has returned a non-zero mask, and will only be called with masks compatible with the one returned from supports()
   */
  virtual bool transform(const sensor_msgs::PointCloud2ConstPtr& cloud, uint32_t mask, const Ogre::Matrix4& transform, V_PointCloudPoint& out) = 0;

  /**
   * \brief "Score" a message for how well supported the message is.  For example, a "flat color" transformer can support any cloud, but will
   * return a score of 0 here since it should not be preferred over others that explicitly support fields in the message.  This allows that
   * "flat color" transformer to still be selectable, but generally not chosen automatically.
   */
  virtual uint8_t score(const sensor_msgs::PointCloud2ConstPtr& cloud) { return 0; }

  /**
   * \brief Create any properties necessary for this transformer.
   * Will be called once when the transformer is loaded.  All
   * properties must be added to the out_props vector.
   */
  virtual void createProperties( Property* parent_property,
                                 uint32_t mask,
                                 QList<Property*>& out_props ) {}

Q_SIGNALS:
  /** @brief Subclasses should emit this signal whenever they think the points should be re-transformed. */
  void needRetransform();
};

} // namespace rviz

#endif
