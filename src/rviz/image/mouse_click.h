#ifndef RVIZ_MOUSE_CLICK_H
#define RVIZ_MOUSE_CLICK_H

#include <QObject>

#ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <iostream>
#include <string>
#include <boost/shared_ptr.hpp>

#include <QMouseEvent>

#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/String.h"

#include "rviz/rviz_export.h"
#include "rviz/display.h"
#endif


namespace rviz
{
/** @brief Class for capturing mouse clicks.
 *
 * This class handles mouse clicking functionalities integrated into the ImageDisplay.
 * It uses a qt event filter to capture mouse clicks, handles image resizing and checks if the click was
 * inside or outside the image. It also scales the pixel coordinates to get them w.r.t. the image (not
 * the window) size. Mouse clicks image pixel coordinates are published as ros geometry_msgs
 * PointStamped. The z component is ignored. The topic name where the mouse clicks are published is
 * defined created after the subscribed image topic as: <image_topic>/mouse_click.
 */

class RVIZ_EXPORT MouseClick : QObject
{
public:
  MouseClick(QObject* parent, const ros::NodeHandle& nh);
  ~MouseClick();

  void onInitialize();

  /** @brief ROS topic management. */
  void publish();
  void unpublish();

  void setDimensions(int img_width, int img_height, int win_width, int win_height);
  void setTopic(const QString& image_topic);
  void updateTopic(const QString& image_topic);

private:
  virtual bool eventFilter(QObject* obj, QEvent* event);

  bool has_dimensions_;
  int img_width_, img_height_, win_width_, win_height_; // To assess if the clicks are inside the image.
  boost::shared_ptr<ros::NodeHandle> node_handle_;
  boost::shared_ptr<ros::Publisher> publisher_;
  std::string topic_;
  bool is_topic_name_ok_;
};

} // namespace rviz
#endif
