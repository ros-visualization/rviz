#include "rviz/image/mouse_click.h"

#include <ros/names.h>


namespace rviz
{

MouseClick::MouseClick() : QObject()
{
}

MouseClick::~MouseClick()
{
}


void MouseClick::onInitialize(QObject* parent)
{
  setParent(parent);
  parent->installEventFilter(this);

  has_dimensions_ = false;
  is_topic_name_ok_ = false;
  node_handle_.reset(new ros::NodeHandle());
}

void MouseClick::publish()
{
  if (is_topic_name_ok_)
  {
    publisher_.reset(
        new ros::Publisher(node_handle_->advertise<geometry_msgs::PointStamped>(topic_, 1000)));
  }
}

void MouseClick::unpublish()
{
  publisher_.reset();
}


bool MouseClick::eventFilter(QObject* obj, QEvent* event)
{
  if (has_dimensions_ == false)
    return false; // Cannot compute pixel coordinates.

  if (is_topic_name_ok_ == false)
    return false; // Cannot publish.


  if (event->type() == QEvent::MouseButtonPress)
  {
    QMouseEvent* me = static_cast<QMouseEvent*>(event);
    QPointF windowPos = me->windowPos();

    if (img_width_ != 0 && img_height_ != 0 && win_width_ != 0 && win_height_ != 0)
    {
      float img_aspect = float(img_width_) / float(img_height_);
      float win_aspect = float(win_width_) / float(win_height_);

      int pix_x = -1;
      int pix_y = -1;
      if (img_aspect > win_aspect) // Window is taller than the image: black bars over and under image.
      {
        pix_x = int(float(windowPos.x()) / float(win_width_) * float(img_width_));

        int resized_img_height = int(float(win_width_) / float(img_width_) * float(img_height_));
        int bias = int((float(win_height_) - float(resized_img_height)) / 2.0);
        pix_y = (float(windowPos.y()) - bias) / float(resized_img_height) * float(img_height_);
      }
      else // Window wider than the image: black bards on the side.
      {
        pix_y = int(float(windowPos.y()) / float(win_height_) * float(img_height_));

        int resized_img_width = int(float(win_height_) / float(img_height_) * float(img_width_));
        int bias = int((float(win_width_) - float(resized_img_width)) / 2.0);
        pix_x = (float(windowPos.x()) - bias) / float(resized_img_width) * float(img_width_);
      }

      // Publish if clicked point is inside the image.
      if (pix_x > 0 && pix_x < img_width_ && pix_y > 0 && pix_y < img_height_)
      {
        geometry_msgs::PointStamped point_msgs;
        point_msgs.header.stamp = ros::Time::now();
        point_msgs.point.x = pix_x;
        point_msgs.point.y = pix_y;
        publisher_->publish(point_msgs);
      }
    }
  }
  return QObject::eventFilter(obj, event);
}

void MouseClick::setDimensions(int img_width, int img_height, int win_width, int win_height)
{
  img_width_ = img_width;
  img_height_ = img_height;
  win_width_ = win_width;
  win_height_ = win_height;
  has_dimensions_ = true;
}

void MouseClick::setTopic(const QString& image_topic)
{
  // Build the click full topic name based on the image topic.
  topic_ = image_topic.toStdString().append("/mouse_click");

  std::string error;
  if (ros::names::validate((const std::string)topic_, error))
  {
    is_topic_name_ok_ = true;
  }
  else
  {
    is_topic_name_ok_ = false;
  }
}

void MouseClick::updateTopic(const QString& image_topic)
{
  unpublish();
  setTopic(image_topic);
  publish();
}

} // namespace rviz
