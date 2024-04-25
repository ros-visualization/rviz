#include "rviz/image/mouse_click.h"
#include <QWidget>
#include <ros/names.h>


namespace rviz
{
MouseClick::MouseClick(QWidget* widget, const ros::NodeHandle& nh) : QObject(widget)
{
  img_width_ = img_height_ = win_width_ = win_height_ = 0;
  node_handle_ = nh;
  topic_name_ok_ = false;
}

void MouseClick::enable()
{
  if (topic_name_ok_)
  {
    publisher_ = node_handle_.advertise<geometry_msgs::PointStamped>(topic_, 1);
    parent()->installEventFilter(this);
  }
}

void MouseClick::disable()
{
  parent()->removeEventFilter(this);
  publisher_.shutdown();
}

bool MouseClick::eventFilter(QObject* obj, QEvent* event)
{
  if (event->type() == QEvent::MouseButtonPress || event->type() == QEvent::MouseMove)
  {
    QMouseEvent* me = static_cast<QMouseEvent*>(event);
    QPointF windowPos = me->windowPos();
    bool left_button = me->buttons() == Qt::LeftButton;

    if (left_button && img_width_ != 0 && img_height_ != 0 && win_width_ != 0 && win_height_ != 0)
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
      else // Window wider than the image: black bars on the side.
      {
        pix_y = int(float(windowPos.y()) / float(win_height_) * float(img_height_));

        int resized_img_width = int(float(win_height_) / float(img_height_) * float(img_width_));
        int bias = int((float(win_width_) - float(resized_img_width)) / 2.0);
        pix_x = (float(windowPos.x()) - bias) / float(resized_img_width) * float(img_width_);
      }

      // Publish if clicked point is inside the image.
      if (pix_x >= 0 && pix_x < img_width_ && pix_y >= 0 && pix_y < img_height_)
      {
        geometry_msgs::PointStamped point_msg;
        point_msg.header.stamp = ros::Time::now();
        point_msg.point.x = pix_x;
        point_msg.point.y = pix_y;
        publisher_.publish(point_msg);
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
}

void MouseClick::setImageTopic(const QString& topic)
{
  disable();

  // Build the click full topic name based on the image topic
  topic_ = topic.toStdString().append("/mouse_click");

  std::string error;
  topic_name_ok_ = ros::names::validate(topic_, error);

  enable();
}

} // namespace rviz
