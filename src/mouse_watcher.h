#ifndef MOUSE_WATCHER_H
#define MOUSE_WATCHER_H

#ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include "../../../../../../../usr/include/x86_64-linux-gnu/qt5/QtCore/QObject"
#include "../../../../../../../usr/include/OGRE/OgreMaterial.h"
#include "../../../../../../../usr/include/OGRE/OgreRenderTargetListener.h"
#include "../../../../../../../usr/include/OGRE/OgreSharedPtr.h"
#include "../../../../../../../opt/ros/noetic/include/rviz/image/image_display_base.h"
#include "../../../../../../../opt/ros/noetic/include/rviz/image/ros_image_texture.h"
#include "../../../../../../../opt/ros/noetic/include/rviz/render_panel.h"
#include "../../../../../../../opt/ros/noetic/include/rviz/properties/bool_property.h"
#include "../../../../../../../opt/ros/noetic/include/rviz/properties/float_property.h"
#include "../../../../../../../opt/ros/noetic/include/rviz/properties/int_property.h"
#endif

#include <iostream>
#include <string>

#include <QMouseEvent>

#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/String.h"

using namespace std;

namespace atom_rviz
{
class MouseWatcher : public QObject
{
public:
    MouseWatcher(QWidget * parent);
    virtual bool eventFilter(QObject * obj, QEvent * event);
    void setDimensions(int _img_width, int _img_height, int _win_width, int _win_height);
    void setTopic(string image_topic);

private:
    string image_click_topic;
    bool has_dimensions;
    int img_width;
    int img_height;
    int win_width;
    int win_height;
    ros::Publisher* mouse_event_pub;
    ros::NodeHandle* node_handle;
};

} // namespace rviz
#endif
