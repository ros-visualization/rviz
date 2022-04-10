#include <image_display_with_click/mouse_watcher.h>

using namespace std;

namespace atom_rviz
{

MouseWatcher::MouseWatcher(QWidget * parent) : QObject(parent) {
        has_dimensions = false;
        node_handle = new ros::NodeHandle();
        mouse_event_pub = new ros::Publisher();
    }


bool MouseWatcher::eventFilter(QObject * obj, QEvent * event)
    {
        if (event->type() == QEvent::MouseButtonPress)
        {
            QMouseEvent * me = static_cast<QMouseEvent *>(event);
            QPointF windowPos = me->windowPos();
            printf("User clicked mouse at %f,%f\n", windowPos.x(), windowPos.y());
            cout << "img_width = " << img_width << endl;
            cout << "img_height = " << img_height << endl;
            cout << "win_width = " << win_width << endl;
            cout << "win_height = " << win_height << endl;

            if (img_width != 0 && img_height != 0 && win_width != 0 && win_height != 0)
            {
                float img_aspect = float(img_width) / float(img_height);
                float win_aspect = float(win_width) / float(win_height);
                cout << "img_aspect = " << img_aspect << endl;
                cout << "win_aspect = " << win_aspect << endl;

                int pix_x=-1;
                int pix_y=-1;
                if (img_aspect > win_aspect) // case where the window is taller than the image
                {
                    cout << "black bars on top!" << endl;
                    pix_x = int(float(windowPos.x()) / float(win_width) * float(img_width));

                    int resized_img_height = int(float(win_width) / float(img_width) * float(img_height));
                    int bias = int( ( float(win_height) - float(resized_img_height) )/2.0);
                    pix_y = (float(windowPos.y()) - bias) / float(resized_img_height) * float(img_height);
                    cout << "pix_x = " << pix_x << endl;
                    cout << "resized_img_height = " << resized_img_height << endl;
                    cout << "bias = " << bias << endl;
                    cout << "pix_y = " << pix_y << endl;
                }
                else // case where the window is wider than the image
                {
                    cout << "black bars on side!" << endl;
                    pix_y = int(float(windowPos.y()) / float(win_height) * float(img_height));

                    int resized_img_width = int(float(win_height) / float(img_height) * float(img_width));
                    int bias = int( ( float(win_width) - float(resized_img_width) )/2.0);
                    pix_x = (float(windowPos.x()) - bias) / float(resized_img_width) * float(img_width);
                    cout << "pix_x = " << pix_x << endl;
                    cout << "resized_img_width = " << resized_img_width << endl;
                    cout << "bias = " << bias << endl;
                    cout << "pix_y = " << pix_y << endl;
                }

                // Check if clicked point is inside the image, publish if so
                if (pix_x > 0 && pix_x < img_width && pix_y > 0 && pix_y < img_height)
                {
                    cout << "You clicked inside the image. I should publish ..." << endl;
                    geometry_msgs::PointStamped point_msgs;
                    point_msgs.header.stamp = ros::Time::now();
                    point_msgs.point.x = pix_x;
                    point_msgs.point.y = pix_y;
                    mouse_event_pub->publish(point_msgs);
                }
            }
        }
        return QObject::eventFilter(obj, event);
    }

    void MouseWatcher::setDimensions(int _img_width, int _img_height, int _win_width, int _win_height)
    {
        img_width = _img_width;
        img_height = _img_height;
        win_width = _win_width;
        win_height = _win_height;
        has_dimensions = true;
    }

    void MouseWatcher::setTopic(string image_topic)
    {
        string tmp_topic = image_topic + "/click"; // build the click full topic name based on the image topic

        if (tmp_topic.compare(image_click_topic) != 0) // if topic changed, reconfigure the publisher
        {
            image_click_topic = tmp_topic; // set new topic
            cout << "Changed pub topic, reconfiguring to publish on topic " << image_click_topic << endl;

            mouse_event_pub->shutdown(); // shutdown current publisher
            delete mouse_event_pub; // delete the mem allocation (to avoid mem leaks)
            // reconfigure new publisher
            mouse_event_pub = new ros::Publisher(node_handle->advertise<geometry_msgs::PointStamped>(image_click_topic, 1000));
        }
    }

} // atom_rviz
