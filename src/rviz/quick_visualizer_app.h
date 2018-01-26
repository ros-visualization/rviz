#ifndef QUICK_VISUALIZER_APP_H
#define QUICK_VISUALIZER_APP_H

#include <QObject>
#include <QTimer>

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <ros/ros.h>
#endif

namespace rviz
{

class QuickVisualizerApp : public QObject
{
    Q_OBJECT
public:
    explicit QuickVisualizerApp(QObject *parent = nullptr);

    /** Start everything.  Pass in command line arguments.
     * @return false on failure, true on success. */
    bool init( int argc, char** argv );

private:
  void startContinueChecker();

  /** If ros::ok() is false, close all windows. */
  void checkContinue();

  QTimer continue_timer_;
  ros::NodeHandlePtr nh_;

};

} // end namespace rviz


#endif // QUICK_VISUALIZER_APP_H
