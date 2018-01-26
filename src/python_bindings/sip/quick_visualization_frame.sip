namespace rviz {

class QuickVisualizationFrame: QQuickItem /NoDefaultCtors/
{
%TypeHeaderCode
#define ROS_PACKAGE_NAME "rviz"
#include <rviz/quick_visualization_frame.h>
%End

public:
  explicit QuickVisualizationFrame(QQuickItem* parent = Q_NULLPTR);
  virtual ~QuickVisualizationFrame();

  rviz::VisualizationManager* getManager();
  QString getStatusText() const;

  bool isInitialized() const;

  static void registerTypes();
};

};
