#ifndef QUICK_VISUALIZATION_FRAME_H
#define QUICK_VISUALIZATION_FRAME_H

#include <QQuickItem>
#include <QQmlParserStatus>

#include "rviz/config.h"
#include "rviz/panel.h"
#include "rviz/ogre_helpers/qt_quick_ogre_render_window.h"

namespace rviz {

class RenderPanel;

class QuickVisualizationFrame : public QQuickItem
{
  Q_OBJECT
  Q_INTERFACES(QQmlParserStatus)
  Q_PROPERTY(QString statusText READ getStatusText NOTIFY statusTextChanged)
  Q_PROPERTY(QtQuickOgreRenderWindow* renderWindow READ getRenderWindow WRITE setRenderWindow NOTIFY renderWindowChanged)

public:
  explicit QuickVisualizationFrame(QQuickItem* parent = Q_NULLPTR);

  ~QuickVisualizationFrame();

  void componentComplete();

  /** @brief Initialize the visualizer.  Creates the VisualizationManager.
   *
   * This function must be called before load(), save(), getManager(),
   * or addPanelByName(), since it creates the VisualizationManager
   * instance which those calls depend on.
   *
   * This function also calls VisualizationManager::initialize(),
   * which means it will start the update timer and generally get
   * things rolling. */
  Q_INVOKABLE void initialize(QtQuickOgreRenderWindow *render_window);

  QString getStatusText() const;

  QtQuickOgreRenderWindow* getRenderWindow() const;

public Q_SLOTS:
  void reset();
  void showMessage(const QString &message);

  void setRenderWindow(QtQuickOgreRenderWindow* renderWindow);

Q_SIGNALS:
  void statusTextChanged(const QString &status_text);
  void renderWindowChanged(QtQuickOgreRenderWindow* render_window);

private Q_SLOTS:
  void onOgreInitializing();
  void onOgreInitialized();
  void updateFps();

private:
  RenderPanel* render_panel_;
  QtQuickOgreRenderWindow* render_window_;
  VisualizationManager* manager_;

  bool initialized_;

  QString status_text_;
};

} // namespace rviz

#endif // QUICK_VISUALIZATION_FRAME_H
