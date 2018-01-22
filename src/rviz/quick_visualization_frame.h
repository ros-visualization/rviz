#ifndef QUICK_VISUALIZATION_FRAME_H
#define QUICK_VISUALIZATION_FRAME_H

#include <QQuickItem>
#include <QQmlParserStatus>

#include "rviz/config.h"
#include "rviz/panel.h"

namespace rviz {

class RenderPanel;

class QuickVisualizationFrame : public QQuickItem
{
  Q_OBJECT
  Q_INTERFACES(QQmlParserStatus)
  Q_PROPERTY(QString getStatusText READ getStatusText NOTIFY statusTextChanged)

public:
  explicit QuickVisualizationFrame(QQuickItem* parent = Q_NULLPTR);

  ~QuickVisualizationFrame();

  /** @brief Initialize the visualizer.  Creates the VisualizationManager.
   *
   * This function must be called before load(), save(), getManager(),
   * or addPanelByName(), since it creates the VisualizationManager
   * instance which those calls depend on.
   *
   * This function also calls VisualizationManager::initialize(),
   * which means it will start the update timer and generally get
   * things rolling. */
  Q_INVOKABLE void initialize( const QString& display_config_file = "" );

  QString getStatusText() const;

  void componentComplete();

public Q_SLOTS:
  void reset();
  void showMessage(const QString &message);

Q_SIGNALS:
  void statusTextChanged(const QString &status_text);

private Q_SLOTS:
  void onOgreInitialized();
  void updateFps();

private:
  RenderPanel* render_panel_;
  VisualizationManager* manager_;

  bool initialized_;

  QString status_text_;
};

} // namespace rviz

#endif // QUICK_VISUALIZATION_FRAME_H
