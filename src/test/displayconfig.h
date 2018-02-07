#ifndef DISPLAYCONFIG_H
#define DISPLAYCONFIG_H

#include <QObject>

#include "rviz/visualization_manager.h"
#include "rviz/quick_visualization_frame.h"
#include "rviz/display.h"

class DisplayConfig : public QObject
{
    Q_OBJECT
    Q_PROPERTY(rviz::QuickVisualizationFrame* frame READ getFrame WRITE setFrame NOTIFY frameChanged)
    Q_PROPERTY(QString source READ getSource WRITE setSource NOTIFY sourceChanged)
    Q_PROPERTY(bool loaded READ getLoaded NOTIFY loadedChanged)

public:
  explicit DisplayConfig(QObject *parent = nullptr);

  ~DisplayConfig();

  rviz::QuickVisualizationFrame* getFrame() const;
  QString getSource() const;
  bool getLoaded() const;

Q_SIGNALS:
  void frameChanged(rviz::QuickVisualizationFrame* frame);
  void sourceChanged(const QString &source);
  void loadedChanged(bool loaded);

public Q_SLOTS:
  void setFrame(rviz::QuickVisualizationFrame* frame);
  void setSource(const QString &source);

private:
  rviz::QuickVisualizationFrame* frame_;
  QString source_;
  bool loaded_;

  bool frameIsInitialized();

private Q_SLOTS:
  void initialize();
  void updateConfig();
};

#endif // DISPLAYCONFIG_H
