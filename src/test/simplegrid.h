#ifndef SIMPLEGRID_H
#define SIMPLEGRID_H

#include <QObject>

#include "rviz/visualization_manager.h"
#include "rviz/quick_visualization_frame.h"
#include "rviz/display.h"


class SimpleGrid : public QObject
{
  Q_OBJECT
  Q_PROPERTY(rviz::QuickVisualizationFrame* frame READ getFrame WRITE setFrame NOTIFY frameChanged)
  Q_PROPERTY(int lineWidth READ getLineWidth WRITE setLineWidth NOTIFY lineWidthChanged)
  Q_PROPERTY(QColor color READ getColor WRITE setColor NOTIFY colorChanged)

public:
  explicit SimpleGrid(QObject *parent = nullptr);

    ~SimpleGrid();

  rviz::QuickVisualizationFrame* getFrame() const;
  int getLineWidth() const;
  QColor getColor() const;

Q_SIGNALS:
  void frameChanged(rviz::QuickVisualizationFrame* frame);
  void lineWidthChanged(int lineWidth);

  void colorChanged(QColor color);

public Q_SLOTS:
  void setFrame(rviz::QuickVisualizationFrame* frame);
  void setLineWidth(int lineWidth);
  void setColor(QColor color);

private:
  rviz::Display* grid_;

  rviz::QuickVisualizationFrame* frame_;
  int line_width_;
  QColor color_;

private Q_SLOTS:
  void initialize();
  void updateProperties();
};

#endif // SIMPLEGRID_H
