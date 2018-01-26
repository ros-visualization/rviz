#include "simplegrid.h"

#include <QColor>

#include <ros/ros.h>


SimpleGrid::SimpleGrid(QObject *parent)
  : QObject(parent)
  , grid_(nullptr)
  , frame_(nullptr)
  , line_width_(1)
  , color_(Qt::white)
{
  connect(this, &SimpleGrid::lineWidthChanged, this, &SimpleGrid::updateProperties);
  connect(this, &SimpleGrid::colorChanged, this, &SimpleGrid::updateProperties);
}

SimpleGrid::~SimpleGrid()
{

}

rviz::QuickVisualizationFrame *SimpleGrid::getFrame() const
{
  return frame_;
}

int SimpleGrid::getLineWidth() const
{
  return line_width_;
}

QColor SimpleGrid::getColor() const
{
  return color_;
}

void SimpleGrid::setFrame(rviz::QuickVisualizationFrame *frame)
{
  if (frame_ == frame) {
    return;
  }

  frame_ = frame;
  Q_EMIT frameChanged(frame_);

  if (frame_->isInitialized()) {
      initialize();
  }
  else {
    connect(frame_, &rviz::QuickVisualizationFrame::initializedChanged,
            this, &SimpleGrid::initialize);
  }
}

void SimpleGrid::setLineWidth(int line_width)
{
  if (line_width_ == line_width) {
    return;
  }

  line_width_ = line_width;
  Q_EMIT lineWidthChanged(line_width_);
}

void SimpleGrid::setColor(QColor color)
{
  if (color_ == color) {
    return;
  }

  color_ = color;
  Q_EMIT colorChanged(color_);
}

void SimpleGrid::initialize()
{
  grid_ = frame_->getManager()->createDisplay( "rviz/Grid", "My grid", true );
  ROS_ASSERT( grid_ != NULL );

  updateProperties();
}

void SimpleGrid::updateProperties()
{
  if (!grid_) {
    return;
  }

  grid_->subProp("Line Style")->subProp("Line Width")->setValue(line_width_);
  grid_->subProp( "Color" )->setValue( color_ );
}
