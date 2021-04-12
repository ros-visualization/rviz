#include "image_render_panel.h"

namespace  rviz {
ImageRenderPanel::ImageRenderPanel(QWidget* parent) : RenderPanel(parent) {}

ImageRenderPanel::~ImageRenderPanel()
{
}

void ImageRenderPanel::mouseMoveEvent(QMouseEvent* event)
{
  QWidget::mouseMoveEvent(event);
}

void ImageRenderPanel::mousePressEvent(QMouseEvent* event)
{
  QWidget::mousePressEvent(event);
}

void ImageRenderPanel::mouseReleaseEvent(QMouseEvent* event)
{
  QWidget::mouseReleaseEvent(event);
}
}
