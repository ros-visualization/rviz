#ifndef RVIZ_IMAGE_RENDER_PANEL_H
#define RVIZ_IMAGE_RENDER_PANEL_H

#include "rviz/render_panel.h"
namespace rviz
{
class ImageRenderPanel : public RenderPanel
{
public:
  ImageRenderPanel(QWidget* parent = nullptr);
  ~ImageRenderPanel() override;

protected:
  void mouseMoveEvent(QMouseEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
};
} // namespace rviz

#endif // RVIZ_IMAGE_RENDER_PANEL_H
