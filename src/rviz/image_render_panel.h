#include "rviz/render_panel.h"
namespace rviz {
class ImageRenderPanel ：public RenderPanel {
public:
ImageRenderPanel（QWidget* parent = nullptr);
protected:
void mouseMoveEvent(QMouseEvent* event)override;
void mousePressEvent(QMouseEvent* event)override;
void mouseReleaseEvent(QMouseEvent* event)override;
}
}
