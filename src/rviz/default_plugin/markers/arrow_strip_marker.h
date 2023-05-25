#ifndef RVIZ_ARROW_STRIP_MARKER_H
#define RVIZ_ARROW_STRIP_MARKER_H

#include "marker_base.h"

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class Arrow;
class DisplayContext;

class ArrowStripMarker : public MarkerBase
{
public:
  ArrowStripMarker(MarkerDisplay* owner, DisplayContext* context, Ogre::SceneNode* parent_node);
  ~ArrowStripMarker();

protected:
  void onNewMessage(const MarkerConstPtr& old_message, const MarkerConstPtr& new_message) override;

  std::vector<Arrow*> arrows_;
};
} // namespace rviz

#endif
