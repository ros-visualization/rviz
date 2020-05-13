#ifndef WRENCHSTAMPED_VISUAL_H
#define WRENCHSTAMPED_VISUAL_H

#include <geometry_msgs/Wrench.h>
#include "screw_visual.h"

namespace rviz
{
// For API compatibility we provide the old WrenchVisual class as well
class [[deprecated("Replace with ScrewVisual")]] WrenchVisual;
class RVIZ_EXPORT WrenchVisual : public ScrewVisual
{
public:
  using ScrewVisual::ScrewVisual;

  inline void setWrench(const Ogre::Vector3& force, const Ogre::Vector3& torque)
  {
    setScrew(force, torque);
  }
  inline void setWrench(const geometry_msgs::Wrench& wrench)
  {
    setScrew(wrench.force, wrench.torque);
  }

  inline void setForceColor(float r, float g, float b, float a)
  {
    setLinearColor(r, g, b, a);
  }
  inline void setTorqueColor(float r, float g, float b, float a)
  {
    setAngularColor(r, g, b, a);
  }
  inline void setForceScale(float s)
  {
    setLinearScale(s);
  }
  inline void setTorqueScale(float s)
  {
    setAngularScale(s);
  }
};

} // end namespace rviz

#endif // WRENCHSTAMPED_VISUAL_H
