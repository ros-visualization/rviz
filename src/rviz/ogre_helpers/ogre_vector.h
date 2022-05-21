// OgrePrerequisites.h is needed for version_check to work
#include <OgrePrerequisites.h>
#include <rviz/ogre_helpers/version_check.h>
#if OGRE_VERSION < OGRE_VERSION_CHECK(1, 12, 0)
#include <OgreVector3.h>
#else
#include <OgreVector.h>
#endif
