#include "initialization.h"

#include <OgreRoot.h>
#include <OgreRenderSystem.h>

#include <exception>
#include <stdexcept>

#include <ros/package.h>

namespace rviz
{

void cleanupOgre()
{
  delete Ogre::Root::getSingletonPtr();
}

// This should be folded into RenderSystem, but it should work fine as
// is, so I'm leaving it for now.
void initializeResources( const V_string& resource_paths )
{
  V_string::const_iterator path_it = resource_paths.begin();
  V_string::const_iterator path_end = resource_paths.end();
  for( ; path_it != path_end; ++path_it )
  {
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( *path_it, "FileSystem", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
  }

  Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
}

} // namespace rviz
