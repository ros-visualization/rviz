/*
 * OgreRenderQueueClearer.cpp
 *
 *  Created on: Aug 23, 2012
 *      Author: gossow
 */

#include "ogre_render_queue_clearer.h"

#include <OGRE/OgreRoot.h>
#include <OGRE/OgrePass.h>

namespace rviz
{

OgreRenderQueueClearer::OgreRenderQueueClearer() {
	// TODO Auto-generated constructor stub

}

OgreRenderQueueClearer::~OgreRenderQueueClearer() {
	// TODO Auto-generated destructor stub
}


bool OgreRenderQueueClearer::frameStarted (const Ogre::FrameEvent &evt)
{
	// Workaround taken from http://www.ogre3d.org/mantis/view.php?id=130
	// in case a plugin creates its own scene manager.
	//
	// Has the following modifications:
	// 1. Need to pass 'true' to RenderQueue::clear( bool destroyPassMaps ).
	// 2. Don't need to call Ogre::Pass::processPendingPassUpdates(),
	//    since this is done within RenderQueue::clear().

	// This workaround is only necessary if there is more than one
	// scene manager present, so check for that
	Ogre::SceneManagerEnumerator::SceneManagerIterator it = Ogre::Root::getSingletonPtr()->getSceneManagerIterator();
	it.getNext();
	if ( !it.hasMoreElements() )
	{
		return true;
	}

	it = Ogre::Root::getSingletonPtr()->getSceneManagerIterator();
	while ( it.hasMoreElements() )
	{
		it.getNext()->getRenderQueue()->clear( true );
	}
	return true;
}

}

