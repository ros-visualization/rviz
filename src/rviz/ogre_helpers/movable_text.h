/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Adapted from: http://www.ogre3d.org/wiki/index.php/MovableText
//          now: http://www.ogre3d.org/tikiwiki/tiki-index.php?page=MovableText
// Original authors:
/*
 * File: MovableText.h
 *
 * description: This create create a billboarding object that display a text.
 *
 * @author  2003 by cTh see gavocanov@rambler.ru
 * @update  2006 by barraq see nospam@barraquand.com
 */

#ifndef OGRE_TOOLS_MOVABLE_TEXT_H
#define OGRE_TOOLS_MOVABLE_TEXT_H

#include <OgreMovableObject.h>
#include <OgreRenderable.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <OgreSharedPtr.h>


namespace Ogre
{
class RenderQueue;
class Camera;
class Font;
}

namespace rviz
{

class MovableText : public Ogre::MovableObject, public Ogre::Renderable
{
  /******************************** MovableText data ****************************/
public:
  enum HorizontalAlignment
  {
    H_LEFT, H_CENTER
  };
  enum VerticalAlignment
  {
    V_BELOW, V_ABOVE, V_CENTER
  };

protected:
  Ogre::String mFontName;
  Ogre::String mType;
  Ogre::String mName;
  Ogre::String mCaption;
  HorizontalAlignment mHorizontalAlignment;
  VerticalAlignment mVerticalAlignment;

  Ogre::ColourValue mColor;
  Ogre::RenderOperation mRenderOp;
  Ogre::AxisAlignedBox mAABB;
  Ogre::LightList mLList;

  Ogre::Real mCharHeight;
  Ogre::Real mLineSpacing;
  Ogre::Real mSpaceWidth;

  bool mNeedUpdate;
  bool mUpdateColors;
  bool mOnTop;

  Ogre::Real mTimeUntilNextToggle;
  Ogre::Real mRadius;

  Ogre::Vector3 mGlobalTranslation;
  Ogre::Vector3 mLocalTranslation;

  Ogre::Camera *mpCam;
  Ogre::RenderWindow *mpWin;
  Ogre::Font *mpFont;
  Ogre::MaterialPtr mpMaterial;
  Ogre::MaterialPtr mpBackgroundMaterial;

  /******************************** public methods ******************************/
public:
  MovableText(
    const Ogre::String &caption,
    const Ogre::String &fontName = "Liberation Sans",
    Ogre::Real charHeight = 1.0,
    const Ogre::ColourValue &color = Ogre::ColourValue::White);
  virtual ~MovableText();

#if (OGRE_VERSION_MAJOR >= 1 && OGRE_VERSION_MINOR >= 6)
  virtual void visitRenderables(Ogre::Renderable::Visitor* visitor, bool debugRenderables = false);
#endif

  // Set settings
  void setFontName(const Ogre::String &fontName);
  void setCaption(const Ogre::String &caption);
  void setColor(const Ogre::ColourValue &color);
  void setCharacterHeight(Ogre::Real height);
  void setLineSpacing(Ogre::Real height);
  void setSpaceWidth(Ogre::Real width);
  void setTextAlignment(const HorizontalAlignment& horizontalAlignment,
      const VerticalAlignment& verticalAlignment);
  void setGlobalTranslation(Ogre::Vector3 trans);
  void setLocalTranslation(Ogre::Vector3 trans);
  void showOnTop(bool show = true);

  // Get settings
  const Ogre::String &getFontName() const
  {
    return mFontName;
  }
  const Ogre::String &getCaption() const
  {
    return mCaption;
  }
  const Ogre::ColourValue &getColor() const
  {
    return mColor;
  }

  Ogre::Real getCharacterHeight() const
  {
    return mCharHeight;
  }
  Ogre::Real getSpaceWidth() const
  {
    return mSpaceWidth;
  }
  Ogre::Vector3 getGlobalTranslation() const
  {
    return mGlobalTranslation;
  }
  Ogre::Vector3 getLocalTranslation() const
  {
    return mLocalTranslation;
  }
  bool getShowOnTop() const
  {
    return mOnTop;
  }
  Ogre::AxisAlignedBox GetAABB(void)
  {
    return mAABB;
  }

  const Ogre::MaterialPtr &getMaterial(void) const
  {
    assert(!mpMaterial.isNull());
    return mpMaterial;
  }
  ;


  /******************************** protected methods and overload **************/
protected:

  // from MovableText, create the object
  void _setupGeometry();
  void _updateColors();

  // from Ogre::MovableObject
  void getWorldTransforms(Ogre::Matrix4 *xform) const;
  Ogre::Real getBoundingRadius(void) const
  {
    return mRadius;
  }
  ;
  Ogre::Real getSquaredViewDepth(const Ogre::Camera *cam) const
  {
    return 0;
  }
  ;
  const Ogre::Quaternion &getWorldOrientation(void) const;
  const Ogre::Vector3 &getWorldPosition(void) const;
  const Ogre::AxisAlignedBox &getBoundingBox(void) const
  {
    return mAABB;
  }
  ;
  const Ogre::String &getName(void) const
  {
    return mName;
  }
  ;
  const Ogre::String &getMovableType(void) const
  {
    static Ogre::String movType = "MovableText";
    return movType;
  }
  ;

  void _notifyCurrentCamera(Ogre::Camera *cam);
  void _updateRenderQueue(Ogre::RenderQueue* queue);

  // from renderable
  void getRenderOperation(Ogre::RenderOperation &op);
  const Ogre::LightList &getLights(void) const
  {
    return mLList;
  }
  ;
};

}

#endif
