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
 * File: MovableText.cpp
 *
 * description: This create create a billboarding object that display a text.
 *
 * @author  2003 by cTh see gavocanov@rambler.ru
 * @update  2006 by barraq see nospam@barraquand.com
 */

#include "movable_text.h"

#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <OgreRoot.h>
#include <OgreCamera.h>
#include <OgreSceneNode.h>
#include <OgreMaterialManager.h>
#include <OgreHardwareBufferManager.h>
#include <OgreFontManager.h>
#include <OgreFont.h>

#include <sstream>

using namespace Ogre;

#define POS_TEX_BINDING    0
#define COLOUR_BINDING     1

namespace rviz
{

MovableText::MovableText(const String &caption, const String &fontName, Real charHeight, const ColourValue &color)
: mFontName(fontName)
, mType("MovableText")
, mCaption(caption)
, mHorizontalAlignment(H_LEFT)
, mVerticalAlignment(V_BELOW)
, mColor(color)
, mCharHeight(charHeight)
, mLineSpacing(0.01)
, mSpaceWidth(0)
, mUpdateColors(true)
, mOnTop(false)
, mTimeUntilNextToggle(0)
, mGlobalTranslation(0.0)
, mLocalTranslation(0.0)
, mpCam(NULL)
, mpWin(NULL)
, mpFont(NULL)
{
  static int count = 0;
  std::stringstream ss;
  ss << "MovableText" << count++;
  mName = ss.str();

  mRenderOp.vertexData = NULL;
  this->setFontName(mFontName);
  this->_setupGeometry();
}

MovableText::~MovableText()
{
  if (mRenderOp.vertexData)
    delete mRenderOp.vertexData;
  // May cause crashing... check this and comment if it does
  if (!mpMaterial.isNull())
    MaterialManager::getSingletonPtr()->remove(mpMaterial->getName());
}

void MovableText::setFontName(const String &fontName)
{
  if ((Ogre::MaterialManager::getSingletonPtr()->resourceExists(mName + "Material")))
  {
    Ogre::MaterialManager::getSingleton().remove(mName + "Material");
  }

  if (mFontName != fontName || mpMaterial.isNull() || !mpFont)
  {
    mFontName = fontName;
    mpFont
        = (Font *) FontManager::getSingleton().getByName(mFontName).getPointer();
    if (!mpFont)
      throw Exception(Exception::ERR_ITEM_NOT_FOUND, "Could not find font "
          + fontName, "MovableText::setFontName");

    mpFont->load();
    if (!mpMaterial.isNull())
    {
      MaterialManager::getSingletonPtr()->remove(mpMaterial->getName());
      mpMaterial.setNull();
    }

    mpMaterial = mpFont->getMaterial()->clone(mName + "Material");
    if (!mpMaterial->isLoaded())
      mpMaterial->load();

    mpMaterial->setDepthCheckEnabled(!mOnTop);
    mpMaterial->setDepthBias(1.0, 1.0);
    mpMaterial->setDepthWriteEnabled(mOnTop);
    mpMaterial->setLightingEnabled(false);
    mNeedUpdate = true;
  }
}

void MovableText::setCaption(const String &caption)
{
  if (caption != mCaption)
  {
    mCaption = caption;
    mNeedUpdate = true;
  }
}

void MovableText::setColor(const ColourValue &color)
{
  if (color != mColor)
  {
    mColor = color;
    mUpdateColors = true;
  }
}

void MovableText::setCharacterHeight(Real height)
{
  if (height != mCharHeight)
  {
    mCharHeight = height;
    mNeedUpdate = true;
  }
}

void MovableText::setLineSpacing(Real height)
{
  if (height != mLineSpacing)
  {
    mLineSpacing = height;
    mNeedUpdate = true;
  }
}

void MovableText::setSpaceWidth(Real width)
{
  if (width != mSpaceWidth)
  {
    mSpaceWidth = width;
    mNeedUpdate = true;
  }
}

void MovableText::setTextAlignment(
    const HorizontalAlignment& horizontalAlignment,
    const VerticalAlignment& verticalAlignment)
{
  if (mHorizontalAlignment != horizontalAlignment)
  {
    mHorizontalAlignment = horizontalAlignment;
    mNeedUpdate = true;
  }
  if (mVerticalAlignment != verticalAlignment)
  {
    mVerticalAlignment = verticalAlignment;
    mNeedUpdate = true;
  }
}

void MovableText::setGlobalTranslation(Vector3 trans)
{
  mGlobalTranslation = trans;
}

void MovableText::setLocalTranslation(Vector3 trans)
{
  mLocalTranslation = trans;
}

void MovableText::showOnTop(bool show)
{
  if (mOnTop != show && !mpMaterial.isNull())
  {
    mOnTop = show;
    mpMaterial->setDepthBias(1.0, 1.0);
    mpMaterial->setDepthCheckEnabled(!mOnTop);
    mpMaterial->setDepthWriteEnabled(mOnTop);
  }
}

void MovableText::_setupGeometry()
{
  assert(mpFont);
  assert(!mpMaterial.isNull());

  unsigned int vertexCount = 0;

  //count letters to determine how many vertices are needed
  std::string::iterator i = mCaption.begin();
  std::string::iterator iend = mCaption.end();
  for ( ; i != iend; ++i )
  {
    if ((*i != ' ') && (*i != '\n'))
    {
      vertexCount += 6;
    }
  }

  if (mRenderOp.vertexData)
  {
    delete mRenderOp.vertexData;
    mRenderOp.vertexData = NULL;
    mUpdateColors = true;
  }

  if (mCaption.empty())
  {
    return;
  }

  if (!mRenderOp.vertexData)
    mRenderOp.vertexData = new VertexData();

  mRenderOp.indexData = 0;
  mRenderOp.vertexData->vertexStart = 0;
  mRenderOp.vertexData->vertexCount = vertexCount;
  mRenderOp.operationType = RenderOperation::OT_TRIANGLE_LIST;
  mRenderOp.useIndexes = false;

  VertexDeclaration *decl = mRenderOp.vertexData->vertexDeclaration;
  VertexBufferBinding *bind = mRenderOp.vertexData->vertexBufferBinding;
  size_t offset = 0;

  // create/bind positions/tex.ccord. buffer
  if (!decl->findElementBySemantic(VES_POSITION))
    decl->addElement(POS_TEX_BINDING, offset, VET_FLOAT3, VES_POSITION);

  offset += VertexElement::getTypeSize(VET_FLOAT3);

  if (!decl->findElementBySemantic(VES_TEXTURE_COORDINATES))
    decl->addElement(POS_TEX_BINDING, offset, Ogre::VET_FLOAT2,
        Ogre::VES_TEXTURE_COORDINATES, 0);

  HardwareVertexBufferSharedPtr ptbuf =
      HardwareBufferManager::getSingleton().createVertexBuffer(
          decl->getVertexSize(POS_TEX_BINDING),
          mRenderOp.vertexData->vertexCount,
          HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY);
  bind->setBinding(POS_TEX_BINDING, ptbuf);

  // Colours - store these in a separate buffer because they change less often
  if (!decl->findElementBySemantic(VES_DIFFUSE))
    decl->addElement(COLOUR_BINDING, 0, VET_COLOUR, VES_DIFFUSE);

  HardwareVertexBufferSharedPtr cbuf =
      HardwareBufferManager::getSingleton().createVertexBuffer(
          decl->getVertexSize(COLOUR_BINDING),
          mRenderOp.vertexData->vertexCount,
          HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY);
  bind->setBinding(COLOUR_BINDING, cbuf);

  float *pPCBuff =
      static_cast<float*> (ptbuf->lock(HardwareBuffer::HBL_DISCARD));

  Real spaceWidth = mSpaceWidth;
  // Derive space width from a capital A
  if (spaceWidth == 0)
    spaceWidth = mpFont->getGlyphAspectRatio('A') * mCharHeight * 2.0;

  float total_height = mCharHeight;
  float total_width = 0.0f;
  float current_width = 0.0f;
  i = mCaption.begin();
  iend = mCaption.end();
  for ( ; i != iend; ++i )
  {
    if (*i == '\n')
    {
      total_height += mCharHeight + mLineSpacing;

      if ( current_width > total_width )
      {
        total_width = current_width;
      }
      current_width = 0.0;
    }
    else if (*i == ' ')
    {
      current_width += spaceWidth;
    }
    else
    {
      current_width += mpFont->getGlyphAspectRatio(*i) * mCharHeight * 2.0;
    }
  }

  if ( current_width > total_width )
  {
    total_width = current_width;
  }

  float top = 0.0f;
  switch (mVerticalAlignment)
  {
  case MovableText::V_ABOVE:
    top = total_height * 2;
    break;
  case MovableText::V_CENTER:
    top = 0.5 * total_height * 2;
    break;
  case MovableText::V_BELOW:
    top = 0.0f;
    break;
  }

  float starting_left = 0.0f;
  switch (mHorizontalAlignment)
  {
  case MovableText::H_LEFT:
    starting_left = 0.0f;
    break;
  case MovableText::H_CENTER:
    starting_left = -total_width / 2.0f;
    break;
  }

  float left = starting_left;

  bool newLine = true;
  Real len = 0.0f;
  // for calculation of AABB
  Ogre::Vector3 min(9999999.0f), max(-9999999.0f), currPos(0.0f);
  Ogre::Real maxSquaredRadius = -99999999.0f;
  float largestWidth = 0.0f;
  for (i = mCaption.begin(); i != iend; ++i)
  {
    if (newLine)
    {
      len = 0.0f;
      for (String::iterator j = i; j != iend && *j != '\n'; j++)
      {
        if (*j == ' ')
          len += spaceWidth;
        else
          len += mpFont->getGlyphAspectRatio(*j) * mCharHeight * 2.0;
      }
      newLine = false;
    }

    if (*i == '\n')
    {
      left = starting_left;
      top -= (mCharHeight + mLineSpacing) * 2.0;
      newLine = true;
      continue;
    }

    if (*i == ' ')
    {
      // Just leave a gap, no tris
      left += spaceWidth;
      currPos = Ogre::Vector3(left, top, 0.0);
      min.makeFloor(currPos);
      max.makeCeil(currPos);
      maxSquaredRadius = std::max(maxSquaredRadius, currPos.squaredLength());
      continue;
    }

    Real horiz_height = mpFont->getGlyphAspectRatio(*i);
    Real u1, u2, v1, v2;
    Ogre::Font::UVRect utmp;
    utmp = mpFont->getGlyphTexCoords(*i);
    u1 = utmp.left;
    u2 = utmp.right;
    v1 = utmp.top;
    v2 = utmp.bottom;

    // each vert is (x, y, z, u, v)
    //-------------------------------------------------------------------------------------
    // First tri
    //
    // Upper left
    currPos = Ogre::Vector3(left, top, 0.0);

    *pPCBuff++ = currPos.x;
    *pPCBuff++ = currPos.y;
    *pPCBuff++ = currPos.z;
    *pPCBuff++ = u1;
    *pPCBuff++ = v1;

    // Deal with bounds


    min.makeFloor(currPos);
    max.makeCeil(currPos);
    maxSquaredRadius = std::max(maxSquaredRadius, currPos.squaredLength());

    top -= mCharHeight * 2.0;

    // Bottom left
    currPos = Ogre::Vector3(left, top, 0.0);
    *pPCBuff++ = currPos.x;
    *pPCBuff++ = currPos.y;
    *pPCBuff++ = currPos.z;
    *pPCBuff++ = u1;
    *pPCBuff++ = v2;

    // Deal with bounds
    min.makeFloor(currPos);
    max.makeCeil(currPos);
    maxSquaredRadius = std::max(maxSquaredRadius, currPos.squaredLength());

    top += mCharHeight * 2.0;
    left += horiz_height * mCharHeight * 2.0;

    // Top right
    currPos = Ogre::Vector3(left, top, 0.0);
    *pPCBuff++ = currPos.x;
    *pPCBuff++ = currPos.y;
    *pPCBuff++ = currPos.z;
    *pPCBuff++ = u2;
    *pPCBuff++ = v1;
    //-------------------------------------------------------------------------------------

    // Deal with bounds
    min.makeFloor(currPos);
    max.makeCeil(currPos);
    maxSquaredRadius = std::max(maxSquaredRadius, currPos.squaredLength());

    //-------------------------------------------------------------------------------------
    // Second tri
    //
    // Top right (again)
    currPos = Ogre::Vector3(left, top, 0.0);
    *pPCBuff++ = currPos.x;
    *pPCBuff++ = currPos.y;
    *pPCBuff++ = currPos.z;
    *pPCBuff++ = u2;
    *pPCBuff++ = v1;

    min.makeFloor(currPos);
    max.makeCeil(currPos);
    maxSquaredRadius = std::max(maxSquaredRadius, currPos.squaredLength());

    top -= mCharHeight * 2.0;
    left -= horiz_height * mCharHeight * 2.0;

    // Bottom left (again)
    currPos = Ogre::Vector3(left, top, 0.0);
    *pPCBuff++ = currPos.x;
    *pPCBuff++ = currPos.y;
    *pPCBuff++ = currPos.z;
    *pPCBuff++ = u1;
    *pPCBuff++ = v2;

    min.makeFloor(currPos);
    max.makeCeil(currPos);
    maxSquaredRadius = std::max(maxSquaredRadius, currPos.squaredLength());

    left += horiz_height * mCharHeight * 2.0;

    // Bottom right
    currPos = Ogre::Vector3(left, top, 0.0);
    *pPCBuff++ = currPos.x;
    *pPCBuff++ = currPos.y;
    *pPCBuff++ = currPos.z;
    *pPCBuff++ = u2;
    *pPCBuff++ = v2;
    //-------------------------------------------------------------------------------------
    min.makeFloor(currPos);
    max.makeCeil(currPos);
    maxSquaredRadius = std::max(maxSquaredRadius, currPos.squaredLength());

    // Go back up with top
    top += mCharHeight * 2.0;

    float currentWidth = (left + 1) / 2 - 0;
    if (currentWidth > largestWidth)
      largestWidth = currentWidth;
  }
  // Taking empty last line into account for the AABB
  if(newLine)
  {
    top -= mCharHeight * 2.0;
    currPos = Ogre::Vector3(left, top, 0.0);
    min.makeFloor(currPos);
    max.makeCeil(currPos);
    maxSquaredRadius = std::max(maxSquaredRadius, currPos.squaredLength());
  }
  // Unlock vertex buffer
  ptbuf->unlock();

  // update AABB/Sphere radius
  mAABB = Ogre::AxisAlignedBox(min, max);
  mRadius = Ogre::Math::Sqrt(maxSquaredRadius);

  if (mUpdateColors)
    this->_updateColors();

  mNeedUpdate = false;
}

void MovableText::_updateColors(void)
{
  assert(mpFont);
  assert(!mpMaterial.isNull());

  // Convert to system-specific
  RGBA color;
  Root::getSingleton().convertColourValue(mColor, &color);
  HardwareVertexBufferSharedPtr vbuf =
      mRenderOp.vertexData->vertexBufferBinding->getBuffer(COLOUR_BINDING);
  RGBA *pDest = static_cast<RGBA*> (vbuf->lock(HardwareBuffer::HBL_DISCARD));
  for (int i = 0; i < (int) mRenderOp.vertexData->vertexCount; ++i)
    *pDest++ = color;
  vbuf->unlock();
  mUpdateColors = false;
}

const Quaternion& MovableText::getWorldOrientation(void) const
{
  assert(mpCam);
  return const_cast<Quaternion&> (mpCam->getDerivedOrientation());
}

#if( (OGRE_VERSION_MAJOR >= 1 && OGRE_VERSION_MINOR >= 6) || OGRE_VERSION_MAJOR >= 2 )
void MovableText::visitRenderables(Ogre::Renderable::Visitor* visitor, bool debugRenderables)
{
  visitor->visit( this, 0, false );
}
#endif

const Vector3& MovableText::getWorldPosition(void) const
{
  assert(mParentNode);
  return mParentNode->_getDerivedPosition();
}

void MovableText::getWorldTransforms(Matrix4 *xform) const
{
  if (this->isVisible() && mpCam)
  {
    Matrix3 rot3x3, scale3x3 = Matrix3::IDENTITY;

    // store rotation in a matrix
    mpCam->getDerivedOrientation().ToRotationMatrix(rot3x3);

    // parent node position
    Vector3 ppos = mParentNode->_getDerivedPosition() + Vector3::UNIT_Y
        * mGlobalTranslation;
    ppos += rot3x3 * mLocalTranslation;

    // apply scale
    scale3x3[0][0] = mParentNode->_getDerivedScale().x / 2;
    scale3x3[1][1] = mParentNode->_getDerivedScale().y / 2;
    scale3x3[2][2] = mParentNode->_getDerivedScale().z / 2;

    // apply all transforms to xform
    *xform = (rot3x3 * scale3x3);
    xform->setTrans(ppos);
  }
}

void MovableText::getRenderOperation(RenderOperation &op)
{
  if (this->isVisible())
  {
    if (mNeedUpdate)
      this->_setupGeometry();
    if (mUpdateColors)
      this->_updateColors();
    op = mRenderOp;
  }
}

void MovableText::_notifyCurrentCamera(Camera *cam)
{
  mpCam = cam;
}

void MovableText::_updateRenderQueue(RenderQueue* queue)
{
  if (this->isVisible())
  {
    if (mNeedUpdate)
      this->_setupGeometry();
    if (mUpdateColors)
      this->_updateColors();

    queue->addRenderable(this, mRenderQueueID, OGRE_RENDERABLE_DEFAULT_PRIORITY);
    //queue->addRenderable(this, mRenderQueueID, RENDER_QUEUE_SKIES_LATE);
  }
}

} // namespace rviz

