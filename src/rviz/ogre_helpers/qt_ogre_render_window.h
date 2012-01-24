#ifndef QT_OGRE_RENDER_WINDOW_OGRE_RENDER_WINDOW_H_
#define QT_OGRE_RENDER_WINDOW_OGRE_RENDER_WINDOW_H_

#include <boost/function.hpp>

#include "render_widget.h"

#include <OGRE/OgreColourValue.h>

namespace Ogre
{
class Root;
class RenderWindow;
class Viewport;
class Camera;
}

namespace rviz
{

/** Qt Ogre render window widget.  Similar in API to
 *  wxOgreRenderWindow from ogre_tools release 1.6, but with much of
 *  the guts replaced by new RenderSystem and RenderWidget classes
 *  inspired by the initialization sequence of Gazebo's renderer.
 */
class QtOgreRenderWindow : public RenderWidget {
public:
  /** Constructor.
  	@param parent The parent wxWindow component.
   */
  QtOgreRenderWindow( QWidget* parent = 0 );

  /**
   * Set a callback which is called before each render
   * @param func The callback functor
   */
  virtual void setPreRenderCallback( boost::function<void ()> func );
  /**
     * Set a callback which is called after each render
     * @param func The callback functor
     */
  virtual void setPostRenderCallback( boost::function<void ()> func );

  /** Overrides the default implementation.
  	This override is here for convenience. Returns a symbolic 320x240px size.
  	@return A size of 320x240 (just a symbolic 4:3 size).
   */
  virtual QSize sizeHint () const { return QSize( 320, 240 ); }

  /** Gets the associated Ogre viewport.  If this is called before
   * QWidget::show() on this widget, it will fail an assertion.
   * Several functions of Ogre::Viewport are duplicated in this class
   * which can be called before QWidget::show(), and their effects are
   * propagated to the viewport when it is created.
   */
  Ogre::Viewport* getViewport() const;

  /** Set the camera associated with this render window's viewport.
   */
  void setCamera( Ogre::Camera* camera );

  /**
   * \brief Set the scale of the orthographic window.  Only valid for an orthographic camera.
   * @param scale The scale
   */
  void setOrthoScale( float scale );

  void setAutoRender(bool auto_render) { auto_render_ = auto_render; }

  ////// Functions mimicked from Ogre::Viewport to satisfy timing of
  ////// after-constructor creation of Ogre::RenderWindow.
  void setOverlaysEnabled( bool overlays_enabled );
  void setBackgroundColor( Ogre::ColourValue color );

protected:
  virtual void paintEvent( QPaintEvent* e );
  virtual void resizeEvent( QResizeEvent* event );

  /**
   * Sets the aspect ratio on the camera
   */
  void setCameraAspectRatio();

  Ogre::Viewport* viewport_;

  Ogre::Root* ogre_root_;

  boost::function<void ()> pre_render_callback_;      ///< Functor which is called before each render
  boost::function<void ()> post_render_callback_;     ///< Functor which is called after each render

  float ortho_scale_;
  bool auto_render_;

  Ogre::Camera* camera_;
  bool overlays_enabled_;
  Ogre::ColourValue background_color_;
};

} // namespace rviz

#endif // QT_OGRE_RENDER_WINDOW_OGRE_RENDER_WINDOW_H_
