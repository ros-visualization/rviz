#ifndef QTWIDGETOGRERENDERWINDOW_H
#define QTWIDGETOGRERENDERWINDOW_H
#include "qt_ogre_render_window.h"

namespace rviz
{

/**
 * Qt Ogre render window widget.  Similar in API to
 *  wxOgreRenderWindow from ogre_tools release 1.6, but with much of
 *  the guts replaced by new RenderSystem and RenderWidget classes
 *  inspired by the initialization sequence of Gazebo's renderer.
 */
class QtWidgetOgreRenderWindow : public RenderWidget, public QtOgreRenderWindow {
    Q_OBJECT

public:
  /** Constructor.
    @param parent The parent wxWindow component.
   */
  QtWidgetOgreRenderWindow( QWidget* parent = 0 );

  /** Destructor.  */
  virtual ~QtWidgetOgreRenderWindow();

  virtual void setPreRenderCallback( boost::function<void ()> func );
  virtual void setPostRenderCallback( boost::function<void ()> func );

  /** Overrides the default implementation.
    This override is here for convenience. Returns a symbolic 320x240px size.
    @return A size of 320x240 (just a symbolic 4:3 size).
   */
  virtual QSize sizeHint () const { return QSize( 320, 240 ); }

  Ogre::Viewport* getViewport() const;

  Ogre::RenderWindow* getRenderWindow() { return render_window_; }

  void setCamera( Ogre::Camera* camera );

  Ogre::Camera* getCamera() const { return camera_; }

  void setOrthoScale( float scale );

  bool enableStereo(bool enable);

  void setupStereo();

  void setAutoRender(bool auto_render) { auto_render_ = auto_render; }

  ////// Functions mimicked from Ogre::Viewport to satisfy timing of
  ////// after-constructor creation of Ogre::RenderWindow.
  void setOverlaysEnabled( bool overlays_enabled );
  void setBackgroundColor( Ogre::ColourValue color );

  void setFocus(Qt::FocusReason reason);
  QPoint mapFromGlobal(const QPoint &point) const;
  QPoint mapToGlobal(const QPoint &point) const;
  void setCursor(const QCursor &cursor);

  void keyPressEvent( QKeyEvent* event);
  void wheelEvent( QWheelEvent* event);
  void leaveEvent( QEvent* event);

  QRect rect() const;

protected:
  virtual void paintEvent( QPaintEvent* e );
  virtual void resizeEvent( QResizeEvent* event );

  // When stereo is enabled, these are called before/after rendering each
  // viewport.
  virtual void preViewportUpdate(const Ogre::RenderTargetViewportEvent& evt);
  virtual void postViewportUpdate(const Ogre::RenderTargetViewportEvent& evt);

  /**
   * Sets the aspect ratio on the camera
   */
  void setCameraAspectRatio();

  /**
   * prepare a viewport's camera for stereo rendering.
   * This should only be called from StereoRenderTargetListener
   */
  void prepareStereoViewport(Ogre::Viewport*);

  Ogre::Viewport* viewport_;

  Ogre::Root* ogre_root_;

  boost::function<void ()> pre_render_callback_;      ///< Functor which is called before each render
  boost::function<void ()> post_render_callback_;     ///< Functor which is called after each render

  float ortho_scale_;
  bool auto_render_;

  Ogre::Camera* camera_;
  bool overlays_enabled_;
  Ogre::ColourValue background_color_;

  // stereo rendering
  bool stereo_enabled_;				// true if we were asked to render stereo
  bool rendering_stereo_;			// true if we are actually rendering stereo
  Ogre::Camera* left_camera_;
  Ogre::Camera* right_camera_;
  Ogre::Viewport* right_viewport_;
};

} // namespace rviz

#endif // QTWIDGETOGRERENDERWINDOW_H
