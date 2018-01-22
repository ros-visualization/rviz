#ifndef OGRE_VIEWPORT_SUPPORT_H
#define OGRE_VIEWPORT_SUPPORT_H

#include <QRect>

#include <boost/function.hpp>

#include <OgreColourValue.h>
#include <OgreRenderTargetListener.h>

namespace Ogre
{
class RenderWindow;
class Viewport;
class Camera;
class Root;
}

namespace rviz
{

class RenderSystem;

class OgreViewportSupport : public Ogre::RenderTargetListener
{
public:
    explicit OgreViewportSupport();

    ~OgreViewportSupport();

    /** Set the camera associated with this render window's viewport.
     */
    void setCamera( Ogre::Camera* camera );

    Ogre::Camera* getCamera() const { return camera_; }

    Ogre::RenderWindow* getRenderWindow() { return render_window_; }

    /**
       * \brief Set the scale of the orthographic window.  Only valid for an orthographic camera.
       * @param scale The scale
       */
    void setOrthoScale( float scale );

    /** \brief Enable or disable stereo rendering
       * If stereo is not supported this is ignored.
       * @return the old setting (whether stereo was enabled before)
       */
    bool enableStereo(bool enable);

    /** \brief Prepare to render in stereo if enabled and supported. */
    void setupStereo();

    void setAutoRender(bool auto_render) { auto_render_ = auto_render; }

    ////// Functions mimicked from Ogre::Viewport to satisfy timing of
    ////// after-constructor creation of Ogre::RenderWindow.
    void setOverlaysEnabled( bool overlays_enabled );
    void setBackgroundColor( Ogre::ColourValue color );

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

    /** Gets the associated Ogre viewport.  If this is called before
     * QWidget::show() on this widget, it will fail an assertion.
     * Several functions of Ogre::Viewport are duplicated in this class
     * which can be called before QWidget::show(), and their effects are
     * propagated to the viewport when it is created.
     */
    Ogre::Viewport* getViewport() const;

    virtual QRect rect() const = 0;

protected:
    // When stereo is enabled, these are called before/after rendering each
    // viewport.
    virtual void preViewportUpdate(const Ogre::RenderTargetViewportEvent& evt);
    virtual void postViewportUpdate(const Ogre::RenderTargetViewportEvent& evt);

    void initializeRenderSystem();
    void initialize();

    virtual void updateScene() = 0;

    /**
     * Sets the aspect ratio on the camera
     */
    void setCameraAspectRatio();

    /**
     * prepare a viewport's camera for stereo rendering.
     * This should only be called from StereoRenderTargetListener
     */
    void prepareStereoViewport(Ogre::Viewport*);

    RenderSystem* render_system_;
    Ogre::RenderWindow* render_window_;
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

#endif // OGRE_VIEWPORT_SUPPORT_H
