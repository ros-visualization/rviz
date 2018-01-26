#include <GL/glew.h>
#include "qt_quick_ogre_render_window.h"

#include <RenderSystems/GL/OgreGLTexture.h>
#include <RenderSystems/GL/OgreGLFrameBufferObject.h>
#include <RenderSystems/GL/OgreGLFBORenderTexture.h>

#include <OgreRoot.h>
#include <OgreViewport.h>
#include <OgreCamera.h>
#include <OgreRenderWindow.h>
#include <OgreStringConverter.h>
#include <OgreRenderTargetListener.h>

#include <QWindow>
#include <QQuickWindow>
#include <QTimer>
#include <QOpenGLContext>
#include <QOpenGLFunctions>
#include <QSGGeometryNode>

#include <ros/console.h>
#include <ros/assert.h>

#include "render_system.h"

#if OGRE_PLATFORM == OGRE_PLATFORM_LINUX
#include <stdlib.h>
#endif

namespace rviz
{

QtQuickOgreRenderWindow::QtQuickOgreRenderWindow(QQuickItem *parent)
  : QQuickItem(parent)
  , initialized_(false)
  , geometry_(QSGGeometry::defaultAttributes_TexturedPoint2D(), 4)
  , texture_(nullptr)
  , render_target_(nullptr)
{
  setFlag(ItemHasContents);
  setFlag(ItemAcceptsInputMethod);
  setSmooth(false);
  setAcceptHoverEvents(true);
  setAcceptedMouseButtons(Qt::AllButtons);

  if (window()) {
    onWindowChanged(window());
  }
  else {
    connect(this, &QQuickItem::windowChanged, this, &QtQuickOgreRenderWindow::onWindowChanged);
  }
}

QtQuickOgreRenderWindow::~QtQuickOgreRenderWindow()
{

}

void QtQuickOgreRenderWindow::onWindowChanged(QQuickWindow *window)
{
  if (!window || initialized_) {
    return;
  }

  // start Ogre once we are in the rendering thread (Ogre must live in the rendering thread)
  connect(window, &QQuickWindow::beforeRendering,
          this, &QtQuickOgreRenderWindow::initializeOgre, Qt::DirectConnection);

  connect(this, &QtQuickOgreRenderWindow::ogreInitialized, [this]() { Ogre::Root::getSingleton().addFrameListener(this); });

  connect(&update_timer_, &QTimer::timeout, window, &QQuickWindow::update);
  update_timer_.start(16);

  initialized_ = true;
}

void QtQuickOgreRenderWindow::setFocus(Qt::FocusReason)
{
  QQuickItem::setFocus(true);
}

QPoint QtQuickOgreRenderWindow::mapFromGlobal(const QPoint &point) const
{
  return QQuickItem::mapFromGlobal(point).toPoint();
}

QPoint QtQuickOgreRenderWindow::mapToGlobal(const QPoint &point) const
{
  return QQuickItem::mapToGlobal(point).toPoint();
}

void QtQuickOgreRenderWindow::setCursor(const QCursor &cursor)
{
  QQuickItem::setCursor(cursor);
}

bool QtQuickOgreRenderWindow::containsPoint(const QPoint &point) const
{
  return QQuickItem::contains(QPointF(point));
}

double QtQuickOgreRenderWindow::getWindowPixelRatio() const
{
  return window()->devicePixelRatio();
}

QRect QtQuickOgreRenderWindow::rect() const
{
  return QQuickItem::boundingRect().toRect();
}

void QtQuickOgreRenderWindow::keyPressEvent(QKeyEvent *event)
{
  emitKeyPressEvent( event );
}

void QtQuickOgreRenderWindow::wheelEvent(QWheelEvent *event)
{
  emitWheelEvent( event );
}

void QtQuickOgreRenderWindow::mouseMoveEvent(QMouseEvent *event)
{
  emitMouseEvent( event );
}

void QtQuickOgreRenderWindow::mousePressEvent(QMouseEvent *event)
{
  emitMouseEvent( event );
}

void QtQuickOgreRenderWindow::mouseReleaseEvent(QMouseEvent *event)
{
  emitMouseEvent( event );
}

void QtQuickOgreRenderWindow::mouseDoubleClickEvent(QMouseEvent *event)
{
  emitMouseEvent( event );
}

void QtQuickOgreRenderWindow::updateScene()
{
  QQuickItem::update();
}

QSGNode *QtQuickOgreRenderWindow::updatePaintNode(QSGNode *oldNode, QQuickItem::UpdatePaintNodeData *)
{
  disconnect(window(), &QQuickWindow::beforeSynchronizing, this, &QQuickItem::update);
  connect(window(), &QQuickWindow::beforeSynchronizing, this, &QQuickItem::update);

  if (width() <= 0 || height() <= 0 || !texture_)
  {
    if(oldNode) {
      delete oldNode;
    }
    return nullptr;
  }

  QSGGeometryNode *node = static_cast<QSGGeometryNode *>(oldNode);

  if(!node)
  {
    node = new QSGGeometryNode();
    node->setGeometry(&geometry_);
    node->setMaterial(&material_);
    node->setOpaqueMaterial(&material_opaque_);
  }

  node->markDirty(QSGNode::DirtyGeometry);
  node->markDirty(QSGNode::DirtyMaterial);
  return node;
}

void QtQuickOgreRenderWindow::updateFBO()
{
  QSize wsz(static_cast<qint32>(width()), static_cast<qint32>(height()));

  if (width() <= 0 || height() <= 0 || (wsz == size_)) {
    return;
  }

  size_ = wsz;

  if (render_target_) {
    Ogre::TextureManager::getSingleton().remove("RttTex");
  }

  Ogre::TexturePtr rtt = Ogre::TextureManager::getSingleton().createManual
      ("RttTex", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D,
       static_cast<quint32>(size_.width()), static_cast<quint32>(size_.height()),
       0, Ogre::PF_R8G8B8A8, Ogre::TU_RENDERTARGET, 0, false);

  render_target_ = rtt->getBuffer()->getRenderTarget();

  viewport_ = render_target_->addViewport(camera_);
  render_target_->getViewport(0)->setClearEveryFrame(true);
  render_target_->getViewport(0)->setBackgroundColour(background_color_);
  render_target_->getViewport(0)->setOverlaysEnabled(overlays_enabled_);
  render_target_->addListener(this);

  //Ogre::Real aspectRatio = Ogre::Real(m_size.width()) / Ogre::Real(m_size.height());
  //camera_->setAspectRatio(aspectRatio);

  QSGGeometry::updateTexturedRectGeometry(&geometry_,
                                          QRectF(0, 0, size_.width(), size_.height()),
                                          QRectF(0, 0, 1, 1));

  Ogre::GLTexture *native_texture = static_cast<Ogre::GLTexture *>(rtt.get());

  delete texture_;

  texture_ = window()->createTextureFromId(native_texture->getGLID(), size_);

  material_.setTexture(texture_);
  material_opaque_.setTexture(texture_);
}

bool QtQuickOgreRenderWindow::frameStarted(const Ogre::FrameEvent &)
{
  updateFBO();
  return true;
}

void QtQuickOgreRenderWindow::preRenderTargetUpdate(const Ogre::RenderTargetEvent &)
{
  if (render_target_) {
    Ogre::GLFBOManager::getSingleton().bind(render_target_);
  }
}

void QtQuickOgreRenderWindow::postRenderTargetUpdate(const Ogre::RenderTargetEvent &)
{
  if (render_target_) {
    Ogre::GLFBOManager::getSingleton().unbind(render_target_);
  }
}

void QtQuickOgreRenderWindow::initializeOgre()
{
  // we only want to initialize once
  disconnect(window(), &QQuickWindow::beforeRendering,
            this, &QtQuickOgreRenderWindow::initializeOgre);

  // Setup the shared opengl context.
  qt_gl_context_   = QOpenGLContext::currentContext();
  ogre_gl_context_ = new QOpenGLContext();
  ogre_gl_context_->setFormat(window()->requestedFormat());
  ogre_gl_context_->setShareContext(qt_gl_context_);
  ogre_gl_context_->create();

  // Initialize the ogre render system. The ogre context must be activated first
  activateOgreContext();

  OgreViewportSupport::initializeRenderSystem();

  RenderSystem::WindowIDType win_id = window()->winId();
  double pixel_ratio = window()->devicePixelRatio();
  render_window_ = render_system_->makeRenderWindow(win_id, 1, 1, pixel_ratio);
  render_window_->setVisible(false);
  render_window_->update(false);

  OgreViewportSupport::initialize();

  Q_EMIT ogreInitializing();

  doneOgreContext();

  // Connect the before rendering call to render now that we've finished initializing
  // Again the render call must be on the rendering thread.
  connect(window(), &QQuickWindow::beforeRendering,
          this, &QtQuickOgreRenderWindow::render, Qt::DirectConnection);

  Q_EMIT ogreInitialized();
}

void QtQuickOgreRenderWindow::render()
{
  activateOgreContext();
  Ogre::Root::getSingleton().renderOneFrame();
  doneOgreContext();
}

void QtQuickOgreRenderWindow::activateOgreContext()
{
  glPopAttrib();
  glPopClientAttrib();

  qt_gl_context_->functions()->glUseProgram(0);
  qt_gl_context_->doneCurrent();

  ogre_gl_context_->makeCurrent(window());
}

void QtQuickOgreRenderWindow::doneOgreContext()
{
  ogre_gl_context_->functions()->glBindBuffer(GL_ARRAY_BUFFER, 0);
  ogre_gl_context_->functions()->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
  ogre_gl_context_->functions()->glBindRenderbuffer(GL_RENDERBUFFER, 0);
  ogre_gl_context_->functions()->glBindFramebuffer(GL_FRAMEBUFFER_EXT, 0);

  // unbind all possible remaining buffers; just to be on safe side
  ogre_gl_context_->functions()->glBindBuffer(GL_ARRAY_BUFFER, 0);
  ogre_gl_context_->functions()->glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, 0);
  ogre_gl_context_->functions()->glBindBuffer(GL_COPY_READ_BUFFER, 0);
  ogre_gl_context_->functions()->glBindBuffer(GL_COPY_WRITE_BUFFER, 0);
  ogre_gl_context_->functions()->glBindBuffer(GL_DRAW_INDIRECT_BUFFER, 0);
  //    m_ogreContext->functions()->glBindBuffer(GL_DISPATCH_INDIRECT_BUFFER, 0);
  ogre_gl_context_->functions()->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
  ogre_gl_context_->functions()->glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
  ogre_gl_context_->functions()->glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
  //    m_ogreContext->functions()->glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
  ogre_gl_context_->functions()->glBindBuffer(GL_TEXTURE_BUFFER, 0);
  ogre_gl_context_->functions()->glBindBuffer(GL_TRANSFORM_FEEDBACK_BUFFER, 0);
  ogre_gl_context_->functions()->glBindBuffer(GL_UNIFORM_BUFFER, 0);

  ogre_gl_context_->doneCurrent();

  qt_gl_context_->makeCurrent(window());
  glPushAttrib(GL_ALL_ATTRIB_BITS);
  glPushClientAttrib(GL_CLIENT_ALL_ATTRIB_BITS);
}

} // namespace rviz
