#include "qt_quick_ogre_render_window.h"
#include "orthographic.h"
#include "render_system.h"

#include <QApplication>
#include <QWindow>
#include <QQuickWindow>
#include <QOpenGLFunctions>
#include <QTimer>

#include <OgreRoot.h>
#include <OgreViewport.h>
#include <OgreCamera.h>
#include <OgreRenderWindow.h>
#include <OgreStringConverter.h>
#include <OgreGpuProgramManager.h>
#include <OgreRenderTargetListener.h>

#include <ros/console.h>
#include <ros/assert.h>

#if OGRE_PLATFORM == OGRE_PLATFORM_LINUX
#include <stdlib.h>
#endif

namespace rviz
{

QtQuickOgreRenderWindow::QtQuickOgreRenderWindow(QQuickItem *parent)
  : QQuickItem(parent)
{
  connect(this, &QQuickItem::windowChanged, this, &QtQuickOgreRenderWindow::onWindowChanged);

  if (window()) {
    onWindowChanged(window());
  }
}

QtQuickOgreRenderWindow::~QtQuickOgreRenderWindow()
{

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

QRect QtQuickOgreRenderWindow::rect() const
{
  return QQuickItem::boundingRect().toRect();
}

void QtQuickOgreRenderWindow::updateScene()
{
  QQuickItem::update();
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
  //render_window_ = render_system_->makeRenderWindow(win_id, static_cast<quint32>(width()), static_cast<quint32>(height()), pixel_ratio);
  render_window_ = render_system_->makeRenderWindow(win_id, 1, 1, pixel_ratio);
  render_window_->setVisible(true);
  render_window_->setAutoUpdated(true);

  OgreViewportSupport::initialize();

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
  ROS_INFO("render");
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

void QtQuickOgreRenderWindow::onWindowChanged(QQuickWindow *window)
{
  if (!window) {
    return;
  }

  // start Ogre once we are in the rendering thread (Ogre must live in the rendering thread)
  connect(window, &QQuickWindow::beforeRendering,
          this, &QtQuickOgreRenderWindow::initializeOgre, Qt::DirectConnection);

  QTimer::singleShot(16, this, &QQuickItem::update);
}


} // namespace rviz
