/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include "rviz/splash_screen.h"
#include "rviz/load_resource.h"
#include "env_config.h"

#include <QPainter>
#include <QPoint>

#include <iostream>
#include <QCoreApplication>

namespace rviz
{
SplashScreen::SplashScreen(const QPixmap& pixmap) : QSplashScreen()
{
  const int bottom_border = 27;
  QPixmap splash(pixmap.width(), pixmap.height() + bottom_border);
  splash.fill(QColor(0, 0, 0));

  QPainter painter(&splash);

  painter.drawPixmap(QPoint(0, 0), pixmap);

  QPixmap overlay = loadPixmap("package://rviz/images/splash_overlay.png");
  painter.drawTiledPixmap(QRect(0, pixmap.height() - overlay.height(), pixmap.width(), pixmap.height()),
                          overlay);

  // draw version info
  QString version_info = "r" + QString(get_version().c_str());
  version_info += " (" + QString(get_distro().c_str()) + ")";

  painter.setPen(QColor(160, 160, 160));
  QRect r = splash.rect();
  r.setRect(r.x() + 5, r.y() + 5, r.width() - 10, r.height() - 10);
  painter.drawText(r, Qt::AlignRight | Qt::AlignBottom, version_info);

  setPixmap(splash);
}

void SplashScreen::showMessage(const QString& message)
{
  QSplashScreen::showMessage(message, Qt::AlignLeft | Qt::AlignBottom, Qt::white);
}

} // end namespace rviz
