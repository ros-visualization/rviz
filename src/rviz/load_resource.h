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

#ifndef RESOURCE_RETRIEVING_H_
#define RESOURCE_RETRIEVING_H_

#include <QPixmap>
#include <QCursor>
#include <QString>

namespace rviz
{
// Helper functions to load resources based on their resource url,
// e.g. "package://rviz/icons/package.png",
// or "file:///home/user/.ros/config.yaml".

/* @brief Try to load the pixmap url from disk or the cache.
 *        In case of a failure, the result will be an empty QPixmap.
 *        If fill_cache is set to true (default), the image will be
 *        stored in the cache after loading it from disk.
 */
QPixmap loadPixmap(QString url, bool fill_cache = true);

/* @brief Load the default cursor: an arrow.
 *        The fill_cache parameter is ignored.
 */
QCursor getDefaultCursor(bool fill_cache = true);

/* @brief Create a cursor using a shape in a file/url.
 *        In case of a failure, the result will be the default arrow cursor.
 *        If fill_cache is set to true (default), the image will be
 *        stored in the cache after loading it from disk.
 */
QCursor makeIconCursor(QString icon_url, bool fill_cache = true);

/* @brief Create a cursor using the shape in the icon QPixmap.
 *        If fill_cache is set to true (default), the image will be
 *        stored in the cache using \e cache_key.
 */
QCursor makeIconCursor(QPixmap icon, QString cache_key = "", bool fill_cache = true);


} // namespace rviz


#endif /* RESOURCE_RETRIEVING_H_ */
