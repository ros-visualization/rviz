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
#ifndef INTERACTIVE_OBJECT_H
#define INTERACTIVE_OBJECT_H

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#include <QCursor>

namespace rviz
{
class ViewportMouseEvent;

/** @brief Abstract base class of things in the scene which handle mouse events.
 *
 * Currently (visualization-1.8) this is only needed as a bridge
 * between interactive markers in the default plugin and the
 * interaction tool in the main executable.  Once the interaction tool
 * is plugin-ized and put into the default plugin, this can probably
 * be removed. */
class InteractiveObject
{
public:
  virtual ~InteractiveObject(){};
  virtual bool isInteractive() = 0;
  virtual void enableInteraction(bool enable) = 0;
  virtual void handleMouseEvent(ViewportMouseEvent& event) = 0;
  virtual const QCursor& getCursor() const = 0;
};

typedef boost::shared_ptr<InteractiveObject> InteractiveObjectPtr;
typedef boost::weak_ptr<InteractiveObject> InteractiveObjectWPtr;

} // end namespace rviz

#endif // INTERACTIVE_OBJECT_H
