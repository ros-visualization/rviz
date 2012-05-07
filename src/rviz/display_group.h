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
#ifndef DISPLAY_GROUP_H
#define DISPLAY_GROUP_H

#include "display.h"

namespace rviz
{

class DisplayFactory;

class DisplayGroup: public Display
{
Q_OBJECT
public:
  DisplayGroup();

  /** @brief Return data appropriate for the given column (0 or 1) and
   * role for this DisplayGroup.
   */
  virtual QVariant getViewData( int column, int role ) const;

  /** @brief Return item flags appropriate for the given column (0 or
   * 1) for this DisplayGroup. */
  virtual Qt::ItemFlags getViewFlags( int column ) const;

  /** @brief Load subproperties and the list of displays in this group
   * from the given YAML node, which must be a map. */
  virtual void load( const YAML::Node& yaml_node );

  /** @brief Load just the list of displays from a map with key
   * "Displays". If "Displays" key not present, does nothing. */
  virtual void loadDisplays( const YAML::Node& yaml_node );

  /** @brief Save subproperties and the list of displays in this group
   * to the given YAML emitter, which must be in a map context. */
  virtual void save( YAML::Emitter& emitter );

  /** @brief Save just the list of displays in a map with key
   * "Displays".  Requires emitter to be in a map context. */
  virtual void saveDisplays( YAML::Emitter& emitter );

  /** @brief Remove and destruct all child Displays. */
  virtual void clear();

  /** @brief Return the index-th Display in this group, or NULL if the
   * index is invalid. */
  virtual Display* getDisplayAt( int index ) const;

  /** @brief Call update() on all child Displays. */
  virtual void update( float wall_dt, float ros_dt );

  /** @brief Reset this and all child Displays. */
  virtual void reset();

protected:
  /** @brief Update the fixed frame in all contained displays. */
  virtual void fixedFrameChanged();
};

} // end namespace rviz

#endif // DISPLAY_GROUP_H
