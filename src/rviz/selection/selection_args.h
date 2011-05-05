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


#ifndef RVIZ_SELECTION_ARGS_H
#define RVIZ_SELECTION_ARGS_H

#include "forwards.h"

#include <boost/signals.hpp>


namespace rviz
{

struct SelectionSettingArgs
{
  SelectionSettingArgs()
  {}
};
typedef boost::signal<void (const SelectionSettingArgs&)> SelectionSettingSignal;

struct SelectionSetArgs
{
  SelectionSetArgs(const M_Picked& old_selection, const M_Picked& new_selection)
  : old_selection_(old_selection)
  , new_selection_(new_selection)
  {}

  const M_Picked& old_selection_;
  const M_Picked& new_selection_;
};
typedef boost::signal<void (const SelectionSetArgs&)> SelectionSetSignal;

struct SelectionAddedArgs
{
  SelectionAddedArgs(const M_Picked& added)
  : added_(added)
  {}

  const M_Picked& added_;
};
typedef boost::signal<void (const SelectionAddedArgs&)> SelectionAddedSignal;

struct SelectionRemovedArgs
{
  SelectionRemovedArgs(const M_Picked& removed)
  : removed_(removed)
  {}

  const M_Picked& removed_;
};
typedef boost::signal<void (const SelectionRemovedArgs&)> SelectionRemovedSignal;

}

#endif
