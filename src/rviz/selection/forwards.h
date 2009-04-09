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

#ifndef RVIZ_SELECTION_FORWARDS_H
#define RVIZ_SELECTION_FORWARDS_H

#include <vector>
#include <set>
#include <map>
#include <boost/unordered_map.hpp>

namespace rviz
{

typedef uint32_t CollObjectHandle;
typedef std::vector<CollObjectHandle> V_CollObject;
typedef std::vector<V_CollObject> VV_CollObject;
typedef std::set<CollObjectHandle> S_CollObject;

struct Pixel
{
  Pixel()
  {

  }

  Pixel(int _x, int _y, uint32_t _handle)
  : x(_x)
  , y(_y)
  , handle(_handle)
  {}

  bool operator<(const Pixel& rhs)
  {
    if (x != rhs.x)
    {
      return x < rhs.x;
    }

    if (y != rhs.y)
    {
      return y < rhs.y;
    }

    return handle < rhs.handle;
  }

  int x;
  int y;

  uint32_t handle;
};
typedef std::vector<Pixel> V_Pixel;
typedef std::map<std::pair<int, int>, V_Pixel> MV_Pixel;

typedef std::set<uint64_t> S_uint64;
typedef std::vector<uint64_t> V_uint64;

struct Picked
{
  Picked(CollObjectHandle _handle)
  : handle(_handle)
  {

  }

  CollObjectHandle handle;

  S_uint64 extra_handles;
};
typedef boost::unordered_map<CollObjectHandle, Picked> M_Picked;

}

#endif
