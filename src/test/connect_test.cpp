/*
 * Copyright (c) 2011, Willow Garage, Inc.
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

#include <stdio.h>
#include <chrono>

// This is a simple speed test to compare suppressing a signal/slot
// emission by one of two methods:
//
// 1) disconnect and reconnect the signal/slot.
//
// 2) define a bool suppress_ member which makes the slot do nothing.
//
// Results for Qt 4.6 are that disconnect/reconnect takes about 11
// times as long as boolean suppressor.

#include "connect_test.h"

int main(int /*argc*/, char** /*argv*/)
{
  MyObject* obj = new MyObject;
  obj->enableChanges();

  QObject::connect(obj, SIGNAL(changed()), obj, SLOT(onChanged()));
  obj->emitChanged();

  int count = 1000000;

  auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < count; i++)
  {
    QObject::disconnect(obj, SIGNAL(changed()), obj, SLOT(onChanged()));
    obj->emitChanged();
    QObject::connect(obj, SIGNAL(changed()), obj, SLOT(onChanged()));
  }
  auto end = std::chrono::steady_clock::now();
  printf("disconnect/emit/connect %d times took %lf seconds.\n", count,
         std::chrono::duration<double>(end - start).count());

  obj->emitChanged();

  start = std::chrono::steady_clock::now();
  for (int i = 0; i < count; i++)
  {
    obj->suppressChanges();
    obj->emitChanged();
    obj->enableChanges();
  }
  end = std::chrono::steady_clock::now();
  printf("suppress/emit/enable %d times took %lf seconds.\n", count,
         std::chrono::duration<double>(end - start).count());

  return 0;
}
