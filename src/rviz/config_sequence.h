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
#ifndef CONFIG_SEQUENCE_H
#define CONFIG_SEQUENCE_H

#include "rviz/config.h"

namespace rviz
{

/** @brief ConfigSequence is both an iterator and accessor API for
 * sequences of Config objects. */
class ConfigSequence
{
public:
  /** @brief If not at the end of the sequence, returns the next one
   * and advances the iterator.  If at the end, creates and appends
   * a new item and returns it. */
  Config makeNext();

  /** @brief Advance the iterator and return the next Config.  If at
   * the end of the sequence, returns an invalid Config. */
  Config getNext();

  /** @brief Resets the iterator to the start of the sequence. */
  void start();

  /** @brief Returns true if the iterator is not at the end of the
   * sequence, false if it is at the end. */
  bool hasNext();

private:
  ConfigSequence(); // private constructor to avoid people making these separate from their owner.
  Config::NodePtr node_;
  int next_child_num_;
  friend class Config;
};

} // end namespace rviz

#endif // CONFIG_SEQUENCE_H
