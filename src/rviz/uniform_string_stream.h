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
#ifndef UNIFORM_STRING_STREAM_H
#define UNIFORM_STRING_STREAM_H

#include <sstream>

namespace rviz
{
/** @brief std::stringstream subclass which defaults to the "C" locale,
 * so serialization of numbers is uniform across locales.
 *
 * For reading floats in, use parseFloat() instead of operator>>,
 * because operator>> is the one from std::stringstream which only
 * handles "C" style floats.  parseFloat() handles "C" and also
 * European-style floats which use the ",", like "1,2" parses to
 * 1.2f */
class UniformStringStream : public std::stringstream
{
public:
  UniformStringStream();
  UniformStringStream(const std::string& str);

  /** @brief Parse a float, supporting both period- and comma- style
   * floats (1,2 and 1.2).
   *
   * Uses operator>>(std::string&) internally, so consumes up to next
   * whitespace from the stream. */
  void parseFloat(float& f);
};

} // end namespace rviz

#endif // UNIFORM_STRING_STREAM_H
