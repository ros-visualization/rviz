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
#ifndef CLASS_ID_RECORDING_FACTORY_H
#define CLASS_ID_RECORDING_FACTORY_H

#include <rviz/factory.h>

namespace rviz
{
template <class Type>
/** @brief Templated factory which informs objects created by it what their class identifier string was.
 *
 * Calls a setClassId() function on any instance created by a protected makeRaw() function
 * (pure virtual in this class). */
class ClassIdRecordingFactory : public Factory
{
public:
  /** @brief Instantiate and return a instance of a subclass of Type using makeRaw().
   * @param class_id A string identifying the class uniquely among
   *        classes of its parent class.  rviz::GridDisplay might be
   *        rviz/Grid, for example.
   * @param error_return If non-NULL and there is an error,
   *        error_return is set to a description of the problem.
   * @return A new instance of the class identified by class_id, or NULL if there was an error.
   *
   * If make() returns NULL and error_return is not NULL,
   * *error_return will be set.  On success, *error_return will not be
   * changed. */
  virtual Type* make(const QString& class_id, QString* error_return = nullptr)
  {
    Type* obj = makeRaw(class_id, error_return);
    if (obj != nullptr)
    {
      obj->setClassId(class_id);
      obj->setDescription(getClassDescription(class_id));
    }
    return obj;
  }

protected:
  virtual Type* makeRaw(const QString& class_id, QString* error_return = nullptr) = 0;
};

} // end namespace rviz

#endif // CLASS_ID_RECORDING_FACTORY_H
