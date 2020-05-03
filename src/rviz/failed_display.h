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
#ifndef FAILED_DISPLAY_H
#define FAILED_DISPLAY_H

#include "display.h"

namespace rviz
{
/** @brief A FailedDisplay instance represents a Display class we
 * tried and failed to instantiate.
 *
 * FailedDisplay stores the class id which it was supposed to be, and
 * an error message describing the failure.
 *
 * The load() and save() functions work together to ensure that loaded
 * configuration data is saved out again without modification.  This
 * ensures that running rviz with a missing plugin library won't
 * damage config files which refer to it. */
class FailedDisplay : public Display
{
public:
  FailedDisplay(const QString& desired_class_id, const QString& error_message);

  QVariant getViewData(int column, int role) const override;

  QString getDescription() const override;

  /** @brief Store the given Config data for later, so we can return it
   * with save() when someone writes this back to a file. */
  void load(const Config& config) override;

  /** @brief Save Config equivalent to the last which was sent to load().*/
  void save(Config config) const override;

private:
  Config saved_config_;
  QString error_message_;
};

} // end namespace rviz

#endif // FAILED_DISPLAY_H
