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
#ifndef FAILED_TOOL_H
#define FAILED_TOOL_H

#include "tool.h"

namespace rviz
{
/** @brief A FailedTool instance represents a Tool class we
 * tried and failed to instantiate.
 *
 * FailedTool stores the class id which it was supposed to be, and
 * an error message describing the failure.
 *
 * The load() and save() functions work together to ensure that loaded
 * configuration data is saved out again without modification.  This
 * ensures that running rviz with a missing plugin library won't
 * damage config files which refer to it. */
class FailedTool : public Tool
{
public:
  FailedTool(const QString& desired_class_id, const QString& error_message);

  virtual QString getDescription() const;

  void activate() override;
  void deactivate() override
  {
  }

  int processMouseEvent(ViewportMouseEvent& /*event*/) override
  {
    return 0;
  }

  /** @brief Store the given config data for later, so we can return it
   * with save() when someone writes this back to a file. */
  void load(const Config& config) override;

  /** @brief Copy saved config data from last call to load() into config. */
  void save(Config config) const override;

private:
  Config saved_config_;
  QString error_message_;
};

} // end namespace rviz

#endif // FAILED_TOOL_H
