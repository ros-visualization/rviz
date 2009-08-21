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

#ifndef RVIZ_NEW_DISPLAY_DIALOG_H
#define RVIZ_NEW_DISPLAY_DIALOG_H

#include "generated/rviz_generated.h"
#include "plugin/plugin.h"

#include <vector>
#include <set>
#include <string>

class wxHtmlLinkEvent;
class wxTreeEvent;
class wxMouseEvent;

namespace rviz
{

typedef std::vector<std::string> V_string;
typedef std::set<std::string> S_string;

class NewDisplayDialog : public NewDisplayDialogGenerated
{
public:
  NewDisplayDialog( wxWindow* parent, const L_Plugin& plugins, const S_string& current_display_names );

  std::string getPackageName();
  std::string getClassName();
  std::string getDisplayName();

protected:
  virtual void onDisplaySelected( wxTreeEvent& event );
  virtual void onDisplayDClick( wxMouseEvent& event );
  virtual void onOK( wxCommandEvent& event );
  virtual void onCancel( wxCommandEvent& event );
  virtual void onNameEnter( wxCommandEvent& event );

  void onLinkClicked(wxHtmlLinkEvent& event);

  int32_t getSelectionIndex();

  struct DisplayTypeInfoWithPlugin
  {
    PluginPtr plugin;
    DisplayTypeInfoPtr typeinfo;
  };
  typedef std::vector<DisplayTypeInfoWithPlugin> V_DisplayTypeInfoWithPlugin;

  V_DisplayTypeInfoWithPlugin typeinfo_;
  const S_string& current_display_names_;
};

} //namespace rviz

#endif
