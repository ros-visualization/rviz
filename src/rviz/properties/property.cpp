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

#include "property.h"
#include "ros_topic_property.h"

#include <wx/wx.h>
#include <wx/propgrid/propgrid.h>
#include <wx/propgrid/advprops.h>
#include <wx/config.h>

namespace rviz
{

static const wxColour ERROR_COLOR(178, 23, 46);
static const wxColour WARN_COLOR(222, 213, 17);

wxPGProperty* getCategoryPGProperty(const CategoryPropertyWPtr& wprop)
{
  CategoryPropertyPtr prop = wprop.lock();

  if (prop)
  {
    return prop->getPGProperty();
  }

  return NULL;
}

void setPropertyHelpText(wxPGProperty* property, const std::string& text)
{
  if (property)
  {
    property->SetHelpString(wxString::FromAscii(text.c_str()));
  }
}

void setPropertyToColors(wxPGProperty* property, const wxColour& fg_color, const wxColour& bg_color, uint32_t column)
{
  if (!property)
  {
    return;
  }

  wxPGCell* cell = property->GetCell( column );
  if ( !cell )
  {
    cell = new wxPGCell( *(wxString*)0, wxNullBitmap, *wxLIGHT_GREY, *wxGREEN );
    property->SetCell( column, cell );
  }

  cell->SetFgCol(fg_color);
  cell->SetBgCol(bg_color);
}

void setPropertyToError(wxPGProperty* property, uint32_t column)
{
  setPropertyToColors(property, *wxWHITE, ERROR_COLOR, column);
}

void setPropertyToWarn(wxPGProperty* property, uint32_t column)
{
  setPropertyToColors(property, *wxWHITE, WARN_COLOR, column);
}

void setPropertyToOK(wxPGProperty* property, uint32_t column)
{
  setPropertyToColors(property, wxNullColour, wxNullColour, column);
}

void setPropertyToDisabled(wxPGProperty* property, uint32_t column)
{
  setPropertyToColors(property, wxColour(0x33, 0x44, 0x44), wxColour(0xaa, 0xaa, 0xaa), column);
}

PropertyBase::PropertyBase()
: grid_(NULL)
, property_(NULL)
{

}

PropertyBase::~PropertyBase()
{
  if (property_ && grid_)
  {
    grid_->DeleteProperty( property_ );
  }
}

void PropertyBase::setPropertyGrid(wxPropertyGrid* grid)
{
  grid_ = grid;
}

void PropertyBase::setPGClientData()
{
  if (property_)
  {
    property_->SetClientData( this );
  }
}

void PropertyBase::hide()
{
  if (property_)
  {
    property_->Hide(true);
  }
}

void PropertyBase::show()
{
  if (property_)
  {
    property_->Hide(false);
  }
}

bool PropertyBase::isSelected()
{
  if (property_ && grid_)
  {
    return grid_->GetSelectedProperty() == property_;
  }

  return false;
}

StatusProperty::StatusProperty(const std::string& name, const std::string& prefix, const CategoryPropertyWPtr& parent, void* user_data)
: name_(wxString::FromAscii(name.c_str()))
, prefix_(wxString::FromAscii(prefix.c_str()))
, parent_(parent)
, user_data_(user_data)
, top_property_(0)
, enabled_(true)
, top_status_(status_levels::Ok)
{}

StatusProperty::~StatusProperty()
{
  if (grid_)
  {
    if (top_property_)
    {
      grid_->DeleteProperty(top_property_);
    }
  }
}

void StatusProperty::enable()
{
  boost::mutex::scoped_lock lock(status_mutex_);
  enabled_ = true;

  changed();
}

void StatusProperty::disable()
{
  clear();

  boost::mutex::scoped_lock lock(status_mutex_);
  enabled_ = false;

  changed();
}

void StatusProperty::clear()
{
  boost::mutex::scoped_lock lock(status_mutex_);

  if (!enabled_)
  {
    return;
  }

  M_StringToStatus::iterator it = statuses_.begin();
  M_StringToStatus::iterator end = statuses_.end();
  for (; it != end; ++it)
  {
    Status& status = it->second;
    status.kill = true;
  }

  changed();
}

void StatusProperty::setStatus(StatusLevel level, const std::string& name, const std::string& text)
{
  boost::mutex::scoped_lock lock(status_mutex_);

  if (!enabled_)
  {
    return;
  }

  Status& status = statuses_[name];
  wxString wx_name = wxString::FromAscii(name.c_str());
  wxString wx_text = wxString::FromAscii(text.c_str());

  // Status hasn't changed, return
  if (status.level == level && status.text == wx_text && !status.kill)
  {
    return;
  }

  status.name = wx_name;
  status.text = wx_text;
  status.level = level;
  status.kill = false;

  changed();
}

void StatusProperty::deleteStatus(const std::string& name)
{
  boost::mutex::scoped_lock lock(status_mutex_);

  if (!enabled_)
  {
    return;
  }

  M_StringToStatus::iterator it = statuses_.find(name);
  if (it != statuses_.end())
  {
    Status& status = it->second;
    status.kill = true;
  }
}

void StatusProperty::writeToGrid()
{
  boost::mutex::scoped_lock lock(status_mutex_);

  if ( !top_property_ )
  {
    wxString top_name = name_ + wxT("TopStatus");

    if ( parent_.lock() )
    {
      top_property_ = grid_->AppendIn( getCategoryPGProperty(parent_), new wxPropertyCategory(name_, prefix_ + top_name) );
    }
    else
    {
      top_property_ = grid_->AppendIn( grid_->GetRoot(), new wxPropertyCategory(name_, prefix_ + top_name));
    }

    top_property_->SetClientData( this );

    grid_->DisableProperty(top_property_);
    grid_->Collapse(top_property_);
  }

  bool expanded = top_property_->IsExpanded();

  top_status_ = status_levels::Ok;

  std::vector<std::string> to_erase;
  M_StringToStatus::iterator it = statuses_.begin();
  M_StringToStatus::iterator end = statuses_.end();
  for (; it != end; ++it)
  {
    Status& status = it->second;

    if (status.kill)
    {
      to_erase.push_back(it->first);
      continue;
    }

    if (!status.property)
    {
      status.property = grid_->AppendIn(top_property_, new wxStringProperty(status.name, prefix_ + name_ + status.name, status.text) );
    }

    if (status.level > top_status_)
    {
      top_status_ = status.level;
    }

    if (enabled_)
    {
      switch (status.level)
      {
      case status_levels::Ok:
        setPropertyToOK(status.property);
        break;
      case status_levels::Warn:
        setPropertyToColors(status.property, WARN_COLOR, *wxWHITE);
        //setPropertyToWarn(status.property);
        break;
      case status_levels::Error:
        setPropertyToColors(status.property, ERROR_COLOR, *wxWHITE);
        //setPropertyToError(status.property);
        break;
      }
    }
    else
    {
      setPropertyToDisabled(status.property);
    }

    grid_->SetPropertyValue(status.property, status.text);
  }

  std::vector<std::string>::iterator kill_it = to_erase.begin();
  std::vector<std::string>::iterator kill_end = to_erase.end();
  for (; kill_it != kill_end; ++kill_it)
  {
    Status& status = statuses_[*kill_it];
    if (status.property)
    {
      grid_->DeleteProperty(status.property);
    }

    statuses_.erase(*kill_it);
  }

  if (!expanded)
  {
    grid_->Collapse(top_property_);
  }

  wxString label;
  if (enabled_)
  {
    switch (top_status_)
    {
    case status_levels::Ok:
      setPropertyToColors(top_property_, *wxBLACK, *wxWHITE);
      //setPropertyToOK(top_property_);
      label = name_ + wxT(": OK");
      break;
    case status_levels::Warn:
      setPropertyToColors(top_property_, WARN_COLOR, *wxWHITE);
      //setPropertyToWarn(top_property_);
      label = name_ + wxT(": Warning");
      break;
    case status_levels::Error:
      setPropertyToColors(top_property_, ERROR_COLOR, *wxWHITE);
      //setPropertyToError(top_property_);
      label = name_ + wxT(": Error");
      break;
    }
  }
  else
  {
    setPropertyToDisabled(top_property_);
    label = name_ + wxT(": Disabled");
  }

  grid_->SetPropertyLabel(top_property_, label);
  wxPGCell* cell = top_property_->GetCell( 0 );
  if ( cell )
  {
    //cell->SetText(label);
  }

  grid_->Sort(top_property_);
}

StatusLevel StatusProperty::getTopLevelStatus()
{
  return top_status_;
}

void StatusProperty::setPGClientData()
{
  if (top_property_)
  {
    top_property_->SetClientData( this );
  }
}

void BoolProperty::writeToGrid()
{
  if ( !property_ )
  {
    property_ = grid_->AppendIn( getCategoryPGProperty(parent_), new wxBoolProperty( name_, prefix_ + name_, get() ) );
    property_->SetAttribute( wxPG_BOOL_USE_CHECKBOX, true );

    if ( !hasSetter() )
    {
      grid_->DisableProperty( property_ );
    }
  }
  else
  {
    grid_->SetPropertyValue(property_, get());
  }
}

void BoolProperty::readFromGrid()
{
  wxVariant var = property_->GetValue();
  set( var.GetBool() );
}

void BoolProperty::saveToConfig( wxConfigBase* config )
{
  config->Write( prefix_ + name_, (int)get() );
}

void BoolProperty::loadFromConfig( wxConfigBase* config )
{
  bool val;
  if (!config->Read( prefix_ + name_, &val, get() ))
  {
    V_wxString::iterator it = legacy_names_.begin();
    V_wxString::iterator end = legacy_names_.end();
    for (; it != end; ++it)
    {
      if (config->Read( prefix_ + *it, &val, get() ))
      {
        break;
      }
    }
  }

  set( val );
}

void IntProperty::setMin( int min )
{
  if (property_)
  {
    property_->SetAttribute( wxT("Min"), min );
  }
}

void IntProperty::setMax( int max )
{
  if (property_)
  {
    property_->SetAttribute( wxT("Max"), max );
  }
}

void IntProperty::writeToGrid()
{
  if ( !property_ )
  {
    property_ = grid_->AppendIn( getCategoryPGProperty(parent_), new wxIntProperty( name_, prefix_ + name_, get() ) );

    if ( !hasSetter() )
    {
      grid_->DisableProperty( property_ );
    }
  }
  else
  {
    grid_->SetPropertyValue(property_, (long)get());
  }
}

void IntProperty::readFromGrid()
{
  wxVariant var = property_->GetValue();
  set( var.GetLong() );
}

void IntProperty::saveToConfig( wxConfigBase* config )
{
  config->Write( prefix_ + name_, (int)get() );
}

void IntProperty::loadFromConfig( wxConfigBase* config )
{
  long val;
  if (!config->Read( prefix_ + name_, &val, get() ))
  {
    V_wxString::iterator it = legacy_names_.begin();
    V_wxString::iterator end = legacy_names_.end();
    for (; it != end; ++it)
    {
      if (config->Read( prefix_ + *it, &val, get() ))
      {
        break;
      }
    }
  }

  set( val );
}

void FloatProperty::setMin( float min )
{
  if (property_)
  {
    property_->SetAttribute( wxT("Min"), min );
  }
}

void FloatProperty::setMax( float max )
{
  if (property_)
  {
    property_->SetAttribute( wxT("Max"), max );
  }
}


void FloatProperty::writeToGrid()
{
  if ( !property_ )
  {
    property_ = grid_->AppendIn( getCategoryPGProperty(parent_), new wxFloatProperty( name_, prefix_ + name_, get() ) );

    if ( !hasSetter() )
    {
      grid_->DisableProperty( property_ );
    }
  }
  else
  {
    grid_->SetPropertyValue(property_, (double)get());
  }
}

void FloatProperty::readFromGrid()
{
  wxVariant var = property_->GetValue();
  set( var.GetDouble() );
}

void FloatProperty::saveToConfig( wxConfigBase* config )
{
  config->Write( prefix_ + name_, (float)get() );
}

void FloatProperty::loadFromConfig( wxConfigBase* config )
{
  double val;
  if (!config->Read( prefix_ + name_, &val, get() ))
  {
    V_wxString::iterator it = legacy_names_.begin();
    V_wxString::iterator end = legacy_names_.end();
    for (; it != end; ++it)
    {
      if (config->Read( prefix_ + *it, &val, get() ))
      {
        break;
      }
    }
  }

  set( val );
}

void DoubleProperty::setMin( double min )
{
  if (property_)
  {
    property_->SetAttribute( wxT("Min"), min );
  }
}

void DoubleProperty::setMax( double max )
{
  if (property_)
  {
    property_->SetAttribute( wxT("Max"), max );
  }
}


void DoubleProperty::writeToGrid()
{
  if ( !property_ )
  {
    property_ = grid_->AppendIn( getCategoryPGProperty(parent_), new wxFloatProperty( name_, prefix_ + name_, get() ) );

    if ( !hasSetter() )
    {
      grid_->DisableProperty( property_ );
    }
  }
  else
  {
    grid_->SetPropertyValue(property_, (double)get());
  }
}

void DoubleProperty::readFromGrid()
{
  wxVariant var = property_->GetValue();
  set( var.GetDouble() );
}

void DoubleProperty::saveToConfig( wxConfigBase* config )
{
  config->Write( prefix_ + name_, (float)get() );
}

void DoubleProperty::loadFromConfig( wxConfigBase* config )
{
  double val;
  if (!config->Read( prefix_ + name_, &val, get() ))
  {
    V_wxString::iterator it = legacy_names_.begin();
    V_wxString::iterator end = legacy_names_.end();
    for (; it != end; ++it)
    {
      if (config->Read( prefix_ + *it, &val, get() ))
      {
        break;
      }
    }
  }

  set( val );
}

void StringProperty::writeToGrid()
{
  if ( !property_ )
  {
    property_ = grid_->AppendIn( getCategoryPGProperty(parent_), new wxStringProperty( name_, prefix_ + name_, wxString::FromAscii( get().c_str() ) ) );

    if ( !hasSetter() )
    {
      grid_->DisableProperty( property_ );
    }
  }
  else
  {
    grid_->SetPropertyValue(property_, wxString::FromAscii( get().c_str() ));
  }
}

void StringProperty::readFromGrid()
{
  wxVariant var = property_->GetValue();
  set( (const char*)var.GetString().mb_str() );
}

void StringProperty::saveToConfig( wxConfigBase* config )
{
  config->Write( prefix_ + name_, wxString::FromAscii( get().c_str() ) );
}

void StringProperty::loadFromConfig( wxConfigBase* config )
{
  wxString val;
  if (!config->Read( prefix_ + name_, &val, wxString::FromAscii( get().c_str() ) ))
  {
    V_wxString::iterator it = legacy_names_.begin();
    V_wxString::iterator end = legacy_names_.end();
    for (; it != end; ++it)
    {
      if (config->Read( prefix_ + *it, &val, wxString::FromAscii( get().c_str() ) ))
      {
        break;
      }
    }
  }

  set( (const char*)val.mb_str() );
}

void ROSTopicStringProperty::setMessageType(const std::string& message_type)
{
  message_type_ = message_type;
  ros_topic_property_->setMessageType(message_type);
}

void ROSTopicStringProperty::writeToGrid()
{
  if ( !property_ )
  {
    ros_topic_property_ = new ROSTopicProperty( message_type_, name_, prefix_ + name_, wxString::FromAscii( get().c_str() ) );
    property_ = grid_->AppendIn( getCategoryPGProperty(parent_), ros_topic_property_ );

    if ( !hasSetter() )
    {
      grid_->DisableProperty( property_ );
    }
  }
  else
  {
    grid_->SetPropertyValue(property_, wxString::FromAscii( get().c_str() ));
  }
}

void ColorProperty::writeToGrid()
{
  if ( !property_ )
  {
    Color c = get();
    property_ = grid_->AppendIn( getCategoryPGProperty(parent_), new wxColourProperty( name_, prefix_ + name_, wxColour( c.r_ * 255, c.g_ * 255, c.b_ * 255 ) ) );

    if ( !hasSetter() )
    {
      grid_->DisableProperty( property_ );
    }
  }
  else
  {
    Color c = get();
    wxVariant var;
    var << wxColour( c.r_ * 255, c.g_ * 255, c.b_ * 255 );
    grid_->SetPropertyValue(property_, var);
  }
}

void ColorProperty::readFromGrid()
{
  wxVariant var = property_->GetValue();
  wxColour col;
  col << var;
  set( Color( col.Red() / 255.0f, col.Green() / 255.0f, col.Blue() / 255.0f ) );
}

void ColorProperty::saveToConfig( wxConfigBase* config )
{
  Color c = get();

  config->Write( prefix_ + name_ + wxT("R"), c.r_ );
  config->Write( prefix_ + name_ + wxT("G"), c.g_ );
  config->Write( prefix_ + name_ + wxT("B"), c.b_ );
}

void ColorProperty::loadFromConfig( wxConfigBase* config )
{
  Color c = get();
  double r, g, b;
  bool found = true;
  found &= config->Read( prefix_ + name_ + wxT("R"), &r, c.r_ );
  found &= config->Read( prefix_ + name_ + wxT("G"), &g, c.g_ );
  found &= config->Read( prefix_ + name_ + wxT("B"), &b, c.b_ );

  if (!found)
  {
    V_wxString::iterator it = legacy_names_.begin();
    V_wxString::iterator end = legacy_names_.end();
    for (; it != end; ++it)
    {
      found = true;
      found &= config->Read( prefix_ + *it + wxT("R"), &r, c.r_ );
      found &= config->Read( prefix_ + *it + wxT("G"), &g, c.g_ );
      found &= config->Read( prefix_ + *it + wxT("B"), &b, c.b_ );

      if (found)
      {
        break;
      }
    }
  }

  set( Color( r, g, b ) );
}

EnumProperty::EnumProperty( const std::string& name, const std::string& prefix, const CategoryPropertyWPtr& parent, const Getter& getter, const Setter& setter )
: Property<int>( name, prefix, parent, getter, setter )
, choices_(new wxPGChoices)
{
}

void EnumProperty::addOption( const std::string& name, int value )
{
  choices_->Add(wxString::FromAscii( name.c_str() ), value);
  changed();
}

void EnumProperty::clear ()
{
  choices_->Clear();
  changed();
}

void EnumProperty::writeToGrid()
{
  if (isSelected())
  {
    changed();
    return;
  }

  if ( !property_ )
  {
    property_ = grid_->AppendIn( getCategoryPGProperty(parent_), new wxEnumProperty( name_, prefix_ + name_ ) );
    grid_->SetPropertyChoices(property_, *choices_);

    if ( !hasSetter() )
    {
      grid_->DisableProperty( property_ );
    }
  }
  else
  {
    grid_->SetPropertyChoices(property_, *choices_);
    grid_->SetPropertyValue(property_, (long)get());
  }
}

void EnumProperty::readFromGrid()
{
  wxVariant var = property_->GetValue();
  set( var.GetLong() );
}

void EnumProperty::saveToConfig( wxConfigBase* config )
{
  config->Write( prefix_ + name_, (int)get() );
}

void EnumProperty::loadFromConfig( wxConfigBase* config )
{
  long val = 0xffffffff;
  if (!config->Read( prefix_ + name_, &val, get() ))
  {
    V_wxString::iterator it = legacy_names_.begin();
    V_wxString::iterator end = legacy_names_.end();
    for (; it != end; ++it)
    {
      if (config->Read( prefix_ + *it, &val, get() ))
      {
        break;
      }
    }
  }

  set( val );
}

EditEnumProperty::EditEnumProperty( const std::string& name, const std::string& prefix, const CategoryPropertyWPtr& parent, const Getter& getter, const Setter& setter )
: Property<std::string>( name, prefix, parent, getter, setter )
, choices_(new wxPGChoices)
{
}

void EditEnumProperty::addOption( const std::string& name )
{
  choices_->Add(wxString::FromAscii( name.c_str() ));
  changed();
}

void EditEnumProperty::clear ()
{
  choices_->Clear();
  changed();
}

void EditEnumProperty::writeToGrid()
{
  if (isSelected())
  {
    changed();
    return;
  }

  if ( !property_ )
  {
    property_ = grid_->AppendIn( getCategoryPGProperty(parent_), new wxEditEnumProperty( name_, prefix_ + name_ ) );
    grid_->SetPropertyChoices(property_, *choices_);

    if ( !hasSetter() )
    {
      grid_->DisableProperty( property_ );
    }
  }
  else
  {
    grid_->SetPropertyChoices(property_, *choices_);
    grid_->SetPropertyValue(property_, wxString::FromAscii( get().c_str() ));
  }
}

void EditEnumProperty::readFromGrid()
{
  wxString str = property_->GetValueString();
  set( (const char*)str.mb_str() );
}

void EditEnumProperty::saveToConfig( wxConfigBase* config )
{
  config->Write( prefix_ + name_, wxString::FromAscii( get().c_str() ) );
}

void EditEnumProperty::loadFromConfig( wxConfigBase* config )
{
  wxString val;
  if (!config->Read( prefix_ + name_, &val, wxString::FromAscii( get().c_str() ) ))
  {
    V_wxString::iterator it = legacy_names_.begin();
    V_wxString::iterator end = legacy_names_.end();
    for (; it != end; ++it)
    {
      if (config->Read( prefix_ + *it, &val, wxString::FromAscii( get().c_str() ) ))
      {
        break;
      }
    }
  }

  set( (const char*)val.mb_str() );
}

void CategoryProperty::setLabel( const std::string& label )
{
  label_ = wxString::FromAscii(label.c_str());

  if (grid_)
  {
    grid_->SetPropertyLabel( property_, wxString::FromAscii( label.c_str() ) );

    wxPGCell* cell = property_->GetCell( 0 );
    if ( cell )
    {
      //cell->SetText( wxString::FromAscii( label.c_str() ) );
    }
  }
}

void CategoryProperty::expand()
{
  if (property_)
  {
    grid_->Expand( property_ );
  }
}

void CategoryProperty::collapse()
{
  if (property_)
  {
    grid_->Collapse( property_ );
  }
}

void CategoryProperty::writeToGrid()
{
  if ( !property_ )
  {
    if ( parent_.lock() )
    {
      if (checkbox_)
      {
        property_ = grid_->AppendIn( getCategoryPGProperty(parent_), new wxBoolProperty( label_, prefix_ + name_, get() ) );
        property_->SetAttribute( wxPG_BOOL_USE_CHECKBOX, true );
      }
      else
      {
        property_ = grid_->AppendIn( getCategoryPGProperty(parent_), new wxPropertyCategory( label_, prefix_ + name_ ) );
      }
    }
    else
    {
      if (checkbox_)
      {
        property_ = grid_->AppendIn( grid_->GetRoot(),  new wxBoolProperty( label_, prefix_ + name_, get() ) );
        property_->SetAttribute( wxPG_BOOL_USE_CHECKBOX, true );
      }
      else
      {
        property_ = grid_->AppendIn( grid_->GetRoot(),  new wxPropertyCategory( name_, prefix_ + name_ ) );
      }
    }
  }
  else
  {
    if (checkbox_)
    {
      grid_->SetPropertyValue(property_, get());
    }
  }
}

void CategoryProperty::readFromGrid()
{
  if (checkbox_)
  {
    wxVariant var = property_->GetValue();
    set( var.GetBool() );
  }
}

void CategoryProperty::saveToConfig( wxConfigBase* config )
{
  if (checkbox_)
  {
    config->Write( prefix_ + name_, (int)get() );
  }
}

void CategoryProperty::loadFromConfig( wxConfigBase* config )
{
  if (checkbox_)
  {
    bool val;
    if (!config->Read( prefix_ + name_, &val, get() ))
    {
      V_wxString::iterator it = legacy_names_.begin();
      V_wxString::iterator end = legacy_names_.end();
      for (; it != end; ++it)
      {
        if (config->Read( prefix_ + *it, &val, get() ))
        {
          break;
        }
      }
    }

    set( val );
  }
}

void CategoryProperty::setToError()
{
  Property<bool>::setToError();

  if (checkbox_)
  {
    //setPropertyToError(property_, 1);
  }
}

void CategoryProperty::setToWarn()
{
  Property<bool>::setToWarn();

  if (checkbox_)
  {
    //setPropertyToWarn(property_, 1);
  }
}

void CategoryProperty::setToOK()
{
  if (grid_)
  {
    setPropertyToColors(property_, grid_->GetCaptionForegroundColour(), grid_->GetCaptionBackgroundColour(), 0);
    wxPGCell* cell = property_->GetCell(0);
    wxFont font = grid_->GetFont();
    font.SetWeight(wxBOLD);
    cell->SetFont(font);
    //setPropertyToColors(property_, grid_->GetCaptionForegroundColour(), grid_->GetCaptionBackgroundColour(), 1);
  }
}

void CategoryProperty::setToDisabled()
{
  Property<bool>::setToDisabled();
  if (checkbox_)
  {
    //setPropertyToDisabled(property_, 1);
  }
}

Vector3Property::~Vector3Property()
{
  if (composed_parent_)
  {
    grid_->DeleteProperty( composed_parent_ );
  }
}

void Vector3Property::writeToGrid()
{
  if ( !composed_parent_ )
  {
    Ogre::Vector3 v = get();

    wxString composed_name = name_ + wxT("Composed");
    composed_parent_ = grid_->AppendIn( getCategoryPGProperty(parent_), new wxStringProperty( name_, prefix_ + composed_name, wxT("<composed>")) );
    composed_parent_->SetClientData( this );

    x_ = grid_->AppendIn( composed_parent_, new wxFloatProperty( wxT("X"), prefix_ + name_ + wxT("X"), v.x ) );
    y_ = grid_->AppendIn( composed_parent_, new wxFloatProperty( wxT("Y"), prefix_ + name_ + wxT("Y"), v.y ) );
    z_ = grid_->AppendIn( composed_parent_, new wxFloatProperty( wxT("Z"), prefix_ + name_ + wxT("Z"), v.z ) );

    if ( !hasSetter() )
    {
      grid_->DisableProperty( composed_parent_ );
      grid_->DisableProperty( x_ );
      grid_->DisableProperty( y_ );
      grid_->DisableProperty( z_ );
    }

    grid_->Collapse( composed_parent_ );
  }
  else
  {
    Ogre::Vector3 v = get();
    grid_->SetPropertyValue(x_, v.x);
    grid_->SetPropertyValue(y_, v.y);
    grid_->SetPropertyValue(z_, v.z);
  }
}

void Vector3Property::readFromGrid()
{
  set( Ogre::Vector3( x_->GetValue().GetDouble(), y_->GetValue().GetDouble(), z_->GetValue().GetDouble() ) );
}

void Vector3Property::saveToConfig( wxConfigBase* config )
{
  Ogre::Vector3 v = get();

  config->Write( prefix_ + name_ + wxT("X"), v.x );
  config->Write( prefix_ + name_ + wxT("Y"), v.y );
  config->Write( prefix_ + name_ + wxT("Z"), v.z );
}

void Vector3Property::loadFromConfig( wxConfigBase* config )
{
  Ogre::Vector3 v = get();
  double x, y, z;
  bool found = true;
  found &= config->Read( prefix_ + name_ + wxT("X"), &x, v.x );
  found &= config->Read( prefix_ + name_ + wxT("Y"), &y, v.y );
  found &= config->Read( prefix_ + name_ + wxT("Z"), &z, v.z );

  if (!found)
  {
    V_wxString::iterator it = legacy_names_.begin();
    V_wxString::iterator end = legacy_names_.end();
    for (; it != end; ++it)
    {
      found = true;
      found &= config->Read( prefix_ + *it + wxT("X"), &x, v.x );
      found &= config->Read( prefix_ + *it + wxT("Y"), &y, v.y );
      found &= config->Read( prefix_ + *it + wxT("Z"), &z, v.z );

      if (found)
      {
        break;
      }
    }
  }

  set( Ogre::Vector3( x, y, z ) );
}

void Vector3Property::setPGClientData()
{
  if (composed_parent_)
  {
    composed_parent_->SetClientData(this);
    x_->SetClientData( this );
    y_->SetClientData( this );
    z_->SetClientData( this );
  }
}

void Vector3Property::reset()
{
  Property<Ogre::Vector3>::reset();

  composed_parent_ = 0;
  x_ = 0;
  y_ = 0;
  z_ = 0;
}

void Vector3Property::hide()
{
  if (composed_parent_)
  {
    composed_parent_->Hide(true);
  }
}

void Vector3Property::show()
{
  if (composed_parent_)
  {
    composed_parent_->Hide(false);
  }
}

QuaternionProperty::~QuaternionProperty()
{
  if (composed_parent_)
  {
    grid_->DeleteProperty( composed_parent_ );
  }
}

void QuaternionProperty::writeToGrid()
{
  if ( !composed_parent_ )
  {
    Ogre::Quaternion q = get();

    wxString composed_name = name_ + wxT("Composed");
    composed_parent_ = grid_->AppendIn( getCategoryPGProperty(parent_), new wxStringProperty( name_, prefix_ + composed_name, wxT("<composed>")) );
    composed_parent_->SetClientData( this );

    x_ = grid_->AppendIn( composed_parent_, new wxFloatProperty( wxT("X"), prefix_ + name_ + wxT("X"), q.x ) );
    y_ = grid_->AppendIn( composed_parent_, new wxFloatProperty( wxT("Y"), prefix_ + name_ + wxT("Y"), q.y ) );
    z_ = grid_->AppendIn( composed_parent_, new wxFloatProperty( wxT("Z"), prefix_ + name_ + wxT("Z"), q.z ) );
    w_ = grid_->AppendIn( composed_parent_, new wxFloatProperty( wxT("W"), prefix_ + name_ + wxT("W"), q.z ) );

    if ( !hasSetter() )
    {
      grid_->DisableProperty( composed_parent_ );
      grid_->DisableProperty( x_ );
      grid_->DisableProperty( y_ );
      grid_->DisableProperty( z_ );
      grid_->DisableProperty( w_ );
    }

    grid_->Collapse( composed_parent_ );
  }
  else
  {
    Ogre::Quaternion q = get();
    grid_->SetPropertyValue(x_, q.x);
    grid_->SetPropertyValue(y_, q.y);
    grid_->SetPropertyValue(z_, q.z);
    grid_->SetPropertyValue(w_, q.w);
  }
}

void QuaternionProperty::readFromGrid()
{
  set( Ogre::Quaternion( x_->GetValue().GetDouble(), y_->GetValue().GetDouble(), z_->GetValue().GetDouble(), w_->GetValue().GetDouble() ) );
}

void QuaternionProperty::saveToConfig( wxConfigBase* config )
{
  Ogre::Quaternion q = get();

  config->Write( prefix_ + name_ + wxT("X"), q.x );
  config->Write( prefix_ + name_ + wxT("Y"), q.y );
  config->Write( prefix_ + name_ + wxT("Z"), q.z );
  config->Write( prefix_ + name_ + wxT("W"), q.w );
}

void QuaternionProperty::loadFromConfig( wxConfigBase* config )
{
  Ogre::Quaternion q = get();
  double x, y, z, w;
  bool found = true;
  found &= config->Read( prefix_ + name_ + wxT("X"), &x, q.x );
  found &= config->Read( prefix_ + name_ + wxT("Y"), &y, q.y );
  found &= config->Read( prefix_ + name_ + wxT("Z"), &z, q.z );
  found &= config->Read( prefix_ + name_ + wxT("W"), &w, q.w );

  if (!found)
  {
    V_wxString::iterator it = legacy_names_.begin();
    V_wxString::iterator end = legacy_names_.end();
    for (; it != end; ++it)
    {
      found = true;
      found &= config->Read( prefix_ + *it + wxT("X"), &x, q.x );
      found &= config->Read( prefix_ + *it + wxT("Y"), &y, q.y );
      found &= config->Read( prefix_ + *it + wxT("Z"), &z, q.z );
      found &= config->Read( prefix_ + *it + wxT("W"), &w, q.w );

      if (found)
      {
        break;
      }
    }
  }

  set( Ogre::Quaternion( x, y, z, w ) );
}

void QuaternionProperty::setPGClientData()
{
  if (composed_parent_)
  {
    composed_parent_->SetClientData(this);
    x_->SetClientData( this );
    y_->SetClientData( this );
    z_->SetClientData( this );
    w_->SetClientData( this );
  }
}

void QuaternionProperty::reset()
{
  Property<Ogre::Quaternion>::reset();

  composed_parent_ = 0;
  x_ = 0;
  y_ = 0;
  z_ = 0;
  w_ = 0;
}

void QuaternionProperty::hide()
{
  if (composed_parent_)
  {
    composed_parent_->Hide(true);
  }
}

void QuaternionProperty::show()
{
  if (composed_parent_)
  {
    composed_parent_->Hide(false);
  }
}

}
