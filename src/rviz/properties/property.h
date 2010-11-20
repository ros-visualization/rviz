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

#ifndef RVIZ_PROPERTY_H
#define RVIZ_PROPERTY_H

#include "rviz/helpers/color.h"
#include "forwards.h"
#include "rviz/status_level.h"

#include <boost/function.hpp>
#include <boost/signal.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/console.h>
#include <ros/assert.h>

#include <wx/string.h>
#include <wx/colour.h>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>

#include <string>
#include <vector>

class wxConfigBase;
class wxPropertyGrid;
class wxPGProperty;
class wxColour;
class wxPGChoices;

namespace rviz
{

class CategoryProperty;
class VisualizationManager;

void setPropertyHelpText(wxPGProperty* property, const std::string& text);
void setPropertyToColors(wxPGProperty* property, const wxColour& fg_color, const wxColour& bg_color, uint32_t column = 0);
void setPropertyToError(wxPGProperty* property, uint32_t column = 0);
void setPropertyToWarn(wxPGProperty* property, uint32_t column = 0);
void setPropertyToOK(wxPGProperty* property, uint32_t column = 0);
void setPropertyToDisabled(wxPGProperty* property, uint32_t column = 0);
void setPropertyName(wxPGProperty* property, const wxString& name);

/**
 * \brief Abstract base class for properties
 */
class PropertyBase : public boost::enable_shared_from_this<PropertyBase>
{
public:
  typedef boost::signal< void (const PropertyBasePtr&) > ChangedSignal;

  PropertyBase();
  virtual ~PropertyBase();
  virtual void writeToGrid() = 0;
  virtual void readFromGrid() = 0;
  virtual void saveToConfig( wxConfigBase* config ) = 0;
  virtual void loadFromConfig( wxConfigBase* config ) = 0;

  virtual std::string getName() = 0;
  virtual std::string getPrefix() = 0;
  virtual void setPrefix(const std::string& prefix) = 0;
  virtual bool getSave() = 0;

  virtual void* getUserData() = 0;

  virtual wxPGProperty* getPGProperty() = 0;

  virtual CategoryPropertyWPtr getParent() = 0;

  virtual void addLegacyName(const std::string& name) = 0;

  virtual void setPGClientData();
  virtual void setPropertyGrid(wxPropertyGrid* grid);

  virtual void setUserData(void* user_data) = 0;

  virtual void reset()
  {
    grid_ = 0;
    property_ = 0;
  }

  virtual void show();
  virtual void hide();

  virtual bool isSelected();

  /**
   * \brief Add a listener function/method to be called whenever the value in this property has changed.
   * @param slot The function/method to call.  See boost::signals
   */
  void addChangedListener( const ChangedSignal::slot_type& slot )
  {
    changed_.connect( slot );
  }

  /**
   * \brief Notify that the value in this property has changed.  Should be called from within the setter function.
   */
  void changed()
  {
    changed_( shared_from_this() );
  }

protected:
  wxPropertyGrid* grid_;
  wxPGProperty* property_;

  ChangedSignal changed_;
};

class StatusProperty : public PropertyBase
{
public:
  StatusProperty(const std::string& name, const std::string& prefix, const CategoryPropertyWPtr& parent, void* user_data);
  ~StatusProperty();

  virtual void writeToGrid();
  virtual void readFromGrid() {}
  virtual void saveToConfig(wxConfigBase* config) {}
  virtual void loadFromConfig(wxConfigBase* config) {}

  virtual std::string getName() { return (const char*)name_.char_str(); }
  virtual std::string getPrefix() { return (const char*)name_.char_str(); }
  virtual void setPrefix(const std::string& prefix);
  virtual bool getSave() { return false; }
  virtual void* getUserData() { return user_data_; }

  virtual CategoryPropertyWPtr getParent() { return parent_; }

  virtual wxPGProperty* getPGProperty() { return 0; }
  virtual void addLegacyName(const std::string& name) {}

  virtual void setPGClientData();

  virtual void setUserData(void* user_data) { user_data_ = user_data; }

  void setStatus(StatusLevel status, const std::string& name, const std::string& text);
  void deleteStatus(const std::string& name);
  void clear();

  void disable();
  void enable();

  StatusLevel getTopLevelStatus();

private:
  void updateTopLevelStatus();

  wxString name_;
  wxString prefix_;
  CategoryPropertyWPtr parent_;
  void* user_data_;

  wxPGProperty* top_property_;

  struct Status
  {
    Status()
    : level(status_levels::Ok)
    , property(0)
    , kill(false)
    {}

    StatusLevel level;
    wxString name;
    wxString text;
    wxPGProperty* property;
    bool kill;
  };
  typedef std::map<std::string, Status> M_StringToStatus;
  boost::mutex status_mutex_;
  M_StringToStatus statuses_;

  bool enabled_;
  bool prefix_changed_;

  StatusLevel top_status_;
};

/**
 * \class Property
 * \brief Base class for properties
 *
 * The Property (and PropertyManager) interfaces abstract away the various things that must be done with properties,
 * such as setting/getting the value in a wxPropertyGrid, saving the property to disk, etc.  Every property
 * references a Getter and Setter, which it uses to get and set the value.  Properties should not be created
 * directly, rather they should be created through the PropertyManager class.
 */
template<typename T>
class Property : public PropertyBase
{
public:
  typedef boost::function<T (void)> Getter;
  typedef boost::function<void (const T&)> Setter;

public:
  /**
   * \brief Constructor
   *
   * @param name Name of this property (eg, "Color")
   * @param prefix Prefix for this property (eg, "Head Laser Scan")
   * @param grid The wxPropertyGrid to use
   * @param parent The parent to put this property under
   * @param getter Getter function/method.  See boost::function and boost::bind for more information
   * @param setter Setter function/method.  See boost::function and boost::bind for more information
   * @return
   */
  Property( const std::string& name, const std::string& prefix, const CategoryPropertyWPtr& parent, const Getter& getter, const Setter& setter )
  : name_( wxString::FromAscii( name.c_str() ) )
  , prefix_( wxString::FromAscii( prefix.c_str() ) )
  , parent_( parent )
  , save_( true )
  , user_data_( 0 )
  , getter_( getter )
  , setter_( setter )
  {
    if ( setter_ == 0 )
    {
      save_ = false;
    }
  }

  /**
   * Destructor
   */
  virtual ~Property()
  {
  }

  /**
   * \brief Get the current value
   * @return The current value
   */
  T get() { return getter_(); }
  /**
   * \brief Set the value of this property
   * @param val The value
   */
  void set( const T& val )
  {
    setter_( val );

    changed();
  }

  bool hasGetter() { return getter_ != 0; }
  bool hasSetter() { return setter_ != 0; }

  /**
   * \brief Set whether we should save this property to disk or not
   * @param save If true, this property will be saved to disk
   */
  void setSave( bool save ) { save_ = save; }
  /**
   * \brief Returns whether or not this property should be saved to disk.
   */
  bool getSave() { return save_; }

  /**
   * \brief Get the name of this property
   * @return The name of this property
   */
  virtual std::string getName() { return (const char*)name_.mb_str(); }
  /**
   * \brief Get the prefix of this property
   * @return The prefix of this property
   */
  virtual std::string getPrefix() { return (const char*)prefix_.mb_str(); }

  /**
   * \brief Get the wxPGProperty associated with this property.
   * @return The wxPGProperty
   */
  virtual wxPGProperty* getPGProperty()
  {
    return property_;
  }

  /**
   * \brief Get the user data associated with this property
   */
  void* getUserData() { return user_data_; }
  /**
   * \brief Set the user data associated with this property
   * @param user_data
   */
  void setUserData(void* user_data)
  {
    user_data_ = user_data;
  }

  virtual CategoryPropertyWPtr getParent() { return parent_; }

  virtual void addLegacyName(const std::string& name)
  {
    legacy_names_.push_back(wxString::FromAscii(name.c_str()));
  }

  virtual void setToError()
  {
    setPropertyToError(property_);
  }

  virtual void setToWarn()
  {
    setPropertyToWarn(property_);
  }

  virtual void setToOK()
  {
    setPropertyToOK(property_);
  }

  virtual void setToDisabled()
  {
    setPropertyToDisabled(property_);
  }

  virtual void setHelpText(const std::string& text)
  {
    help_text_ = text;
    changed();
  }

  virtual void setPrefix(const std::string& prefix)
  {
    prefix_ = wxString::FromAscii(prefix.c_str());
    setPropertyName(property_, prefix_ + name_);
  }

protected:
  wxString name_;
  wxString prefix_;
  CategoryPropertyWPtr parent_;

  bool save_;

  void* user_data_;

  typedef std::vector<wxString> V_wxString;
  V_wxString legacy_names_;

  std::string help_text_;

private:
  Getter getter_;
  Setter setter_;
};

class BoolProperty : public Property<bool>
{
public:
  BoolProperty( const std::string& name, const std::string& prefix, const CategoryPropertyWPtr& parent, const Getter& getter, const Setter& setter )
  : Property<bool>( name, prefix, parent, getter, setter )
  {
  }

  virtual void writeToGrid();
  virtual void readFromGrid();
  virtual void saveToConfig( wxConfigBase* config );
  virtual void loadFromConfig( wxConfigBase* config );
};

class IntProperty : public Property<int>
{
public:
  IntProperty( const std::string& name, const std::string& prefix, const CategoryPropertyWPtr& parent, const Getter& getter, const Setter& setter )
  : Property<int>( name, prefix, parent, getter, setter )
  {
  }

  void setMin( int min );
  void setMax( int max );

  virtual void writeToGrid();
  virtual void readFromGrid();
  virtual void saveToConfig( wxConfigBase* config );
  virtual void loadFromConfig( wxConfigBase* config );
};

class FloatProperty : public Property<float>
{
public:
  FloatProperty( const std::string& name, const std::string& prefix, const CategoryPropertyWPtr& parent, const Getter& getter, const Setter& setter )
  : Property<float>( name, prefix, parent, getter, setter )
  {
  }

  void setMin( float min );
  void setMax( float max );

  virtual void writeToGrid();
  virtual void readFromGrid();
  virtual void saveToConfig( wxConfigBase* config );
  virtual void loadFromConfig( wxConfigBase* config );
};

class DoubleProperty : public Property<double>
{
public:
  DoubleProperty( const std::string& name, const std::string& prefix, const CategoryPropertyWPtr& parent, const Getter& getter, const Setter& setter )
  : Property<double>( name, prefix, parent, getter, setter )
  {
  }

  void setMin( double min );
  void setMax( double max );

  virtual void writeToGrid();
  virtual void readFromGrid();
  virtual void saveToConfig( wxConfigBase* config );
  virtual void loadFromConfig( wxConfigBase* config );
};

class StringProperty : public Property<std::string>
{
public:
  StringProperty( const std::string& name, const std::string& prefix, const CategoryPropertyWPtr& parent, const Getter& getter, const Setter& setter )
  : Property<std::string>( name, prefix, parent, getter, setter )
  {
  }

  virtual void writeToGrid();
  virtual void readFromGrid();
  virtual void saveToConfig( wxConfigBase* config );
  virtual void loadFromConfig( wxConfigBase* config );
};

class ROSTopicProperty;
class ROSTopicStringProperty : public StringProperty
{
public:
  ROSTopicStringProperty( const std::string& name, const std::string& prefix, const CategoryPropertyWPtr& parent, const Getter& getter, const Setter& setter )
  : StringProperty( name, prefix, parent, getter, setter )
  {
  }

  void setMessageType(const std::string& message_type);

  virtual void writeToGrid();

private:
  std::string message_type_;
  ROSTopicProperty* ros_topic_property_;
};

class ColorProperty : public Property<Color>
{
public:
  ColorProperty( const std::string& name, const std::string& prefix, const CategoryPropertyWPtr& parent, const Getter& getter, const Setter& setter )
  : Property<Color>( name, prefix, parent, getter, setter )
  {
  }

  virtual void writeToGrid();
  virtual void readFromGrid();
  virtual void saveToConfig( wxConfigBase* config );
  virtual void loadFromConfig( wxConfigBase* config );
};

class EnumProperty : public Property<int>
{
public:
  EnumProperty( const std::string& name, const std::string& prefix, const CategoryPropertyWPtr& parent, const Getter& getter, const Setter& setter );

  void addOption( const std::string& name, int value );
  void clear ();

  virtual void writeToGrid();
  virtual void readFromGrid();
  virtual void saveToConfig( wxConfigBase* config );
  virtual void loadFromConfig( wxConfigBase* config );

private:
  boost::shared_ptr<wxPGChoices> choices_;

  boost::mutex mutex_;
};

class EditEnumProperty : public Property<std::string>
{
public:
  EditEnumProperty( const std::string& name, const std::string& prefix, const CategoryPropertyWPtr& parent, const Getter& getter, const Setter& setter );

  void addOption( const std::string& name );
  void clear ();

  void setOptionCallback(const EditEnumOptionCallback& cb);

  virtual void writeToGrid();
  virtual void readFromGrid();
  virtual void saveToConfig( wxConfigBase* config );
  virtual void loadFromConfig( wxConfigBase* config );

private:
  boost::shared_ptr<wxPGChoices> choices_;
  EditEnumOptionCallback option_cb_;
  EditEnumPGProperty* ee_property_;

  boost::mutex mutex_;
};

class TFFramePGProperty;
class TFFrameProperty : public EditEnumProperty
{
public:
  TFFrameProperty( const std::string& name, const std::string& prefix, const CategoryPropertyWPtr& parent, const Getter& getter, const Setter& setter )
  : EditEnumProperty( name, prefix, parent, getter, setter )
  {
  }

  virtual void writeToGrid();

private:
  TFFramePGProperty* tf_frame_property_;
};


class CategoryProperty : public Property<bool>
{
public:
  CategoryProperty( const std::string& label, const std::string& name, const std::string& prefix, const CategoryPropertyWPtr& parent, const Getter& getter, const Setter& setter, bool checkbox )
  : Property<bool>( name, prefix, parent, getter, setter )
  , label_(wxString::FromAscii(label.c_str()))
  , checkbox_(checkbox)
  {
  }

  void setLabel( const std::string& label );
  void expand();
  void collapse();

  virtual void writeToGrid();
  virtual void readFromGrid();
  virtual void saveToConfig( wxConfigBase* config );
  virtual void loadFromConfig( wxConfigBase* config );

  virtual void setToError();
  virtual void setToWarn();
  virtual void setToOK();
  virtual void setToDisabled();

private:
  wxString label_;
  bool checkbox_;
};

class Vector3Property : public Property<Ogre::Vector3>
{
public:
  Vector3Property( const std::string& name, const std::string& prefix, const CategoryPropertyWPtr& parent, const Getter& getter, const Setter& setter )
  : Property<Ogre::Vector3>( name, prefix, parent, getter, setter )
  , composed_parent_( NULL )
  , x_( NULL )
  , y_( NULL )
  , z_( NULL )
  {
  }

  virtual ~Vector3Property();

  virtual void writeToGrid();
  virtual void readFromGrid();
  virtual void saveToConfig( wxConfigBase* config );
  virtual void loadFromConfig( wxConfigBase* config );
  virtual void setPGClientData();
  virtual void reset();
  virtual void show();
  virtual void hide();
  virtual void setPrefix(const std::string& prefix);

  virtual void setToError()
  {
    setPropertyToError(composed_parent_);
    setPropertyToError(x_);
    setPropertyToError(y_);
    setPropertyToError(z_);
  }

  virtual void setToWarn()
  {
    setPropertyToWarn(composed_parent_);
    setPropertyToWarn(x_);
    setPropertyToWarn(y_);
    setPropertyToWarn(z_);
  }

  virtual void setToOK()
  {
    setPropertyToOK(composed_parent_);
    setPropertyToOK(x_);
    setPropertyToOK(y_);
    setPropertyToOK(z_);
  }

  virtual void setToDisabled()
  {
    setPropertyToDisabled(composed_parent_);
    setPropertyToDisabled(x_);
    setPropertyToDisabled(y_);
    setPropertyToDisabled(z_);
  }

protected:
  wxPGProperty* composed_parent_;
  wxPGProperty* x_;
  wxPGProperty* y_;
  wxPGProperty* z_;
};

class QuaternionProperty : public Property<Ogre::Quaternion>
{
public:
  QuaternionProperty( const std::string& name, const std::string& prefix, const CategoryPropertyWPtr& parent, const Getter& getter, const Setter& setter )
  : Property<Ogre::Quaternion>( name, prefix, parent, getter, setter )
  , composed_parent_( NULL )
  , x_( NULL )
  , y_( NULL )
  , z_( NULL )
  , w_( NULL )
  {
  }

  virtual ~QuaternionProperty();

  virtual void writeToGrid();
  virtual void readFromGrid();
  virtual void saveToConfig( wxConfigBase* config );
  virtual void loadFromConfig( wxConfigBase* config );
  virtual void setPGClientData();
  virtual void reset();
  virtual void show();
  virtual void hide();
  virtual void setPrefix(const std::string& prefix);

  virtual void setToError()
  {
    setPropertyToError(composed_parent_);
    setPropertyToError(x_);
    setPropertyToError(y_);
    setPropertyToError(z_);
    setPropertyToError(w_);
  }

  virtual void setToWarn()
  {
    setPropertyToWarn(composed_parent_);
    setPropertyToWarn(x_);
    setPropertyToWarn(y_);
    setPropertyToWarn(z_);
    setPropertyToWarn(w_);
  }

  virtual void setToOK()
  {
    setPropertyToOK(composed_parent_);
    setPropertyToOK(x_);
    setPropertyToOK(y_);
    setPropertyToOK(z_);
    setPropertyToOK(w_);
  }

  virtual void setToDisabled()
  {
    setPropertyToDisabled(composed_parent_);
    setPropertyToDisabled(x_);
    setPropertyToDisabled(y_);
    setPropertyToDisabled(z_);
    setPropertyToDisabled(w_);
  }

protected:
  wxPGProperty* composed_parent_;
  wxPGProperty* x_;
  wxPGProperty* y_;
  wxPGProperty* z_;
  wxPGProperty* w_;
};


} // namespace rviz

#endif
