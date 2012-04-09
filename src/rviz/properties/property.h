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

#include <float.h>
#include <limits.h>

#include "rviz/helpers/color.h"
#include "forwards.h"
#include "rviz/status_level.h"

#include <boost/function.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/console.h>
#include <ros/assert.h>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>

#include <string>
#include <vector>

class QColor;

namespace rviz
{

class Config;
class PropertyTreeWidget;
class PropertyWidgetItem;
class CategoryProperty;
class VisualizationManager;

void setPropertyHelpText(PropertyTreeWidget* grid, PropertyWidgetItem* property, const std::string& text);
void setPropertyToColors(PropertyTreeWidget* grid, PropertyWidgetItem* property, const QColor& fg_color, const QColor& bg_color, uint32_t column = 0);
void setPropertyToError(PropertyTreeWidget* grid, PropertyWidgetItem* property, uint32_t column = 0);
void setPropertyToWarn(PropertyTreeWidget* grid, PropertyWidgetItem* property, uint32_t column = 0);
void setPropertyToOK(PropertyTreeWidget* grid, PropertyWidgetItem* property, uint32_t column = 0);
void setPropertyToDisabled(PropertyTreeWidget* grid, PropertyWidgetItem* property, uint32_t column = 0);

/**
 * \brief Abstract base class for properties
 */
class PropertyBase : public boost::enable_shared_from_this<PropertyBase>
{
public:
  PropertyBase();
  virtual ~PropertyBase();
  void writeToGrid();
  virtual void doWriteToGrid() = 0;
  virtual void readFromGrid() = 0;
  virtual void saveToConfig( Config* config ) = 0;
  virtual void loadFromConfig( Config* config ) = 0;

  virtual std::string getName() = 0;
  virtual std::string getPrefix() = 0;
  virtual void setPrefix(const std::string& prefix) = 0;
  virtual bool getSave() = 0;

  virtual PropertyWidgetItem* getWidgetItem() = 0;

  virtual CategoryPropertyWPtr getParent() = 0;

  virtual void addLegacyName(const std::string& name) = 0;

  virtual void setPropertyTreeWidget(PropertyTreeWidget* grid);
  virtual PropertyTreeWidget* getPropertyTreeWidget() { return grid_; }

  virtual void setUserData(void* user_data) { user_data_ = user_data; }
  void* getUserData() { return user_data_; }

  virtual void reset();

  virtual void show();
  virtual void hide();

  virtual bool isSelected();

  /**
   * \brief Notify that the value in this property has changed.  Should be called from within the setter function.
   */
  void changed();

  /** @brief Notify that the value of this property has changed such that it affects the config file. */
  void configChanged();

protected:
  PropertyTreeWidget* grid_;
  PropertyWidgetItem* widget_item_;
  void* user_data_;

private:
  // I needed to purge boost::signal, and I didn't really want to make
  // every Property a QObject.  There is only one class which needs to
  // be notified of property changes, and that is PropertyManager.
  // Therefore I'm making an explicit function call connection instead
  // of a Qt signal or a boost signal.
  friend class PropertyManager;
  PropertyManager* manager_;
};

class StatusProperty : public PropertyBase
{
public:
  StatusProperty(const std::string& name, const std::string& prefix, const CategoryPropertyWPtr& parent, void* user_data);
  ~StatusProperty();

  virtual void doWriteToGrid();
  virtual void readFromGrid() {}
  virtual void saveToConfig(Config* config) {}
  virtual void loadFromConfig(Config* config) {}

  virtual std::string getName() { return name_; }
  virtual std::string getPrefix() { return prefix_; }
  virtual void setPrefix(const std::string& prefix);
  virtual bool getSave() { return false; }

  virtual CategoryPropertyWPtr getParent() { return parent_; }

  virtual PropertyWidgetItem* getWidgetItem() { return 0; }
  virtual void addLegacyName(const std::string& name) {}

  void setStatus(StatusLevel status, const std::string& name, const std::string& text);
  void deleteStatus(const std::string& name);
  void clear();

  void disable();
  void enable();

  StatusLevel getTopLevelStatus();

private:
  void updateTopLevelStatus();

  std::string name_;
  std::string prefix_;
  CategoryPropertyWPtr parent_;

  PropertyWidgetItem* top_widget_item_;

  struct Status
  {
    Status()
    : level(status_levels::Ok)
    , widget_item(0)
    , kill(false)
    {}

    StatusLevel level;
    std::string name;
    std::string text;
    PropertyWidgetItem* widget_item;
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
 * such as setting/getting the value in a PropertyTreeWidget, saving the property to disk, etc.  Every property
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
   * @param grid The PropertyTreeWidget to use
   * @param parent The parent to put this property under
   * @param getter Getter function/method.  See boost::function and boost::bind for more information
   * @param setter Setter function/method.  See boost::function and boost::bind for more information
   * @return
   */
  Property( const std::string& name, const std::string& prefix, const CategoryPropertyWPtr& parent, const Getter& getter, const Setter& setter )
  : name_( name )
  , prefix_( prefix )
  , parent_( parent )
  , save_( true )
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
    if( hasSetter() )
    {
      setter_( val );
      changed();
      configChanged();
    }
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
  virtual std::string getName() { return name_; }
  /**
   * \brief Get the prefix of this property
   * @return The prefix of this property
   */
  virtual std::string getPrefix() { return prefix_; }

  /**
   * \brief Get the PropertyWidgetItem associated with this property.
   * @return The PropertyWidgetItem
   */
  virtual PropertyWidgetItem* getWidgetItem()
  {
    return widget_item_;
  }

  virtual CategoryPropertyWPtr getParent() { return parent_; }

  virtual void addLegacyName(const std::string& name)
  {
    legacy_names_.push_back(name);
  }

  virtual void setToError()
  {
    setPropertyToError(grid_, widget_item_);
  }

  virtual void setToWarn()
  {
    setPropertyToWarn(grid_, widget_item_);
  }

  virtual void setToOK()
  {
    setPropertyToOK(grid_, widget_item_);
  }

  virtual void setToDisabled()
  {
    setPropertyToDisabled(grid_, widget_item_);
  }

  virtual void setHelpText(const std::string& text)
  {
    help_text_ = text;
    changed();
  }

  virtual void setPrefix(const std::string& prefix)
  {
    prefix_ = prefix;
  }

protected:
  std::string name_;
  std::string prefix_;
  CategoryPropertyWPtr parent_;

  bool save_;

  typedef std::vector<std::string> V_string;
  V_string legacy_names_;

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

  virtual void doWriteToGrid();
  virtual void readFromGrid();
  virtual void saveToConfig( Config* config );
  virtual void loadFromConfig( Config* config );
};

class IntProperty : public Property<int>
{
public:
  IntProperty( const std::string& name, const std::string& prefix, const CategoryPropertyWPtr& parent, const Getter& getter, const Setter& setter )
  : Property<int>( name, prefix, parent, getter, setter )
  , min_( INT_MIN )
  , max_( INT_MAX )
  {
  }

  void setMin( int min );
  void setMax( int max );

  virtual void doWriteToGrid();
  virtual void readFromGrid();
  virtual void saveToConfig( Config* config );
  virtual void loadFromConfig( Config* config );

private:
  int min_;
  int max_;
};

class FloatProperty : public Property<float>
{
public:
  FloatProperty( const std::string& name, const std::string& prefix, const CategoryPropertyWPtr& parent, const Getter& getter, const Setter& setter )
  : Property<float>( name, prefix, parent, getter, setter )
  , min_( -FLT_MAX )
  , max_( FLT_MAX )
  {
  }

  void setMin( float min );
  void setMax( float max );

  virtual void doWriteToGrid();
  virtual void readFromGrid();
  virtual void saveToConfig( Config* config );
  virtual void loadFromConfig( Config* config );

private:
  float min_;
  float max_;
};

class StringProperty : public Property<std::string>
{
public:
  StringProperty( const std::string& name, const std::string& prefix, const CategoryPropertyWPtr& parent, const Getter& getter, const Setter& setter )
  : Property<std::string>( name, prefix, parent, getter, setter )
  {
  }

  virtual void doWriteToGrid();
  virtual void readFromGrid();
  virtual void saveToConfig( Config* config );
  virtual void loadFromConfig( Config* config );
};

class ROSTopicStringProperty : public StringProperty
{
public:
  ROSTopicStringProperty( const std::string& name, const std::string& prefix, const CategoryPropertyWPtr& parent, const Getter& getter, const Setter& setter )
  : StringProperty( name, prefix, parent, getter, setter )
  {
  }

  void setMessageType(const std::string& message_type) { message_type_ = message_type; }

  virtual void doWriteToGrid();
  virtual void readFromGrid();

private:
  std::string message_type_;
};

class ColorProperty : public Property<Color>
{
public:
  ColorProperty( const std::string& name, const std::string& prefix, const CategoryPropertyWPtr& parent, const Getter& getter, const Setter& setter )
  : Property<Color>( name, prefix, parent, getter, setter )
  {
  }

  virtual void doWriteToGrid();
  virtual void readFromGrid();
  virtual void saveToConfig( Config* config );
  virtual void loadFromConfig( Config* config );
};

typedef std::pair<std::string, int> Choice;
typedef std::vector<Choice> Choices;

class EnumProperty : public Property<int>
{
public:
  EnumProperty( const std::string& name, const std::string& prefix, const CategoryPropertyWPtr& parent, const Getter& getter, const Setter& setter )
  : Property<int>( name, prefix, parent, getter, setter )
  {
  }

  void addOption( const std::string& name, int value );
  void clear ();

  virtual void doWriteToGrid();
  virtual void readFromGrid();
  virtual void saveToConfig( Config* config );
  virtual void loadFromConfig( Config* config );

private:
  Choices choices_;

  boost::mutex mutex_;
};

class EditEnumProperty : public Property<std::string>
{
public:
  EditEnumProperty( const std::string& name, const std::string& prefix, const CategoryPropertyWPtr& parent, const Getter& getter, const Setter& setter )
  : Property<std::string>( name, prefix, parent, getter, setter )
    {}

  void addOption( const std::string& name );
  void clear ();

  void setOptionCallback(const EditEnumOptionCallback& cb);

  virtual void doWriteToGrid();
  virtual void readFromGrid();
  virtual void saveToConfig( Config* config );
  virtual void loadFromConfig( Config* config );

private:
  std::vector<std::string> choices_;
  EditEnumOptionCallback option_cb_;

  boost::mutex mutex_;
};

class TFFrameProperty : public EditEnumProperty
{
public:
  TFFrameProperty( const std::string& name, const std::string& prefix, const CategoryPropertyWPtr& parent, const Getter& getter, const Setter& setter )
  : EditEnumProperty( name, prefix, parent, getter, setter )
  {
  }

  void optionCallback( V_string& options_out );

  virtual void doWriteToGrid();
};


class CategoryProperty : public Property<bool>
{
public:
  CategoryProperty( const std::string& label, const std::string& name, const std::string& prefix, const CategoryPropertyWPtr& parent, const Getter& getter, const Setter& setter, bool checkbox )
  : Property<bool>( name, prefix, parent, getter, setter )
  , label_(label)
  , checkbox_(checkbox)
  {
  }

  virtual ~CategoryProperty();
  virtual void reset();

  void setLabel( const std::string& label );
  void expand();
  void collapse();

  virtual void doWriteToGrid();
  virtual void readFromGrid();
  virtual void saveToConfig( Config* config );
  virtual void loadFromConfig( Config* config );

  virtual void setToOK();

private:
  std::string label_;
  bool checkbox_;
};

class Vector3Property : public Property<Ogre::Vector3>
{
public:
  Vector3Property( const std::string& name, const std::string& prefix, const CategoryPropertyWPtr& parent, const Getter& getter, const Setter& setter )
  : Property<Ogre::Vector3>( name, prefix, parent, getter, setter )
  , x_( NULL )
  , y_( NULL )
  , z_( NULL )
  {
  }

  virtual void doWriteToGrid();
  virtual void readFromGrid();
  virtual void saveToConfig( Config* config );
  virtual void loadFromConfig( Config* config );
  virtual void reset();

  virtual void setToError()
  {
    setPropertyToError(grid_, widget_item_);
    setPropertyToError(grid_, x_);
    setPropertyToError(grid_, y_);
    setPropertyToError(grid_, z_);
  }

  virtual void setToWarn()
  {
    setPropertyToWarn(grid_, widget_item_);
    setPropertyToWarn(grid_, x_);
    setPropertyToWarn(grid_, y_);
    setPropertyToWarn(grid_, z_);
  }

  virtual void setToOK()
  {
    setPropertyToOK(grid_, widget_item_);
    setPropertyToOK(grid_, x_);
    setPropertyToOK(grid_, y_);
    setPropertyToOK(grid_, z_);
  }

  virtual void setToDisabled()
  {
    setPropertyToDisabled(grid_, widget_item_);
    setPropertyToDisabled(grid_, x_);
    setPropertyToDisabled(grid_, y_);
    setPropertyToDisabled(grid_, z_);
  }

protected:
  PropertyWidgetItem* x_;
  PropertyWidgetItem* y_;
  PropertyWidgetItem* z_;
};

class QuaternionProperty : public Property<Ogre::Quaternion>
{
public:
  QuaternionProperty( const std::string& name, const std::string& prefix, const CategoryPropertyWPtr& parent, const Getter& getter, const Setter& setter )
  : Property<Ogre::Quaternion>( name, prefix, parent, getter, setter )
  , x_( NULL )
  , y_( NULL )
  , z_( NULL )
  , w_( NULL )
  {
  }

  virtual void doWriteToGrid();
  virtual void readFromGrid();
  virtual void saveToConfig( Config* config );
  virtual void loadFromConfig( Config* config );
  virtual void reset();

  virtual void setToError()
  {
    setPropertyToError( grid_, widget_item_);
    setPropertyToError( grid_, x_);
    setPropertyToError( grid_, y_);
    setPropertyToError( grid_, z_);
    setPropertyToError( grid_, w_);
  }

  virtual void setToWarn()
  {
    setPropertyToWarn( grid_, widget_item_);
    setPropertyToWarn( grid_, x_);
    setPropertyToWarn( grid_, y_);
    setPropertyToWarn( grid_, z_);
    setPropertyToWarn( grid_, w_);
  }

  virtual void setToOK()
  {
    setPropertyToOK( grid_, widget_item_);
    setPropertyToOK( grid_, x_);
    setPropertyToOK( grid_, y_);
    setPropertyToOK( grid_, z_);
    setPropertyToOK( grid_, w_);
  }

  virtual void setToDisabled()
  {
    setPropertyToDisabled( grid_, widget_item_);
    setPropertyToDisabled( grid_, x_);
    setPropertyToDisabled( grid_, y_);
    setPropertyToDisabled( grid_, z_);
    setPropertyToDisabled( grid_, w_);
  }

protected:
  PropertyWidgetItem* x_;
  PropertyWidgetItem* y_;
  PropertyWidgetItem* z_;
  PropertyWidgetItem* w_;
};


} // namespace rviz

#endif
