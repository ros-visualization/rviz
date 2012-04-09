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

#include <QColor>

#include <tf/transform_listener.h>

#include "rviz/config.h"
#include "rviz/frame_manager.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"
#include "rviz/properties/property_widget_item.h"
#include "rviz/properties/property_tree_widget.h"
#include "rviz/properties/topic_info_variant.h"
#include "rviz/properties/color_item.h"
#include "rviz/properties/enum_item.h"
#include "rviz/properties/edit_enum_item.h"
#include "rviz/properties/compound_widget_item.h"

namespace rviz
{

static const QColor ERROR_COLOR(178, 23, 46); // red-ish
static const QColor WARN_COLOR(222, 213, 17); // yellow-ish
static const QColor CATEGORY_COLOR( 4, 89, 127 ); // blue-ish

void PropertyBase::writeToGrid()
{
  // Any change events coming from the grid during a "writeToGrid()"
  // call are caused by us, not by the user, so make sure we don't
  // end up calling readFromGrid() as a result (i.e. ignore them).
  bool ign = grid_->setIgnoreChanges( true );
  doWriteToGrid();
  grid_->setIgnoreChanges( ign );
}

PropertyWidgetItem* getCategoryPGProperty(const CategoryPropertyWPtr& wprop)
{
  CategoryPropertyPtr prop = wprop.lock();

  if (prop)
  {
    return prop->getWidgetItem();
  }

  return NULL;
}

void setPropertyHelpText(PropertyTreeWidget* grid, PropertyWidgetItem* widget_item, const std::string& text)
{
  if( widget_item )
  {
    bool ign = grid->setIgnoreChanges( true );
    widget_item->setWhatsThis( 0, QString::fromStdString( text ));
    widget_item->setWhatsThis( 1, QString::fromStdString( text ));
    grid->setIgnoreChanges( ign );
  }
}

void setPropertyToColors(PropertyTreeWidget* grid, PropertyWidgetItem* widget_item, const QColor& fg_color, const QColor& bg_color, uint32_t column)
{
  if( widget_item )
  {
    bool ign = grid->setIgnoreChanges( true );
    widget_item->setForeground( column, fg_color );
    widget_item->setBackground( column, bg_color );
    grid->setIgnoreChanges( ign );
  }
}

void setPropertyToError(PropertyTreeWidget* grid, PropertyWidgetItem* property, uint32_t column)
{
  setPropertyToColors(grid, property, Qt::white, ERROR_COLOR, column);
}

void setPropertyToWarn(PropertyTreeWidget* grid, PropertyWidgetItem* property, uint32_t column)
{
  setPropertyToColors(grid, property, Qt::white, WARN_COLOR, column);
}

void setPropertyToOK(PropertyTreeWidget* grid, PropertyWidgetItem* property, uint32_t column)
{
  setPropertyToColors(grid, property, Qt::black, Qt::white, column);
}

void setPropertyToDisabled(PropertyTreeWidget* grid, PropertyWidgetItem* property, uint32_t column)
{
  setPropertyToColors(grid, property, QColor(0x33, 0x44, 0x44), QColor(0xaa, 0xaa, 0xaa), column);
}

PropertyBase::PropertyBase()
: grid_(NULL)
, widget_item_(NULL)
, user_data_(NULL)
, manager_(NULL)
{
}

PropertyBase::~PropertyBase()
{
  delete widget_item_;
}

void PropertyBase::reset()
{
  grid_ = 0;

  delete widget_item_;
  widget_item_ = 0;
}

void PropertyBase::setPropertyTreeWidget(PropertyTreeWidget* grid)
{
  grid_ = grid;
}

void PropertyBase::hide()
{
  if( widget_item_ )
  {
    widget_item_->setHidden( true );
  }
}

void PropertyBase::show()
{
  if( widget_item_ )
  {
    widget_item_->setHidden( false );
  }
}

bool PropertyBase::isSelected()
{
  if( widget_item_ && grid_ )
  {
    return grid_->currentItem() == widget_item_;
  }

  return false;
}

void PropertyBase::changed()
{
  // I needed to purge boost::signal, and I didn't really want to make
  // every Property a QObject.  There is only one class which needs to
  // be notified of property changes, and that is PropertyManager.
  // Therefore I'm making an explicit function call connection instead
  // of a Qt signal or a boost signal.
  if( manager_ )
  {
    manager_->propertySet( shared_from_this() );
  }
}

void PropertyBase::configChanged()
{
  if( manager_ )
  {
    manager_->emitConfigChanged();
  }
}

StatusProperty::StatusProperty(const std::string& name, const std::string& prefix, const CategoryPropertyWPtr& parent, void* user_data)
: name_(name)
, prefix_(prefix)
, parent_(parent)
, top_widget_item_(0)
, enabled_(true)
, prefix_changed_(false)
, top_status_(status_levels::Ok)
{
  user_data_ = user_data;
}

StatusProperty::~StatusProperty()
{
  delete top_widget_item_;
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

void StatusProperty::setPrefix(const std::string& prefix)
{
  boost::mutex::scoped_lock lock(status_mutex_);
  prefix_ = prefix;
  prefix_changed_ = true;
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

  // Update the top level status here so that it can be used immediately
  updateTopLevelStatus();

  changed();
}

void StatusProperty::updateTopLevelStatus()
{
  top_status_ = status_levels::Ok;
  M_StringToStatus::iterator it = statuses_.begin();
  M_StringToStatus::iterator end = statuses_.end();
  for (; it != end; ++it)
  {
    Status& status = it->second;

    if (status.kill)
    {
      continue;
    }

    if (status.level > top_status_)
    {
      top_status_ = status.level;
    }
  }
}

void StatusProperty::setStatus(StatusLevel level, const std::string& name, const std::string& text)
{
  boost::mutex::scoped_lock lock(status_mutex_);

  if (!enabled_)
  {
    return;
  }

  Status& status = statuses_[name];

  // Status hasn't changed, return
  if (status.level == level && status.text == text && !status.kill)
  {
    return;
  }

  status.name = name;
  status.text = text;
  status.level = level;
  status.kill = false;

  // Update the top level status here so that it can be used immediately
  updateTopLevelStatus();

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

  // Update the top level status here so that it can be used immediately
  updateTopLevelStatus();

  changed();
}

void StatusProperty::doWriteToGrid()
{
  boost::mutex::scoped_lock lock(status_mutex_);

  if ( !top_widget_item_ )
  {
    std::string top_name = name_ + "TopStatus";

    top_widget_item_ = new PropertyWidgetItem( this, "", false, false, true );
    top_widget_item_->addToParent();
  }

  bool expanded = top_widget_item_->isExpanded();

  top_status_ = status_levels::Ok;

  std::vector<std::string> to_erase;
  M_StringToStatus::iterator it = statuses_.begin();
  M_StringToStatus::iterator end = statuses_.end();
  for( ; it != end; ++it )
  {
    Status& status = it->second;

    if( status.kill )
    {
      to_erase.push_back(it->first);
      continue;
    }

    if( !status.widget_item )
    {
      status.widget_item = new PropertyWidgetItem( this, status.name, false, false, false );
      status.widget_item->addToParent( top_widget_item_ );
    }

    if( status.level > top_status_ )
    {
      top_status_ = status.level;
    }

    if( enabled_ )
    {
      switch( status.level )
      {
      case status_levels::Ok:
        setPropertyToOK( grid_, status.widget_item );
        break;
      case status_levels::Warn:
        setPropertyToColors( grid_, status.widget_item, WARN_COLOR, Qt::white );
        break;
      case status_levels::Error:
        setPropertyToColors( grid_, status.widget_item, ERROR_COLOR, Qt::white );
        break;
      }
    }
    else
    {
      setPropertyToDisabled( grid_, status.widget_item );
    }

    status.widget_item->setRightText( status.text );
    setPropertyHelpText( grid_, status.widget_item, status.text );
  }

  std::vector<std::string>::iterator kill_it = to_erase.begin();
  std::vector<std::string>::iterator kill_end = to_erase.end();
  for( ; kill_it != kill_end; ++kill_it )
  {
    Status& status = statuses_[*kill_it];
    delete status.widget_item;
    statuses_.erase( *kill_it );
  }

  top_widget_item_->setExpanded( expanded );

  std::string label;
  if( enabled_ )
  {
    switch( top_status_ )
    {
    case status_levels::Ok:
      setPropertyToColors( grid_, top_widget_item_, Qt::black, Qt::white );
      label = name_ + ": OK";
      break;
    case status_levels::Warn:
      setPropertyToColors( grid_, top_widget_item_, WARN_COLOR, Qt::white );
      label = name_ + ": Warning";
      break;
    case status_levels::Error:
      setPropertyToColors( grid_, top_widget_item_, ERROR_COLOR, Qt::white );
      label = name_ + ": Error";
      break;
    }
  }
  else
  {
    setPropertyToDisabled( grid_, top_widget_item_ );
    label = name_ + ": Disabled";
  }

  top_widget_item_->setLeftText( label );
  top_widget_item_->sortChildren( 0, Qt::AscendingOrder );
}

StatusLevel StatusProperty::getTopLevelStatus()
{
  return top_status_;
}

void BoolProperty::doWriteToGrid()
{
  if ( !widget_item_ )
  {
    widget_item_ = new PropertyWidgetItem( this, name_, hasSetter(), true );
    widget_item_->addToParent();
  }
  bool ign = getPropertyTreeWidget()->setIgnoreChanges( true );

  widget_item_->setData( 1, Qt::CheckStateRole, get() ? Qt::Checked : Qt::Unchecked );
  setPropertyHelpText(grid_, widget_item_, help_text_);

  getPropertyTreeWidget()->setIgnoreChanges( ign );
}

void BoolProperty::readFromGrid()
{
  QVariant check_state = widget_item_->data( 1, Qt::CheckStateRole );
  set( check_state == Qt::Checked );
}

void BoolProperty::saveToConfig( Config* config )
{
  config->set( prefix_ + name_, (int)get() );
}

void BoolProperty::loadFromConfig( Config* config )
{
  int val;
  if( !config->get( prefix_ + name_, &val, get() ))
  {
    V_string::iterator it = legacy_names_.begin();
    V_string::iterator end = legacy_names_.end();
    for (; it != end; ++it)
    {
      if (config->get( prefix_ + *it, &val, get() ))
      {
        break;
      }
    }
  }

  set( (bool) val );
}

void IntProperty::setMin( int min )
{
  if( widget_item_ )
  {
    widget_item_->min_ = min;
  }
  min_ = min;
}

void IntProperty::setMax( int max )
{
  if (widget_item_)
  {
    widget_item_->max_ = max;
  }
  max_ = max;
}

void IntProperty::doWriteToGrid()
{
  if ( !widget_item_ )
  {
    widget_item_ = new PropertyWidgetItem( this, name_, hasSetter() );
    widget_item_->addToParent();
    widget_item_->max_ = max_;
    widget_item_->min_ = min_;
  }

  widget_item_->setUserData( get() );

  setPropertyHelpText(grid_, widget_item_, help_text_);
}

void IntProperty::readFromGrid()
{
  set( widget_item_->userData().toInt() );
}

void IntProperty::saveToConfig( Config* config )
{
  config->set( prefix_ + name_, (int)get() );
}

void IntProperty::loadFromConfig( Config* config )
{
  int val;
  if (!config->get( prefix_ + name_, &val, get() ))
  {
    V_string::iterator it = legacy_names_.begin();
    V_string::iterator end = legacy_names_.end();
    for (; it != end; ++it)
    {
      if (config->get( prefix_ + *it, &val, get() ))
      {
        break;
      }
    }
  }

  set( val );
}

void FloatProperty::setMin( float min )
{
  if (widget_item_)
  {
    widget_item_->min_ = min;
  }
  min_ = min;
}

void FloatProperty::setMax( float max )
{
  if (widget_item_)
  {
    widget_item_->max_ = max;
  }
  max_ = max;
}

void FloatProperty::doWriteToGrid()
{
  if( !widget_item_ )
  {
    widget_item_ = new PropertyWidgetItem( this, name_, hasSetter() );
    widget_item_->addToParent();
    widget_item_->max_ = max_;
    widget_item_->min_ = min_;
  }

  widget_item_->setUserData( QVariant( get() ));

  setPropertyHelpText(grid_, widget_item_, help_text_);
}

void FloatProperty::readFromGrid()
{
  set( widget_item_->userData().toFloat() );
}

void FloatProperty::saveToConfig( Config* config )
{
  config->set( prefix_ + name_, (float)get() );
}

void FloatProperty::loadFromConfig( Config* config )
{
  float val;
  if (!config->get( prefix_ + name_, &val, get() ))
  {
    V_string::iterator it = legacy_names_.begin();
    V_string::iterator end = legacy_names_.end();
    for (; it != end; ++it)
    {
      if (config->get( prefix_ + *it, &val, get() ))
      {
        break;
      }
    }
  }

  set( val );
}

void StringProperty::doWriteToGrid()
{
  if( !widget_item_ )
  {
    widget_item_ = new PropertyWidgetItem( this, name_, hasSetter() );
    widget_item_->addToParent();
  }

  widget_item_->setUserData( QString::fromStdString( get() ));

  setPropertyHelpText( grid_, widget_item_, help_text_ );
}

void StringProperty::readFromGrid()
{
  set( widget_item_->userData().toString().toStdString() );
}

void StringProperty::saveToConfig( Config* config )
{
  config->set( prefix_ + name_, get() );
}

void StringProperty::loadFromConfig( Config* config )
{
  std::string val;
  if (!config->get( prefix_ + name_, &val, get() ))
  {
    V_string::iterator it = legacy_names_.begin();
    V_string::iterator end = legacy_names_.end();
    for (; it != end; ++it)
    {
      if (config->get( prefix_ + *it, &val, get() ))
      {
        break;
      }
    }
  }

  set( val );
}

void ROSTopicStringProperty::doWriteToGrid()
{
  if ( !widget_item_ )
  {
    widget_item_ = new PropertyWidgetItem( this, name_, hasSetter() );
    widget_item_->addToParent();
  }
  ros::master::TopicInfo topic;
  topic.name = get();
  topic.datatype = message_type_;

  widget_item_->setUserData( QVariant::fromValue( topic ));

  setPropertyHelpText(grid_, widget_item_, help_text_);
}

void ROSTopicStringProperty::readFromGrid()
{
  ros::master::TopicInfo topic = widget_item_->userData().value<ros::master::TopicInfo>();
  set( topic.name );
}

void ColorProperty::doWriteToGrid()
{
  if( !widget_item_ )
  {
    widget_item_ = new ColorItem( this );
    widget_item_->addToParent();
  }

  Color c = get();
  widget_item_->setUserData( QVariant::fromValue( QColor( c.r_ * 255, c.g_ * 255, c.b_ * 255 )));

  setPropertyHelpText( grid_, widget_item_, help_text_ );
}

void ColorProperty::readFromGrid()
{
  QColor col = widget_item_->userData().value<QColor>();
  set( Color( col.red() / 255.0f, col.green() / 255.0f, col.blue() / 255.0f ) );
}

void ColorProperty::saveToConfig( Config* config )
{
  Color c = get();

  config->set( prefix_ + name_ + "R", c.r_ );
  config->set( prefix_ + name_ + "G", c.g_ );
  config->set( prefix_ + name_ + "B", c.b_ );
}

void ColorProperty::loadFromConfig( Config* config )
{
  Color c = get();
  float r, g, b;
  bool found = true;
  found &= config->get( prefix_ + name_ + "R", &r, c.r_ );
  found &= config->get( prefix_ + name_ + "G", &g, c.g_ );
  found &= config->get( prefix_ + name_ + "B", &b, c.b_ );

  if (!found)
  {
    V_string::iterator it = legacy_names_.begin();
    V_string::iterator end = legacy_names_.end();
    for (; it != end; ++it)
    {
      found = true;
      found &= config->get( prefix_ + *it + "R", &r, c.r_ );
      found &= config->get( prefix_ + *it + "G", &g, c.g_ );
      found &= config->get( prefix_ + *it + "B", &b, c.b_ );

      if (found)
      {
        break;
      }
    }
  }

  set( Color( r, g, b ) );
}

void EnumProperty::addOption( const std::string& name, int value )
{
  boost::mutex::scoped_lock lock(mutex_);
  choices_.push_back( Choice( name, value ));
  changed();
}

void EnumProperty::clear ()
{
  boost::mutex::scoped_lock lock(mutex_);
  choices_.clear();
  changed();
}

void EnumProperty::doWriteToGrid()
{
  boost::mutex::scoped_lock lock(mutex_);

  if (isSelected())
  {
    changed();
    return;
  }

  if( !widget_item_ )
  {
    widget_item_ = new EnumItem( this );
    widget_item_->addToParent();
  }
  EnumItem* enum_item = dynamic_cast<EnumItem*>( widget_item_ );
  ROS_ASSERT( enum_item );
  enum_item->setChoices( choices_ );
  enum_item->setChoiceValue( get() );

  setPropertyHelpText( grid_, widget_item_, help_text_ );
}

void EnumProperty::readFromGrid()
{
  EnumItem* enum_item = dynamic_cast<EnumItem*>( widget_item_ );
  ROS_ASSERT( enum_item );
  set( enum_item->getChoiceValue() );
}

void EnumProperty::saveToConfig( Config* config )
{
  config->set( prefix_ + name_, (int)get() );
}

void EnumProperty::loadFromConfig( Config* config )
{
  int val = INT_MAX;
  if( !config->get( prefix_ + name_, &val, get() ))
  {
    V_string::iterator it = legacy_names_.begin();
    V_string::iterator end = legacy_names_.end();
    for (; it != end; ++it)
    {
      if (config->get( prefix_ + *it, &val, get() ))
      {
        break;
      }
    }
  }

  set( val );
}

void EditEnumProperty::addOption( const std::string& name )
{
  boost::mutex::scoped_lock lock(mutex_);
  choices_.push_back( name );
  changed();
}

void EditEnumProperty::setOptionCallback(const EditEnumOptionCallback& cb)
{
  option_cb_ = cb;
  if( EditEnumItem* ee_item = dynamic_cast<EditEnumItem*>( widget_item_ ))
  {
    ee_item->setOptionCallback( cb );
  }

  changed();
}

void EditEnumProperty::clear ()
{
  boost::mutex::scoped_lock lock(mutex_);
  choices_.clear();
  changed();
}

void EditEnumProperty::doWriteToGrid()
{
  boost::mutex::scoped_lock lock(mutex_);

  if (isSelected())
  {
    changed();
    return;
  }

  if ( !widget_item_ )
  {
    widget_item_ = new EditEnumItem( this );
    widget_item_->addToParent();
  }
  EditEnumItem* ee_item = dynamic_cast<EditEnumItem*>( widget_item_ );
  ROS_ASSERT( ee_item );
  ee_item->setOptionCallback( option_cb_ );
  ee_item->setChoices( choices_ );
  ee_item->setChoice( get() );
  
  setPropertyHelpText(grid_, widget_item_, help_text_);
}

void EditEnumProperty::readFromGrid()
{
  EditEnumItem* ee_item = dynamic_cast<EditEnumItem*>( widget_item_ );
  ROS_ASSERT( ee_item );
  set( ee_item->getChoice() );
}

void EditEnumProperty::saveToConfig( Config* config )
{
  config->set( prefix_ + name_, get() );
}

void EditEnumProperty::loadFromConfig( Config* config )
{
  std::string val;
  if (!config->get( prefix_ + name_, &val, get() ))
  {
    V_string::iterator it = legacy_names_.begin();
    V_string::iterator end = legacy_names_.end();
    for (; it != end; ++it)
    {
      if (config->get( prefix_ + *it, &val, get() ))
      {
        break;
      }
    }
  }

  set( val );
}

void TFFrameProperty::optionCallback( V_string& options_out )
{
  typedef std::vector<std::string> V_string;
  FrameManager::instance()->getTFClient()->getFrameStrings( options_out );
  std::sort(options_out.begin(), options_out.end());

  options_out.insert( options_out.begin(), FIXED_FRAME_STRING );
}

void TFFrameProperty::doWriteToGrid()
{
  EditEnumProperty::doWriteToGrid();

  EditEnumItem* ee_item = dynamic_cast<EditEnumItem*>( widget_item_ );
  ROS_ASSERT( ee_item );
  ee_item->setOptionCallback( boost::bind( &TFFrameProperty::optionCallback, this, _1 ));
}

CategoryProperty::~CategoryProperty()
{
  if( widget_item_ )
  {
    // QTreeWidgetItem's destructor deletes all its children, but
    // PropertyManager also deletes each property (child or not)
    // individually.  Therefore before we destroy a category property
    // we need to disconnect (take) all of the widget item's children,
    // which will then be deleted by their respective Property
    // objects.
    widget_item_->takeChildren();
  }
}

void CategoryProperty::reset()
{
  if( widget_item_ )
  {
    // QTreeWidgetItem's destructor deletes all its children, but
    // PropertyManager also deletes each property (child or not)
    // individually.  Therefore before we destroy a category property
    // we need to disconnect (take) all of the widget item's children,
    // which will then be deleted by their respective Property
    // objects.
    widget_item_->takeChildren();
  }
  Property<bool>::reset(); // manually chain reset() like a virtual destructor
}

void CategoryProperty::setLabel( const std::string& label )
{
  label_ = label;

  if( widget_item_ )
  {
    widget_item_->setLeftText( label_ );
  }
}

void CategoryProperty::expand()
{
  if (widget_item_)
  {
    widget_item_->setExpanded( true );
  }
}

void CategoryProperty::collapse()
{
  if (widget_item_)
  {
    widget_item_->setExpanded( false );
  }
}

void CategoryProperty::doWriteToGrid()
{
  if( !widget_item_ )
  {
    widget_item_ = new PropertyWidgetItem( this, label_, checkbox_, checkbox_, !checkbox_ );
    widget_item_->addToParent();
    widget_item_->setExpanded( true );
  }
  // setData() call must be before any setProperty...() calls, because
  // those can trigger the itemChanged() signal which ultimately
  // causes readFromGrid() to be called, which calls the Setter and
  // clobbers our new data.
  if( checkbox_ )
  {
    widget_item_->setData( 1, Qt::CheckStateRole, get() ? Qt::Checked : Qt::Unchecked );
  }

  setPropertyHelpText( grid_, widget_item_, help_text_ );
}

void CategoryProperty::readFromGrid()
{
  if (checkbox_)
  {
    QVariant check_state = widget_item_->data( 1, Qt::CheckStateRole );
    ROS_ASSERT( !check_state.isNull() );
    bool new_state = (check_state != Qt::Unchecked);
    if( get() != new_state )
    {
      set( new_state );
    }
  }
}

void CategoryProperty::saveToConfig( Config* config )
{
  if (checkbox_)
  {
    config->set( prefix_ + name_, get() );
  }
}

void CategoryProperty::loadFromConfig( Config* config )
{
  if (checkbox_)
  {
    int val;
    if (!config->get( prefix_ + name_, &val, get() ))
    {
      V_string::iterator it = legacy_names_.begin();
      V_string::iterator end = legacy_names_.end();
      for (; it != end; ++it)
      {
        if (config->get( prefix_ + *it, &val, get() ))
        {
          break;
        }
      }
    }

    set( (bool) val );
  }
}

void CategoryProperty::setToOK()
{
  if (grid_)
  {
    setPropertyToColors( grid_, widget_item_, Qt::white, CATEGORY_COLOR);

    if( widget_item_ )
    {
      QFont font = widget_item_->font( 0 );
      font.setBold( true );
      widget_item_->setFont( 0, font );
    }
  }
}

void Vector3Property::doWriteToGrid()
{
  if( !widget_item_ )
  {
    widget_item_ = new CompoundWidgetItem( this, name_, hasSetter() );
    widget_item_->addToParent();
    x_ = new PropertyWidgetItem( this, "X", hasSetter() );
    x_->addToParent( widget_item_ );
    y_ = new PropertyWidgetItem( this, "Y", hasSetter() );
    y_->addToParent( widget_item_ );
    z_ = new PropertyWidgetItem( this, "Z", hasSetter() );
    z_->addToParent( widget_item_ );

    widget_item_->setExpanded( false );
  }
  
  Ogre::Vector3 v = get();
  x_->setUserData( QVariant( v.x ));
  y_->setUserData( QVariant( v.y ));
  z_->setUserData( QVariant( v.z ));

  CompoundWidgetItem* cwi = dynamic_cast<CompoundWidgetItem*>( widget_item_ );
  ROS_ASSERT( cwi );
  cwi->updateText();

  setPropertyHelpText( grid_, widget_item_, help_text_ );
  setPropertyHelpText( grid_, x_, help_text_ );
  setPropertyHelpText( grid_, y_, help_text_ );
  setPropertyHelpText( grid_, z_, help_text_ );
}

void Vector3Property::readFromGrid()
{
  float x = x_->userData().toFloat();
  float y = y_->userData().toFloat();
  float z = z_->userData().toFloat();

  CompoundWidgetItem* cwi = dynamic_cast<CompoundWidgetItem*>( widget_item_ );
  ROS_ASSERT( cwi );
  cwi->updateText();

  set( Ogre::Vector3( x, y, z ));
}

void Vector3Property::saveToConfig( Config* config )
{
  Ogre::Vector3 v = get();

  config->set( prefix_ + name_ + "X", v.x );
  config->set( prefix_ + name_ + "Y", v.y );
  config->set( prefix_ + name_ + "Z", v.z );
}

void Vector3Property::loadFromConfig( Config* config )
{
  Ogre::Vector3 v = get();
  float x, y, z;
  bool found = true;
  found &= config->get( prefix_ + name_ + "X", &x, v.x );
  found &= config->get( prefix_ + name_ + "Y", &y, v.y );
  found &= config->get( prefix_ + name_ + "Z", &z, v.z );

  if (!found)
  {
    V_string::iterator it = legacy_names_.begin();
    V_string::iterator end = legacy_names_.end();
    for (; it != end; ++it)
    {
      found = true;
      found &= config->get( prefix_ + *it + "X", &x, v.x );
      found &= config->get( prefix_ + *it + "Y", &y, v.y );
      found &= config->get( prefix_ + *it + "Z", &z, v.z );

      if (found)
      {
        break;
      }
    }
  }

  set( Ogre::Vector3( x, y, z ) );
}

void Vector3Property::reset()
{
  Property<Ogre::Vector3>::reset();

  // Widget item children of widget_item_ are deleted by their parent,
  // in PropertyBase::reset(), so don't need to be deleted here.
  x_ = 0;
  y_ = 0;
  z_ = 0;
}

void QuaternionProperty::doWriteToGrid()
{
  if( !widget_item_ )
  {
    widget_item_ = new CompoundWidgetItem( this, name_, hasSetter() );
    widget_item_->addToParent();
    x_ = new PropertyWidgetItem( this, "X", hasSetter() );
    x_->addToParent( widget_item_ );
    y_ = new PropertyWidgetItem( this, "Y", hasSetter() );
    y_->addToParent( widget_item_ );
    z_ = new PropertyWidgetItem( this, "Z", hasSetter() );
    z_->addToParent( widget_item_ );
    w_ = new PropertyWidgetItem( this, "W", hasSetter() );
    w_->addToParent( widget_item_ );

    widget_item_->setExpanded( false );
  }
  
  Ogre::Quaternion q = get();
  x_->setUserData( QVariant( q.x ));
  y_->setUserData( QVariant( q.y ));
  z_->setUserData( QVariant( q.z ));
  w_->setUserData( QVariant( q.w ));

  CompoundWidgetItem* cwi = dynamic_cast<CompoundWidgetItem*>( widget_item_ );
  ROS_ASSERT( cwi );
  cwi->updateText();

  setPropertyHelpText( grid_, widget_item_, help_text_ );
  setPropertyHelpText( grid_, x_, help_text_ );
  setPropertyHelpText( grid_, y_, help_text_ );
  setPropertyHelpText( grid_, z_, help_text_ );
  setPropertyHelpText( grid_, w_, help_text_ );
}

void QuaternionProperty::readFromGrid()
{
  float x = x_->userData().toFloat();
  float y = y_->userData().toFloat();
  float z = z_->userData().toFloat();
  float w = w_->userData().toFloat();

  CompoundWidgetItem* cwi = dynamic_cast<CompoundWidgetItem*>( widget_item_ );
  ROS_ASSERT( cwi );
  cwi->updateText();

  set( Ogre::Quaternion( w, x, y, z ));
}

void QuaternionProperty::saveToConfig( Config* config )
{
  Ogre::Quaternion q = get();

  config->set( prefix_ + name_ + "X", q.x );
  config->set( prefix_ + name_ + "Y", q.y );
  config->set( prefix_ + name_ + "Z", q.z );
  config->set( prefix_ + name_ + "W", q.w );
}

void QuaternionProperty::loadFromConfig( Config* config )
{
  Ogre::Quaternion q = get();
  float x, y, z, w;
  bool found = true;
  found &= config->get( prefix_ + name_ + "X", &x, q.x );
  found &= config->get( prefix_ + name_ + "Y", &y, q.y );
  found &= config->get( prefix_ + name_ + "Z", &z, q.z );
  found &= config->get( prefix_ + name_ + "W", &w, q.w );

  if (!found)
  {
    V_string::iterator it = legacy_names_.begin();
    V_string::iterator end = legacy_names_.end();
    for (; it != end; ++it)
    {
      found = true;
      found &= config->get( prefix_ + *it + "X", &x, q.x );
      found &= config->get( prefix_ + *it + "Y", &y, q.y );
      found &= config->get( prefix_ + *it + "Z", &z, q.z );
      found &= config->get( prefix_ + *it + "W", &w, q.w );

      if (found)
      {
        break;
      }
    }
  }

  set( Ogre::Quaternion( w, x, y, z ) );
}

void QuaternionProperty::reset()
{
  Property<Ogre::Quaternion>::reset();

  // Widget item children of widget_item_ are deleted by their parent,
  // in PropertyBase::reset(), so don't need to be deleted here.
  x_ = 0;
  y_ = 0;
  z_ = 0;
  w_ = 0;
}

}
