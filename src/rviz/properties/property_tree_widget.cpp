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

#include <QTimer>
#include <QStyledItemDelegate>
#include <QLineEdit>
#include <QPainter>
#include <QMouseEvent>

#include "rviz/properties/property_tree_widget.h"
#include "rviz/properties/property_widget_item.h"
#include "rviz/properties/property.h"
#include "rviz/properties/topic_info_variant.h"
#include "rviz/properties/ros_topic_editor.h"
#include "rviz/properties/color_editor.h"
#include "rviz/uniform_string_stream.h"

namespace rviz
{

////////////////////////////////////////////////////////////////////////
// Delegate for handling items.
////////////////////////////////////////////////////////////////////////
class PropertyTreeDelegate: public QStyledItemDelegate
{
private:
  PropertyTreeWidget* tree_widget_;

public:
  PropertyTreeDelegate( PropertyTreeWidget* tree_widget )
    : QStyledItemDelegate( tree_widget )
    , tree_widget_( tree_widget )
  {
  }

  virtual void paint( QPainter * painter,
                      const QStyleOptionViewItem & option,
                      const QModelIndex & index ) const
  {
    PropertyWidgetItem* item = tree_widget_->getItem( index );
    if( index.column() == 0 || !item->paint( painter, option ))
    {
      QStyledItemDelegate::paint( painter, option, index );
    }
  }

  virtual QWidget *createEditor( QWidget *parent,
                                 const QStyleOptionViewItem & option,
                                 const QModelIndex &index ) const
  {
    if( index.column() != 1 ) // only right column (#1) is editable.
    {
      return 0;
    }
    PropertyWidgetItem* item = tree_widget_->getItem( index );
    if( !item )
    {
      return 0;
    }
    QWidget* editor = item->createEditor( parent, option );
    if( editor != 0 )
    {
      if( LineEditWithButton* lewb = qobject_cast<LineEditWithButton*>( editor ))
      {
        tree_widget_->connect( lewb, SIGNAL( startPersistence() ), tree_widget_, SLOT( startPersistCurrent() ));
        tree_widget_->connect( lewb, SIGNAL( endPersistence() ), tree_widget_, SLOT( endPersistCurrent() ));
      }
      return editor;
    }

    // TODO: clean up the rest of this function.  I think ideally the
    // rest of this function would go out to subclasses of
    // PropertyWidgetItem, as is done with EnumItems and
    // EditEnumItems.  Same for setEditorData() and setModelData().

    QVariant originalValue = index.model()->data(index, Qt::UserRole);

    if( originalValue.canConvert<ros::master::TopicInfo>() )
    {
      RosTopicEditor* editor = new RosTopicEditor( parent );
      tree_widget_->connect( editor, SIGNAL( startPersistence() ), tree_widget_, SLOT( startPersistCurrent() ));
      tree_widget_->connect( editor, SIGNAL( endPersistence() ), tree_widget_, SLOT( endPersistCurrent() ));
      editor->setFrame( false );
      return editor;
    }
    else
    {
      QValidator *validator;
      QLineEdit *lineEdit = new QLineEdit(parent);

      switch (originalValue.type()) {
      case QVariant::Int:
      {
        validator = new QIntValidator( (int) item->min_, (int) item->max_, lineEdit );
        break;
      }
      case QMetaType::Float:
      {
        validator = new QDoubleValidator( item->min_, item->max_, 1000, lineEdit );
        break;
      }
      default:
        validator = NULL;
      }
      lineEdit->setValidator( validator );
      lineEdit->setFrame( false );
      return lineEdit;
    }
  }
  // see qt-examples/tools/settingseditor/variantdelegate.cpp for a more elaborate example.

  // Called to transfer data from model to editor
  void setEditorData(QWidget *editor, const QModelIndex &index) const
  {
    if( index.column() == 1 )
    {
      PropertyWidgetItem* item = tree_widget_->getItem( index );
      if( !item || item->setEditorData( editor ))
      {
        return;
      }
    }
    
    QVariant value = index.model()->data(index, Qt::UserRole);

    if( RosTopicEditor* topic_editor = qobject_cast<RosTopicEditor*>( editor ))
    {
      topic_editor->setTopic( value.value<ros::master::TopicInfo>() );
    }
    else if( QLineEdit *lineEdit = qobject_cast<QLineEdit *>( editor ))
    {
      lineEdit->setText(value.toString());
    }
  }

  // Called to transfer data from editor to model.
  void setModelData( QWidget *editor, QAbstractItemModel *model,
                     const QModelIndex &index ) const
  {
    PropertyWidgetItem* item = tree_widget_->getItem( index );
    if( !item || item->setModelData( editor ))
    {
      return;
    }

    QVariant originalValue = index.model()->data( index, Qt::UserRole );
    QVariant value;
    QString display_string;

    if( RosTopicEditor* topic_editor = qobject_cast<RosTopicEditor*>( editor ))
    {
      if( !topic_editor->isModified() )
      {
        return;
      }
      ros::master::TopicInfo topic = topic_editor->getTopic();
      value = QVariant::fromValue( topic );
      display_string = QString::fromStdString( topic.name );
    }
    else if( QLineEdit* lineEdit = qobject_cast<QLineEdit *>( editor ))
    {
      if( !lineEdit->isModified() )
      {
        return;
      }

      QString text = lineEdit->text();
      const QValidator *validator = lineEdit->validator();
      if( validator ) {
        int pos;
        if( validator->validate( text, pos ) != QValidator::Acceptable )
        {
          return;
        }
      }

      switch (originalValue.type()) {
//    case QVariant::Char:
//      value = text.at(0);
//      break;
//    case QVariant::Color:
//      colorExp.exactMatch(text);
//      value = QColor(qMin(colorExp.cap(1).toInt(), 255),
//                     qMin(colorExp.cap(2).toInt(), 255),
//                     qMin(colorExp.cap(3).toInt(), 255),
//                     qMin(colorExp.cap(4).toInt(), 255));
//      break;
//    case QVariant::Date:
//    {
//      QDate date = QDate::fromString(text, Qt::ISODate);
//      if (!date.isValid())
//        return;
//      value = date;
//    }
//    break;
//    case QVariant::DateTime:
//    {
//      QDateTime dateTime = QDateTime::fromString(text, Qt::ISODate);
//      if (!dateTime.isValid())
//        return;
//      value = dateTime;
//    }
//    break;
//    case QVariant::Point:
//      pointExp.exactMatch(text);
//      value = QPoint(pointExp.cap(1).toInt(), pointExp.cap(2).toInt());
//      break;
//    case QVariant::Rect:
//      rectExp.exactMatch(text);
//      value = QRect(rectExp.cap(1).toInt(), rectExp.cap(2).toInt(),
//                    rectExp.cap(3).toInt(), rectExp.cap(4).toInt());
//      break;
//    case QVariant::Size:
//      sizeExp.exactMatch(text);
//      value = QSize(sizeExp.cap(1).toInt(), sizeExp.cap(2).toInt());
//      break;
//    case QVariant::StringList:
//      value = text.split(",");
//      break;
//    case QVariant::Time:
//    {
//      QTime time = QTime::fromString(text, Qt::ISODate);
//      if (!time.isValid())
//        return;
//      value = time;
//    }
//    break;
      default: // int, float, string, etc which are parsed nicely by QVariant.
        value = text;
        value.convert( originalValue.type() );
      }
      display_string = value.toString();
    }

    // Store the final value variant in the model's UserRole.
    model->setData(index, value, Qt::UserRole);

    // Copy the display string into the DisplayRole so the default painter shows the right stuff.
    bool ign = tree_widget_->setIgnoreChanges( true );
    model->setData(index, display_string, Qt::DisplayRole);
    tree_widget_->setIgnoreChanges( ign );
  }
};

////////////////////////////////////////////////////////////////////////
// Splitter widget class
////////////////////////////////////////////////////////////////////////
class SplitterHandle: public QWidget
{
public:
  SplitterHandle( PropertyTreeWidget* parent = 0 )
  : QWidget( parent )
  , parent_( parent )
  , first_column_size_ratio_( 0.5f )
  {
    setCursor( Qt::SplitHCursor );
    int w = 7;
    setGeometry( parent_->width() / 2 - w/2, 0, w, parent_->height() );
  }

  void onParentResized()
  {
    int new_column_width = int( first_column_size_ratio_ * parent_->width() );
    parent_->setColumnWidth( 0, new_column_width );
    setGeometry( new_column_width - width() / 2, 0, width(), parent_->height() );
  }

  void setRatio( float ratio )
  {
    first_column_size_ratio_ = ratio;
    onParentResized();
  }

  float getRatio()
  {
    return first_column_size_ratio_;
  }

protected:
  void mousePressEvent( QMouseEvent* event )
  {
    if( event->button() == Qt::LeftButton )
    {
      x_press_offset_ = event->x();
    }
  }

  void mouseMoveEvent( QMouseEvent* event )
  {
    int padding = 55;

    if( event->buttons() & Qt::LeftButton )
    {
      QPoint pos_rel_parent = parent_->mapFromGlobal( event->globalPos() );

      int new_x = pos_rel_parent.x() - x_press_offset_;

      if( new_x > parent_->width() - width() - padding )
      {
        new_x = parent_->width() - width() - padding;
      }

      if( new_x < padding )
      {
        new_x = padding;
      }

      if( new_x != x() )
      {
        move( new_x, 0 );

        int new_column_width = new_x + width() / 2;
        parent_->setColumnWidth( 0, new_column_width );

        first_column_size_ratio_ = new_column_width / (float) parent_->width();
      }
    }
  }

private:
  PropertyTreeWidget* parent_;
  int x_press_offset_;

  /** The ratio of the first column width to the entire widget width.
   * Preserved during parent widget resize. */
  float first_column_size_ratio_;
};

////////////////////////////////////////////////////////////////////////
// Main class functions
////////////////////////////////////////////////////////////////////////
PropertyTreeWidget::PropertyTreeWidget( QWidget* parent )
  : QTreeWidget( parent )
  , ignore_changes_( false )
  , splitter_handle_( new SplitterHandle( this ))
  , persisted_item_( 0 )
{
  setHorizontalScrollBarPolicy( Qt::ScrollBarAlwaysOff );
  setHeaderHidden( true );
  setUniformRowHeights( true );
  setItemDelegate( new PropertyTreeDelegate( this ));
  setEditTriggers( QAbstractItemView::AllEditTriggers );
  setColumnCount( 2 );
  setSelectionMode( QAbstractItemView::ExtendedSelection );

  connect( this, SIGNAL( itemChanged( QTreeWidgetItem *, int )),
           this, SLOT( onItemChanged( QTreeWidgetItem *, int )));
}

void PropertyTreeWidget::onItemChanged( QTreeWidgetItem* item, int column_number )
{
  if( !ignore_changes_ )
  {
    PropertyWidgetItem* pwi = dynamic_cast<PropertyWidgetItem*>( item );
    if( pwi )
    {
      pwi->getProperty()->readFromGrid();
    }
  }
}

PropertyWidgetItem* PropertyTreeWidget::getItem( const QModelIndex & index )
{
  return dynamic_cast<PropertyWidgetItem*>( itemFromIndex( index ));
}

void PropertyTreeWidget::resizeEvent( QResizeEvent* event )
{
  splitter_handle_->onParentResized();
}

void PropertyTreeWidget::dropEvent( QDropEvent* event )
{
  QTreeWidget::dropEvent( event );
  Q_EMIT orderChanged();
}

/** Return a string storing the expanded-or-not state of each item,
 * and the splitter position. */
std::string PropertyTreeWidget::saveEditableState()
{
  UniformStringStream output;
  
  bool first = true;
  saveExpandedState( output, invisibleRootItem(), first );

  output << ";splitterratio=" << splitter_handle_->getRatio();

  return output.str();

// Here's what the old rviz saved here:
//
// Property\ Grid\ State=selection=Fixed Frame;expanded=.Global Options,TF.Enabled.TF.StatusTopStatus,TF.Enabled.TF.Tree,MarkerArray.Enabled,MarkerArray.Enabled.MarkerArray.Namespaces;scrollpos=0,0;splitterpos=142,301;ispageselected=1
//
// I'm explicitly not saving what is selected - I don't think it is
// important, and I'm not attempting to be compatible with old files.
}

void PropertyTreeWidget::saveExpandedState( std::ostream& output,
                                            QTreeWidgetItem* parent_item,
                                            bool& first )
{
  for( int child_index = 0; child_index < parent_item->childCount(); child_index++ )
  {
    QTreeWidgetItem* item = parent_item->child( child_index );
    if( item->isExpanded() && item->childCount() > 0 )
    {
      if( first )
      {
        output << "expanded=";
        first = false;
      }
      else
      {
        output << ',';
      }
      PropertyWidgetItem* pwi = dynamic_cast<PropertyWidgetItem*>( item );
      if( pwi )
      {
        output << pwi->getProperty()->getPrefix() << pwi->getProperty()->getName();
        saveExpandedState( output, item, first );
      }
    }
  }
}

/** Restore state from a string previously returned by saveEditableState(). */
void PropertyTreeWidget::restoreEditableState( const std::string& state )
{
  UniformStringStream iss( state );
  std::string assignment;
  while( std::getline( iss, assignment, ';' ))
  {
    size_t equal_pos = assignment.find( '=' );
    if( equal_pos != std::string::npos )
    {
      UniformStringStream value_stream( assignment.substr( equal_pos + 1 ));
      if( 0 == assignment.compare( 0, equal_pos, "splitterratio" ))
      {
        float ratio = 0.5;
        value_stream.parseFloat( ratio );
        splitter_handle_->setRatio( ratio );
      }
      else if( 0 == assignment.compare( 0, equal_pos, "expanded" ))
      {
        std::set<std::string> expanded_entries;
        std::string entry;
        while( std::getline( value_stream, entry, ',' ))
        {
          expanded_entries.insert( entry );
        }

        restoreExpandedState( expanded_entries, invisibleRootItem() );
      }
    }
  }
}

/** Recursive function to iterate through tree of items, reading
 * input as it goes. */
void PropertyTreeWidget::restoreExpandedState( const std::set<std::string>& expanded_entries,
                                               QTreeWidgetItem* parent_item )
{
  // Caveat: this runs before data comes in from subscriptions which
  // actually sets up some of the properties (like TF frame entries),
  // so the expanded state of those are not properly restored.
  for( int child_index = 0; child_index < parent_item->childCount(); child_index++ )
  {
    QTreeWidgetItem* item = parent_item->child( child_index );
    PropertyWidgetItem* pwi = dynamic_cast<PropertyWidgetItem*>( item );
    if( pwi )
    {
      std::string entry_name = pwi->getProperty()->getPrefix() + pwi->getProperty()->getName();
      if( expanded_entries.find( entry_name ) != expanded_entries.end() )
      {
        item->setExpanded( true );
        if( item->childCount() > 0 )
        {
          restoreExpandedState( expanded_entries, item );
        }
      }
      else
      {
        item->setExpanded( false );
      }
    }
  }
}

void PropertyTreeWidget::startPersistCurrent()
{
  persisted_item_ = currentItem();
  openPersistentEditor( persisted_item_, 1 );
}

void PropertyTreeWidget::endPersistCurrent()
{
  if( persisted_item_ )
  {
    QWidget* editor = itemWidget( persisted_item_, 1 );
    if( editor )
    {
      commitData( editor );
    }
    closePersistentEditor( persisted_item_, 1 );

    // Not sure why this is necessary.  Without this, the
    // QAbstractItemView (an ancestor class of this) stays in its
    // "Editing" state after the above closePersistentEditor() call,
    // leading to a broken state where clicking on editable items does
    // not open their editors.
    //
    // QAbstractItemView::dragLeaveEvent() does an unconditional
    // setState(NoState) internally, which fixes the problem.
    dragLeaveEvent( NULL );
  }
}

} // end namespace rviz
