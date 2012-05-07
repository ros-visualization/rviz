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

#include <QApplication>
#include <QPushButton>
#include <QVBoxLayout>
#include <QFileDialog>

#include <fstream>
#include <sstream>

#include <yaml-cpp/yaml.h>

#include "rviz/properties/color_property.h"
#include "rviz/properties/property.h"
#include "rviz/properties/quaternion_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/properties/property_tree_model.h"
#include "rviz/properties/property_tree_widget.h"
#include "rviz/properties/status_list.h"
#include "rviz/display.h"
#include "rviz/display_group.h"
#include "rviz/visualization_manager.h"

#include "fun_display.h"
#include "mock_display_factory.h"

#include "playground.h"

void makeBigTree( DisplayGroup* parent )
{
  Display* disp;
  for( int i = 0; i < 100; i++ )
  {
    disp = new FunDisplay();
    disp->setName( "D" + QString::number( i ));
    disp->setParentProperty( parent );

    for( int j = 0; j < 100; j++ )
    {
      new Property( "prop" + QString::number( j ), j, "how many", disp );
    }
  }
}

Playground::Playground( QWidget* parent )
  : QWidget( parent )
{
  factory_ = new MockDisplayFactory;
  vis_man_ = new VisualizationManager;
  vis_man_->setDisplayFactory( factory_ );

  root_ = new DisplayGroup;
  model_ = new PropertyTreeModel( root_ );
/*
  std::stringstream input(
    "Enabled: true\n"
    "Displays:\n"
    " -\n"
    "   Class: MockDisplay\n"
    "   Name: Steven\n"
    "   Enabled: false\n"
    "   Count: 17\n"
    " -\n"
    "   Name: sub group\n"
    "   Class: DisplayGroup\n"
    "   Enabled: true\n"
    "   Displays:\n"
    "    -\n"
    "      Class: MockDisplay\n"
    "      Name: Curly\n"
    "      Enabled: false\n"
    "      Count: 900\n"
    " -\n"
    "   Class: BrokenDisplay\n"
    "   Name: Joe\n"
    "   Enabled: true\n"
    "   Count: 33\n"
    );

  YAML::Parser parser( input );
  YAML::Node node;
  parser.GetNextDocument( node );

  root_->initialize( vis_man_ );
  root_->load( node );
*/

  Display* display = new Display();
  display->setName( "Test" );
  display->setParentProperty( root_ );
  new Property( "Alpha", 0.5, "The amount of transparency to apply to the grid lines.", display );
  new Property( "Beta Band", 10, "The number of betas to apply to the grid lines.", display );
  new Property( "Gamma Topic", "chubby", "The topic on which to listen for Gamma messages.", display );
  Property* position = new Property( "Position", QVariant(), "Position of the chub.", display );
  new Property( "X", 1.1, "X component of the position of the chub.", position );
  new Property( "Y", 0.717, "Y component of the position of the chub.", position );
  DisplayGroup* group = new DisplayGroup();
  group->setName( "Group" );
  group->setParentProperty( root_ );
  DisplayGroup* sub_group = new DisplayGroup();
  sub_group->setName( "SubGroup" );
  sub_group->setParentProperty( group );
  Display* display2 = new Display();
  display2->setName( "Taste" );
  display2->setParentProperty( group );
  DisplayGroup* cruft = new DisplayGroup();
  cruft->setName( "Cruft" );
  cruft->setParentProperty( root_ );
  new VectorProperty( "Vector", Ogre::Vector3( 1, 2, 3 ), "3 numbers in a row.", display2 );
  new QuaternionProperty( "Quaternion", Ogre::Quaternion( 1, 0, 0, 0 ), "4 numbers in a row!", display2 );
  Display* display3 = new Display();
  display3->setName( "Toast" );
  display3->setParentProperty( group );
  new ColorProperty( "Color", QColor( 1, 2, 3 ), "Better than black and white.", display3 );
  Display* display4 = new Display();
  display4->setName( "Lamby" );
  display4->setParentProperty( group );
  StatusList* stat = new StatusList( display4 );
  stat->setStatus( StatusProperty::Warn, "Topic", "No messages received." );
  stat->setStatus( StatusProperty::Error, "Size", "Size is too large." );
  stat->setStatus( StatusProperty::Ok, "Cheese", "Cheese is tasty." );
  stat = new StatusList( display );
  stat->setStatus( StatusProperty::Warn, "Topic", "No messages received." );
  stat->setStatus( StatusProperty::Error, "Size", "Size is too large." );
  stat->setStatus( StatusProperty::Ok, "Cheese", "Cheese is tasty." );

  FunDisplay* fun = new FunDisplay();
  fun->setName( "Fun" );
  fun->setParentProperty( group );

  fun = new FunDisplay();
  fun->setName( "Funner" );
  fun->setParentProperty( group );

  PropertyTreeWidget* w = new PropertyTreeWidget;
  w->setModel( model_ );

//  makeBigTree( cruft );

  QPushButton* load_button = new QPushButton( "load" );
  connect( load_button, SIGNAL( clicked() ), this, SLOT( openLoadFileDialog() ));

  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addWidget( w );
  main_layout->addWidget( load_button );

  setLayout( main_layout );

  setWindowTitle("fun");
}

void Playground::openLoadFileDialog()
{
  QString filename = QFileDialog::getOpenFileName( this );
  std::ifstream in( filename.toStdString().c_str() );
  if( in )
  {
    YAML::Parser parser( in );
    YAML::Node node;
    parser.GetNextDocument( node );
    root_->initialize( vis_man_ );
    root_->load( node );
  }
  else
  {
    printf("Failed to read file %s.\n", qPrintable( filename ));
  }
}

int main( int argc, char **argv )
{
  QApplication app( argc, argv );

  Playground w;
  w.resize( 400, 600 );
  w.show();

  return app.exec();
}
