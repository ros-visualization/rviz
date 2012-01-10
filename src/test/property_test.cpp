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

#include <QtGui/QApplication>

#include <OGRE/OgreVector3.h>

#include "ros/ros.h"

#include "rviz/properties/property_manager.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_tree_widget.h"
#include "rviz/properties/property_tree_with_help.h"
#include "rviz/frame_manager.h"
#include "ros_spinner.h"

using namespace rviz;

PropertyManager* pm;

class MyPropertyReceiver
{
private:
  bool b_;
  int i_;
  int e_;
  float f_;
  std::string s_;
  std::string t_;
  std::string ee_;
  std::string tf_;
  int index_;
  StatusPropertyPtr sp_;
  Color c_;
  Ogre::Vector3 v_;
  Ogre::Quaternion q_;

public:
  MyPropertyReceiver( int index ): index_( index ) {}
  void setBool( bool value ) {
    b_ = value;
    printf("%d bool = %s\n", index_, b_?"true":"false");
    if( !b_ && sp_ ) {
      sp_->deleteStatus( "Bread" );
      pm->update();
    }
  }
  void setColor( Color value ) { c_ = value; printf("%d color = %.3f, %.3f, %.3f\n", index_, c_.r_, c_.g_, c_.b_); }
  void setInt( int value ) { i_ = value; printf("%d int = %d\n", index_, i_); }
  void setEnum( int value ) { e_ = value; printf("%d enum = %d\n", index_, e_); }
  void setFloat( float value ) { f_ = value; printf("%d float = %f\n", index_, f_); }
  void setString( std::string value ) { s_ = value; printf("%d string = %s\n", index_, s_.c_str()); }
  void setEE( std::string value ) { ee_ = value; printf("%d edit enum = %s\n", index_, ee_.c_str()); }
  void setTF( std::string value ) { tf_ = value; printf("%d tf frame = %s\n", index_, tf_.c_str()); }
  void setTopic( std::string value ) { t_ = value; printf("%d topic = %s\n", index_, t_.c_str()); }
  void setVector3( Ogre::Vector3 value ) { v_ = value; printf("%d vector3 = %f, %f, %f\n", index_, v_.x, v_.y, v_.z); }
  void setQuaternion( Ogre::Quaternion value ) { q_ = value; printf("%d quaternion = %f, %f, %f, %f\n", index_, q_.x, q_.y, q_.z, q_.w); }

  bool getBool() { return b_; }
  Color getColor() { return c_; }
  int getInt() { return i_; }
  int getEnum() { return e_; }
  float getFloat() { return f_; }
  std::string getString() { return s_; }
  std::string getEE() { return ee_; }
  std::string getTF() { return tf_; }
  std::string getTopic() { return t_; }
  Ogre::Vector3 getVector3() { return v_; }
  Ogre::Quaternion getQuaternion() { return q_; }

  void setStatusProperty( StatusPropertyPtr sp ) { sp_ = sp; }

  void optionCallback( V_string& options_out )
  {
    options_out.push_back( "Callee Pays" );
    options_out.push_back( "Caller Pays" );
    options_out.push_back( "Phone Company Pays" );
    options_out.push_back( "Taxpayers Pay" );
  }
};

int main(int argc, char **argv)
{
  ros::init( argc, argv, "property_test" );

  // The frame manager needs to exist in order for the TFFrameProperty
  // to be able to find its list of TF frames.
  FrameManagerPtr frame_manager = FrameManager::instance();

  QApplication app(argc, argv);

  RosSpinner spinner;

  MyPropertyReceiver recvr( 1 );
  MyPropertyReceiver recvr2( 2 );
  MyPropertyReceiver recvr3( 3 );

  pm = new PropertyManager();
  {
    CategoryPropertyWPtr options_category = pm->createCategory( ".Global Options", "",
                                                                CategoryPropertyWPtr(), NULL );
    BoolPropertyWPtr bool_prop =
      pm->createProperty<BoolProperty>( "Lumpy", "",
                                        boost::bind( &MyPropertyReceiver::getBool, &recvr ),
                                        boost::bind( &MyPropertyReceiver::setBool, &recvr, _1 ),
                                        options_category, NULL );
    setPropertyHelpText(bool_prop, "A sample boolean property.");
    BoolPropertyPtr bp = bool_prop.lock();
    bp->set( true );

    StringPropertyWPtr string_prop =
      pm->createProperty<StringProperty>( "Style", "",
                                          boost::bind( &MyPropertyReceiver::getString, &recvr ),
                                          boost::bind( &MyPropertyReceiver::setString, &recvr, _1 ),
                                          options_category, NULL );
    setPropertyHelpText(string_prop, "A sample stringean property.");
    StringPropertyPtr sp = string_prop.lock();
    sp->set( "nubby" );

    ColorPropertyWPtr color_prop =
      pm->createProperty<ColorProperty>( "Cheese Color", "",
                                          boost::bind( &MyPropertyReceiver::getColor, &recvr ),
                                          boost::bind( &MyPropertyReceiver::setColor, &recvr, _1 ),
                                          options_category, NULL );
    setPropertyHelpText(color_prop, "A sample color property.");
    ColorPropertyPtr cp = color_prop.lock();
    cp->set( Color( .1f, .4f, 1.0f ));

    EnumPropertyWPtr enum_prop =
      pm->createProperty<EnumProperty>( "Cheese Enum", "",
                                        boost::bind( &MyPropertyReceiver::getEnum, &recvr ),
                                        boost::bind( &MyPropertyReceiver::setEnum, &recvr, _1 ),
                                        options_category, NULL );
    setPropertyHelpText(enum_prop, "A sample enum property.");
    EnumPropertyPtr ep = enum_prop.lock();
    ep->addOption( "cheddar", 6 );
    ep->addOption( "swiss", 7 );
    ep->addOption( "Humboldt Fog", 99 );
    ep->addOption( "chester", -1 );
    ep->addOption( "blubber", 0 );
    ep->set( 7 );

    TFFramePropertyWPtr tf_prop =
      pm->createProperty<TFFrameProperty>( "Picture Frame", "",
                                           boost::bind( &MyPropertyReceiver::getTF, &recvr ),
                                           boost::bind( &MyPropertyReceiver::setTF, &recvr, _1 ),
                                           options_category, NULL );
    setPropertyHelpText(tf_prop, "A sample tf frame property.");
    TFFramePropertyPtr tfp = tf_prop.lock();
    tfp->set( "base_link" );

    EditEnumPropertyWPtr ee_prop =
      pm->createProperty<EditEnumProperty>( "Chub Enum", "",
                                            boost::bind( &MyPropertyReceiver::getEE, &recvr ),
                                            boost::bind( &MyPropertyReceiver::setEE, &recvr, _1 ),
                                            options_category, NULL );
    setPropertyHelpText(ee_prop, "A sample editable enum property.");
    EditEnumPropertyPtr eep = ee_prop.lock();
    eep->addOption( "Tubby" );
    eep->addOption( "Portly" );
    eep->addOption( "Chunky" );
    eep->addOption( "Chunkstyle" );
    eep->addOption( "Round" );
    eep->set( "Portly" );

    IntPropertyWPtr int_prop =
      pm->createProperty<IntProperty>( "Planet Number", "",
                                       boost::bind( &MyPropertyReceiver::getInt, &recvr2 ),
                                       boost::bind( &MyPropertyReceiver::setInt, &recvr2, _1 ),
                                       options_category, NULL );
    setPropertyHelpText(int_prop, "The number of the planet, from Mercury (1) to Pluto (9)  For more information, see <a href=\"http://en.wikipedia.org/wiki/Solar_System\">The Solar System</a>.");
    IntPropertyPtr ip = int_prop.lock();
    ip->set( 3 );
    ip->setMax( 9 );
    ip->setMin( 1 );

    FloatPropertyWPtr float_prop =
      pm->createProperty<FloatProperty>( "Alpha", "",
                                         boost::bind( &MyPropertyReceiver::getFloat, &recvr2 ),
                                         boost::bind( &MyPropertyReceiver::setFloat, &recvr2, _1 ),
                                         options_category, NULL );
    setPropertyHelpText(float_prop, "Opacity.");
    FloatPropertyPtr fp = float_prop.lock();
    fp->set( 0.5 );
    fp->setMax( 1.0 );
    fp->setMin( 0 );
  }
  {
    CategoryPropertyWPtr options_category = pm->createCheckboxCategory( "Pedestrian", "", "",
                                                                        boost::bind( &MyPropertyReceiver::getBool, &recvr2 ),
                                                                        boost::bind( &MyPropertyReceiver::setBool, &recvr2, _1 ));
    StatusPropertyWPtr stat_prop = pm->createStatus( "Options Status", "prop_prefix", options_category );
    {
      StatusPropertyPtr sp = stat_prop.lock();
      sp->setStatus( status_levels::Error, "Bread", "moldy" );
      sp->setStatus( status_levels::Ok, "Orange Juice", "tasty" );
      sp->setStatus( status_levels::Warn, "Cheese", "hard and crusty" );
      sp->setStatus( status_levels::Warn, "Lemons", "too tangy" );
      recvr.setStatusProperty( sp );
    }
    ROSTopicStringPropertyWPtr topic_prop =
      pm->createProperty<ROSTopicStringProperty>( "Topic", "",
                                                  boost::bind( &MyPropertyReceiver::getTopic, &recvr3 ),
                                                  boost::bind( &MyPropertyReceiver::setTopic, &recvr3, _1 ),
                                                  options_category, NULL );
    setPropertyHelpText(topic_prop, "What topic to listen to.");
    ROSTopicStringPropertyPtr tp = topic_prop.lock();
    tp->setMessageType( "std_msgs/String" );
    tp->set( "chatter" );

    BoolPropertyWPtr bool_prop =
      pm->createProperty<BoolProperty>( "Active", "",
                                        boost::bind( &MyPropertyReceiver::getBool, &recvr3 ),
                                        boost::bind( &MyPropertyReceiver::setBool, &recvr3, _1 ),
                                        options_category, NULL );
    setPropertyHelpText(bool_prop, "A sample boolean property.");
    BoolPropertyPtr bp = bool_prop.lock();
    bp->set( true );

    StringPropertyWPtr string_prop =
      pm->createProperty<StringProperty>( "Cheese type", "",
                                          boost::bind( &MyPropertyReceiver::getString, &recvr3 ),
                                          boost::bind( &MyPropertyReceiver::setString, &recvr3, _1 ),
                                          options_category, NULL );
    setPropertyHelpText(string_prop, "A sample stringean property.");
    StringPropertyPtr sp = string_prop.lock();
    sp->set( "nubby" );

    IntPropertyWPtr int_prop =
      pm->createProperty<IntProperty>( "NumLemons", "",
                                       boost::bind( &MyPropertyReceiver::getInt, &recvr3 ),
                                       boost::bind( &MyPropertyReceiver::setInt, &recvr3, _1 ),
                                       options_category, NULL );
    setPropertyHelpText(int_prop, "Number of lemons in a basket.");
    IntPropertyPtr ip = int_prop.lock();
    ip->set( 12345 );

    FloatPropertyWPtr float_prop =
      pm->createProperty<FloatProperty>( "Size", "",
                                         boost::bind( &MyPropertyReceiver::getFloat, &recvr3 ),
                                         boost::bind( &MyPropertyReceiver::setFloat, &recvr3, _1 ),
                                         options_category, NULL );
    setPropertyHelpText(float_prop, "Size of the basket in furlongs.");
    FloatPropertyPtr fp = float_prop.lock();
    fp->set( 1.398 );

    EditEnumPropertyWPtr ee_prop =
      pm->createProperty<EditEnumProperty>( "Callback Enum", "",
                                            boost::bind( &MyPropertyReceiver::getEE, &recvr3 ),
                                            boost::bind( &MyPropertyReceiver::setEE, &recvr3, _1 ),
                                            options_category, NULL );
    setPropertyHelpText(ee_prop, "A sample editable enum property.");
    EditEnumPropertyPtr eep = ee_prop.lock();
    eep->setOptionCallback( boost::bind(&MyPropertyReceiver::optionCallback, &recvr3, _1 ));
    eep->set( "Caller pays" );

    Vector3PropertyWPtr vector3_prop =
      pm->createProperty<Vector3Property>( "Location", "",
                                           boost::bind( &MyPropertyReceiver::getVector3, &recvr3 ),
                                           boost::bind( &MyPropertyReceiver::setVector3, &recvr3, _1 ),
                                           options_category, NULL );
    setPropertyHelpText(vector3_prop, "Location of the basket in furlongs.");
    Vector3PropertyPtr vp = vector3_prop.lock();
    vp->set( Ogre::Vector3( 1.1, 2.2, 3.3 ));

    QuaternionPropertyWPtr quat_prop =
      pm->createProperty<QuaternionProperty>( "Orientation", "",
                                              boost::bind( &MyPropertyReceiver::getQuaternion, &recvr3 ),
                                              boost::bind( &MyPropertyReceiver::setQuaternion, &recvr3, _1 ),
                                              options_category, NULL );
    setPropertyHelpText(quat_prop, "Orientation of the basket.");
    QuaternionPropertyPtr qp = quat_prop.lock();
    qp->set( Ogre::Quaternion( 1.1, 2.2, 3.3, 4.4 ));

  }

  PropertyTreeWithHelp tree_widget;
  pm->setPropertyTreeWidget( tree_widget.getTree() );

  tree_widget.show();

  int ret = app.exec();

  delete pm;

  return ret;
}
