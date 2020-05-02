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

#include <gtest/gtest.h>

#include <QApplication>

#include <ros/ros.h>

#include <sstream>

#include <rviz/properties/vector_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/display_group.h>
#include <rviz/config.h>
#include <rviz/yaml_config_reader.h>
#include <rviz/yaml_config_writer.h>

#include "mock_display.h"
#include "mock_context.h"

using namespace rviz;

TEST(Display, load_properties)
{
  std::stringstream input("Name: sample\n"
                          "Enabled: true\n"
                          "Count: 7\n"
                          "Pi: 3.2\n"
                          "Offset: {X: -1, Y: 1.1, Z: 1.1e3}\n"
                          "Color: white\n"
                          "Style: loosey goosey\n");

  rviz::YamlConfigReader reader;
  rviz::Config config;
  reader.readStream(config, input);

  MockDisplay d;
  d.load(config);

  EXPECT_EQ(7, d.count_->getValue().toInt());
  EXPECT_EQ("loosey goosey", d.style_->getValue().toString().toStdString());
  EXPECT_EQ(3.2f, d.pi_->getValue().toFloat());
  Ogre::Vector3 offset = d.offset_->getVector();
  EXPECT_EQ(-1.f, offset.x);
  EXPECT_EQ(1.1f, offset.y);
  EXPECT_EQ(1100.f, offset.z);
  EXPECT_EQ("255; 255; 255", d.color_->getValue().toString().toStdString());
  EXPECT_EQ(true, d.getValue().toBool());
}

TEST(DisplayGroup, load_properties)
{
  std::stringstream input("Name: root\n"
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
                          "   Count: 33\n");

  rviz::YamlConfigReader reader;
  rviz::Config config;
  reader.readStream(config, input);

  DisplayGroup g;
  g.load(config);

  EXPECT_EQ(true, g.getValue().toBool());
  EXPECT_EQ(false, g.subProp("Steven")->getValue().toBool());
  EXPECT_EQ(17, g.subProp("Steven")->subProp("Count")->getValue().toInt());
  EXPECT_EQ(900, g.subProp("sub group")->subProp("Curly")->subProp("Count")->getValue().toInt());
  EXPECT_EQ("The class required for this display, 'BrokenDisplay', could not be loaded.",
            g.subProp("Joe")->getDescription().left(74).toStdString());
}

TEST(Display, save_properties)
{
  MockDisplay d;
  d.setName("Steven");
  d.subProp("Count")->setValue(37);

  rviz::YamlConfigWriter writer;
  rviz::Config config;
  d.save(config);
  QString out = writer.writeString(config);

  // Since we instantiated the display directly instead of using the
  // DisplayFactory, it won't know its class name.
  EXPECT_EQ(std::string("Class: \"\"\n"
                        "Name: Steven\n"
                        "Enabled: false\n"
                        "Count: 37\n"
                        "Style: chunky\n"
                        "Pi: 3.14159\n"
                        "Offset: {X: 1, Y: 2, Z: 3}\n"
                        "Color: 10; 20; 30"),
            out.toStdString().c_str());
}

TEST(DisplayGroup, save_properties)
{
  DisplayGroup g;
  g.setName("Charles");

  MockDisplay* d = new MockDisplay;
  d->setName("Steven");
  d->subProp("Count")->setValue(101);
  g.addChild(d);

  d = new MockDisplay;
  d->setName("Katherine");
  d->subProp("Pi")->setValue(1.1);
  g.addChild(d);

  rviz::YamlConfigWriter writer;
  rviz::Config config;
  g.save(config);
  QString out = writer.writeString(config);

  // Since we instantiated the display directly instead of using the
  // DisplayFactory, it won't know its class name.
  EXPECT_EQ(std::string("Class: \"\"\n"
                        "Name: Charles\n"
                        "Enabled: false\n"
                        "Displays:\n"
                        "  - Class: \"\"\n"
                        "    Name: Steven\n"
                        "    Enabled: false\n"
                        "    Count: 101\n"
                        "    Style: chunky\n"
                        "    Pi: 3.14159\n"
                        "    Offset: {X: 1, Y: 2, Z: 3}\n"
                        "    Color: 10; 20; 30\n"
                        "  - Class: \"\"\n"
                        "    Name: Katherine\n"
                        "    Enabled: false\n"
                        "    Count: 10\n"
                        "    Style: chunky\n"
                        "    Pi: 1.1\n"
                        "    Offset: {X: 1, Y: 2, Z: 3}\n"
                        "    Color: 10; 20; 30"),
            out.toStdString().c_str());
}

TEST(DisplayFactory, class_name)
{
  std::stringstream input("Displays:\n"
                          " -\n"
                          "   Class: MockDisplay\n");

  rviz::YamlConfigReader reader;
  rviz::Config config;
  reader.readStream(config, input);

  DisplayGroup g;
  g.load(config);

  EXPECT_EQ(1, g.numChildren());
  EXPECT_EQ("MockDisplay", g.getDisplayAt(0)->getClassId().toStdString());
}

TEST(DisplayFactory, failed_display)
{
  std::stringstream input("Displays:\n"
                          "  - Class: MissingDisplay\n"
                          "    Name: Chubbers\n"
                          "    NumLemurs: 77\n"
                          "    LemurStyle: chunky\n"
                          "    SubYaml:\n"
                          "      - 1\n"
                          "      - foo: bar\n"
                          "        food: bard\n");

  rviz::YamlConfigReader reader;
  rviz::Config config;
  reader.readStream(config, input);

  DisplayGroup g;
  g.load(config);

  EXPECT_EQ(1, g.numChildren());
  EXPECT_EQ("MissingDisplay", g.getDisplayAt(0)->getClassId().toStdString());
  EXPECT_EQ(0, g.getDisplayAt(0)->numChildren()); // FailedDisplay does not have any children.

  // When a FailedDisplay is saved, it should write out its contents
  // that it was loaded with, so data is not lost.
  rviz::YamlConfigWriter writer;
  rviz::Config config2;
  g.save(config2);
  QString out = writer.writeString(config);
  EXPECT_EQ(std::string("Class: \"\"\n"
                        "Name: \"\"\n"
                        "Enabled: false\n"
                        "Displays:\n"
                        "  - Class: MissingDisplay\n"
                        "    LemurStyle: chunky\n"
                        "    Name: Chubbers\n"
                        "    NumLemurs: 77\n"
                        "    SubYaml:\n"
                        "      - 1\n"
                        "      - foo: bar\n"
                        "        food: bard"),
            out.toStdString().c_str());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "display_test", ros::init_options::AnonymousName);
  QApplication app(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
