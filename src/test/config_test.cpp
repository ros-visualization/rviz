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

#include <gtest/gtest.h>
#include <rviz/config.h>

#include <ros/package.h> // For ros::package::getPath()

TEST( Config, comparison )
{
  std::string lhs = "b/c/d";
  std::string rhs = "b/c_d";
  rviz::Config::DirectoryCompare dc;
  EXPECT_FALSE( dc( lhs, rhs ));
}

TEST( Config, set_then_get )
{
  rviz::Config c;
  c.set( "a", "1" );
  int a;
  EXPECT_TRUE( c.get( "a", &a, 2 ));
  EXPECT_EQ( a, 1 );
  float aa;
  EXPECT_TRUE( c.get( "a", &aa, 2.0 ));
  EXPECT_EQ( aa, 1.0 );
  std::string aaa;
  EXPECT_TRUE( c.get( "a", &aaa, "two" ));
  EXPECT_EQ( aaa, "1" );
}

TEST( Config, parse_floats )
{
  rviz::Config c;
  c.set( "f", "1.1" );
  float f;
  EXPECT_TRUE( c.get( "f", &f, 2.0f ));
  EXPECT_EQ( f, 1.1f );

  // In Europe they use "," for a decimal point.
  c.set( "f", "1,2" );
  EXPECT_TRUE( c.get( "f", &f, 2.0f ));
  EXPECT_EQ( f, 1.2f );

  // This test should pass in all locales because Config uses
  // UniformStringStream which forces output in the "C" locale.
  std::string str;
  c.set( "f", 1.2f );
  c.get( "f", &str, "chub" );
  EXPECT_EQ( str, "1.2" );
}

TEST( Config, default_values )
{
  rviz::Config c;
  int a;
  EXPECT_FALSE( c.get( "a", &a, 2 ));
  EXPECT_EQ( a, 2 );
  EXPECT_FALSE( c.get( "a", &a ));
  EXPECT_EQ( a, 0 );
  float aa;
  EXPECT_FALSE( c.get( "a", &aa, 2.0 ));
  EXPECT_EQ( aa, 2.0 );
  EXPECT_FALSE( c.get( "a", &aa ));
  EXPECT_EQ( aa, 0.0 );
  std::string aaa;
  EXPECT_FALSE( c.get( "a", &aaa, "two" ));
  EXPECT_EQ( aaa, "two" );
  EXPECT_FALSE( c.get( "a", &aaa ));
  EXPECT_EQ( aaa, "" );
}

TEST( Config, write_empty_string_value )
{
  rviz::Config c;
  c.set( "key", "" );

  std::stringstream ss;
  c.write( ss );

  EXPECT_EQ( "key=\n", ss.str() );
}

TEST( Config, read_empty_string_value )
{
  std::istringstream ss( "key=\n" );

  rviz::Config c;
  c.read( ss );

  std::string s;
  EXPECT_TRUE( c.get( "key", &s, "default" ));
  EXPECT_EQ( "", s );
}

TEST( Config, set_get_empty_value )
{
  rviz::Config c;
  c.set( "key", "" );

  std::string s;
  EXPECT_TRUE( c.get( "key", &s, "default" ));
  EXPECT_EQ( "", s );
}

TEST( Config, write )
{
  rviz::Config c;
  c.set( "/b/c_d", 1 );
  c.set( "b/alpha", 2 );
  c.set( "z", 6 );
  c.set( "/a/z", 3 );
  c.set( "d", 7 );
  c.set( "/b/c/d", 4);
  c.set( "/a", 5 );
  c.set( "/art/for/arts/sake", 8 );
  std::stringstream ss;
  c.write( ss );
  EXPECT_EQ( 
"a=5\n\
d=7\n\
z=6\n\
[a]\n\
z=3\n\
[art]\n\
[art/for]\n\
[art/for/arts]\n\
sake=8\n\
[b]\n\
alpha=2\n\
c_d=1\n\
[b/c]\n\
d=4\n\
", ss.str() );
}

TEST( Config, read )
{
  std::string input =
"a=1\n\
foo_bar=biff=1; bazz=17\n\
[chub]\n\
sand=wich\n\
[chub/town]\n\
belly=big\n\
tummy=large\n\
";
  std::istringstream iss;
  iss.str( input );
  rviz::Config c;
  c.read( iss );

  std::string s;
  EXPECT_TRUE( c.get( "/chub/town/belly", &s ));
  EXPECT_EQ( "big", s );

  EXPECT_TRUE( c.get( "/chub/sand", &s ));
  EXPECT_EQ( "wich", s );

  EXPECT_TRUE( c.get( "/foo_bar", &s ));
  EXPECT_EQ( "biff=1; bazz=17", s );

  int i;
  EXPECT_TRUE( c.get( "/a", &i ));
  EXPECT_EQ( 1, i );
}

TEST( Config, escape_space )
{
  rviz::Config c;
  c.set( "a b", "c d" );
  std::string s;
  EXPECT_TRUE( c.get( "a b", &s ));
  EXPECT_EQ( "c d", s );

  std::stringstream ss;
  c.write( ss );
  EXPECT_EQ( "a\\ b=c d\n", ss.str() );
}

TEST( Config, escape_colon )
{
  rviz::Config c;
  c.set( "a:b", "c:d" );
  std::string s;
  EXPECT_TRUE( c.get( "a:b", &s ));
  EXPECT_EQ( "c:d", s );

  std::stringstream ss;
  c.write( ss );
  EXPECT_EQ( "a\\:b=c:d\n", ss.str() );
}

TEST( Config, first_slash_is_optional )
{
  rviz::Config c;
  c.set( "a", 1 );
  int i;
  EXPECT_TRUE( c.get( "/a", &i ));
  EXPECT_EQ( 1, i );

  c.set( "/b", 2 );
  EXPECT_TRUE( c.get( "b", &i ));
  EXPECT_EQ( 2, i );
}

TEST( Config, end_to_end )
{
  rviz::Config c;
  c.set( "/a: b/foo/bar", 1.25f );
  c.set( "/a: b/foo/biff", 17 );
  c.set( "a", "a/b=c d" );

  std::stringstream ss;
  c.write( ss );

  c.clear();

  c.read( ss );
  float f;
  EXPECT_TRUE( c.get( "/a: b/foo/bar", &f ));
  EXPECT_EQ( 1.25, f ); // I don't need approximately-equal because I chose a floating-point-friendly number.
  int i;
  EXPECT_TRUE( c.get( "/a: b/foo/biff", &i ));
  EXPECT_EQ( 17, i );
  std::string s;
  EXPECT_TRUE( c.get( "a", &s ));
  EXPECT_EQ( "a/b=c d", s );
}

TEST( Config, file_io )
{
  std::string package_path = ros::package::getPath(ROS_PACKAGE_NAME);

  rviz::Config c;
  EXPECT_TRUE( c.readFromFile( package_path + "/src/test/test_in.vcg" ));
  EXPECT_TRUE( c.writeToFile( package_path + "/build/test_out.vcg" ));

  // This test would be better if it sorted the input and output files
  // and checked that the results were the same.  I don't care enough
  // to learn how to do that with gtest right now.
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

