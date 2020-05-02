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

#include <locale>

#include <gtest/gtest.h>
#include <rviz/uniform_string_stream.h>

using namespace rviz;

TEST(UniformStringStream, parse_floats)
{
  UniformStringStream uss("1,2 3.4 5,6e2");
  float a, b, c;
  uss.parseFloat(a);
  uss.parseFloat(b);
  uss.parseFloat(c);
  EXPECT_TRUE(!!uss);
  EXPECT_EQ(a, 1.2f);
  EXPECT_EQ(b, 3.4f);
  EXPECT_EQ(c, 560.0f);
  uss.parseFloat(a);
  EXPECT_FALSE(!!uss);
}

TEST(UniformStringStream, parse_ints)
{
  UniformStringStream uss("1 2 -3");
  int a, b, c;
  uss >> a;
  uss >> b;
  uss >> c;
  EXPECT_TRUE(!!uss);
  EXPECT_EQ(a, 1);
  EXPECT_EQ(b, 2);
  EXPECT_EQ(c, -3);
  uss >> a;
  EXPECT_FALSE(!!uss);
}

class CommaFloat : public std::numpunct<char>
{
protected:
  char do_decimal_point() const override
  {
    return 'p';
  }
};

TEST(UniformStringStream, print_floats)
{
  UniformStringStream uss;
  uss << 1.2f;
  EXPECT_EQ(uss.str(), "1.2");

  CommaFloat* comma_float_facet = new CommaFloat;
  std::locale new_locale(std::locale::classic(), comma_float_facet);

  std::locale old_locale = std::locale::global(new_locale);

  // Make sure the comma_float_facet is working.
  std::stringstream ss;
  ss << 3.4f;
  EXPECT_EQ(ss.str(), "3p4");

  // Make sure the float facet gets clobbered within UniformStringStream.
  UniformStringStream uss2;
  uss2 << 3.4f;
  EXPECT_EQ(uss2.str(), "3.4");

  // Put things back to normal so other tests don't break.
  std::locale::global(old_locale);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
