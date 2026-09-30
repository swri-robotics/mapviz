// *****************************************************************************
//
// Copyright (c) 2026, Southwest Research Institute® (SwRI®)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include <gps_msgs/msg/gps_fix.hpp>
#include <gps_msgs/msg/gps_status.hpp>
#include <mapviz_plugins/has_position.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

namespace
{
template<typename FixT>
FixT Fix(double latitude, double longitude)
{
  FixT fix;
  fix.status.status = decltype(fix.status)::STATUS_FIX;
  fix.latitude = latitude;
  fix.longitude = longitude;
  fix.altitude = 250.0;
  return fix;
}

template<typename FixT>
class HasPositionTest : public ::testing::Test {};

using FixTypes = ::testing::Types<gps_msgs::msg::GPSFix, sensor_msgs::msg::NavSatFix>;
TYPED_TEST_SUITE(HasPositionTest, FixTypes);
}  // namespace

TYPED_TEST(HasPositionTest, AcceptsAFixWithAPosition)
{
  EXPECT_TRUE(mapviz_plugins::HasPosition(Fix<TypeParam>(29.45, -98.61)));
}

TYPED_TEST(HasPositionTest, AcceptsAFixWithoutAnAltitude)
{
  TypeParam fix = Fix<TypeParam>(29.45, -98.61);
  fix.altitude = std::nan("");
  EXPECT_TRUE(mapviz_plugins::HasPosition(fix));
}

TYPED_TEST(HasPositionTest, RejectsNoFix)
{
  TypeParam fix = Fix<TypeParam>(29.45, -98.61);
  fix.status.status = decltype(fix.status)::STATUS_NO_FIX;
  EXPECT_FALSE(mapviz_plugins::HasPosition(fix));
}

TYPED_TEST(HasPositionTest, RejectsANonFinitePosition)
{
  const double nan = std::nan("");
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(mapviz_plugins::HasPosition(Fix<TypeParam>(nan, -98.61)));
  EXPECT_FALSE(mapviz_plugins::HasPosition(Fix<TypeParam>(29.45, nan)));
  EXPECT_FALSE(mapviz_plugins::HasPosition(Fix<TypeParam>(inf, -98.61)));
  EXPECT_FALSE(mapviz_plugins::HasPosition(Fix<TypeParam>(29.45, -inf)));
}
