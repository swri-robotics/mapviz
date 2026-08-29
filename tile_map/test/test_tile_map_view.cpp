// *****************************************************************************
//
// Copyright (c) 2026, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include <QCoreApplication>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <swri_transform_util/local_xy_util.h>
#include <swri_transform_util/transform.h>
#include <swri_transform_util/wgs84_transformer.h>

#include <tile_map/tile_map_view.hpp>

namespace
{
  /// The tile map plugin always looks up the transform from the local XY origin
  /// frame to the target frame. When the target frame *is* the local XY frame
  /// -- the usual case, and what the mapviz "map" fixed frame gives you -- that
  /// lookup returns the identity.
  geometry_msgs::msg::TransformStamped IdentityTf()
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = "map";
    tf.child_frame_id = "map";
    tf.transform.rotation.w = 1.0;
    return tf;
  }

  using LocalXyPtr = std::shared_ptr<swri_transform_util::LocalXyWgs84Util>;

  swri_transform_util::Transform Wgs84ToLocalXy(const LocalXyPtr& local_xy)
  {
    return swri_transform_util::Transform(
      std::make_shared<swri_transform_util::Wgs84ToTfTransform>(IdentityTf(), local_xy));
  }

  /// A point roughly 250 m north east of the given origin, expressed the way
  /// TileMapView::InitializeTile() expresses tile corners: x is longitude in
  /// degrees, y is latitude in degrees.
  tf2::Vector3 TileCornerNearOrigin(double latitude, double longitude)
  {
    return tf2::Vector3(longitude + 0.002246, latitude + 0.002246, 0.0);
  }
}

namespace tile_map
{
  /// TileMapView grants this fixture friendship. Its transform belongs to the
  /// render thread and has no accessor, so reaching it from a test means going
  /// through here; gtest bodies derive from the fixture and can call these
  /// helpers, but get no access to TileMapView themselves.
  class TileMapViewTest : public testing::Test
  {
  protected:
    static tf2::Vector3 ApplyAppliedTransform(const TileMapView& view, const tf2::Vector3& point)
    {
      return view.transform_ * point;
    }

    /// Assert that the view is projecting tile corners the way the transform it
    /// was handed says it should, rather than leaving them in degrees.
    static void ExpectProjectsLikeLocalXy(
      const TileMapView& view,
      const LocalXyPtr& local_xy,
      const tf2::Vector3& corner)
    {
      double expected_x, expected_y;
      ASSERT_TRUE(local_xy->ToLocalXy(corner.y(), corner.x(), expected_x, expected_y));

      const tf2::Vector3 transformed = ApplyAppliedTransform(view, corner);

      EXPECT_NEAR(expected_x, transformed.x(), 1e-6);
      EXPECT_NEAR(expected_y, transformed.y(), 1e-6);
    }
  };

  /// Regression test for https://github.com/swri-robotics/mapviz/issues/909.
  ///
  /// A WGS84 transform whose local origin is at lat/lon (0, 0) -- what a Gazebo
  /// simulation typically publishes -- has the same origin and orientation as
  /// the identity transform, because transforming (0, 0, 0) lands on the origin
  /// itself. SetTransform() used to compare only those two properties and so
  /// discarded the transform, leaving the tiles in degrees. A tileset covering
  /// 500 m was drawn 4.5 mm across.
  TEST_F(TileMapViewTest, AppliesWgs84TransformWithOriginAtLatLonZero)
  {
    TileMapView view;

    auto local_xy = std::make_shared<swri_transform_util::LocalXyWgs84Util>(0.0, 0.0);
    view.SetTransform(Wgs84ToLocalXy(local_xy));

    const tf2::Vector3 corner = TileCornerNearOrigin(0.0, 0.0);
    ExpectProjectsLikeLocalXy(view, local_xy, corner);

    // Guard against the degenerate failure explicitly: the corner has to come
    // out in meters, not left behind as 0.002246 degrees.
    const tf2::Vector3 transformed = ApplyAppliedTransform(view, corner);
    EXPECT_GT(std::abs(transformed.x()), 200.0);
    EXPECT_GT(std::abs(transformed.y()), 200.0);
  }

  /// The case that always worked: with a real-world origin, transforming
  /// (0, 0, 0) lands millions of meters away from the local origin, so the old
  /// comparison happened to notice the difference.
  TEST_F(TileMapViewTest, AppliesWgs84TransformWithNonZeroOrigin)
  {
    TileMapView view;

    auto local_xy = std::make_shared<swri_transform_util::LocalXyWgs84Util>(29.45, -98.6);
    view.SetTransform(Wgs84ToLocalXy(local_xy));

    ExpectProjectsLikeLocalXy(view, local_xy, TileCornerNearOrigin(29.45, -98.6));
  }

  /// Setting an equivalent transform a second time is a no-op, but it must not
  /// leave a stale or partially applied transform behind. SetTransform() is
  /// called once per frame, so this is the common path.
  TEST_F(TileMapViewTest, KeepsTransformWhenSetAgainWithAnEquivalentTransform)
  {
    TileMapView view;

    auto local_xy = std::make_shared<swri_transform_util::LocalXyWgs84Util>(0.0, 0.0);
    view.SetTransform(Wgs84ToLocalXy(local_xy));
    view.SetTransform(Wgs84ToLocalXy(local_xy));

    ExpectProjectsLikeLocalXy(view, local_xy, TileCornerNearOrigin(0.0, 0.0));
  }

  /// The skip must not become "only ever apply the first transform": when the
  /// local XY origin moves, the tiles have to be re-projected around the new one.
  TEST_F(TileMapViewTest, AppliesUpdatedTransformWhenLocalOriginChanges)
  {
    TileMapView view;

    auto null_island = std::make_shared<swri_transform_util::LocalXyWgs84Util>(0.0, 0.0);
    view.SetTransform(Wgs84ToLocalXy(null_island));

    auto san_antonio = std::make_shared<swri_transform_util::LocalXyWgs84Util>(29.45, -98.6);
    view.SetTransform(Wgs84ToLocalXy(san_antonio));

    ExpectProjectsLikeLocalXy(view, san_antonio, TileCornerNearOrigin(29.45, -98.6));
  }
}

int main(int argc, char** argv)
{
  // TileMapView owns an ImageCache, which needs a Qt event loop to live in.
  QCoreApplication app(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
