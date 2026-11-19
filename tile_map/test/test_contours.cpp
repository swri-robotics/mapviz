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
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include <tile_map/contour_layer.hpp>
#include <tile_map/contours.hpp>
#include <tile_map/terrain.hpp>

using tile_map::AutoContourInterval;
using tile_map::ContourLayer;
using tile_map::ContourLine;
using tile_map::GridPoint;
using tile_map::LabelSpot;
using tile_map::LabelSpots;
using tile_map::TraceContours;

namespace
{
constexpr size_t NO_LIMIT = 10000000;
const double PI = std::acos(-1.0);

/// A grid rising `rise` meters per column going east, from `base` at column 0.
std::vector<float> RisingEast(int width, int height, float base, float rise)
{
  std::vector<float> heights;
  for (int row = 0; row < height; ++row) {
    for (int col = 0; col < width; ++col) {
      heights.push_back(base + rise * col);
    }
  }
  return heights;
}

/// A cone: `top` meters at (cx, cy), falling `fall` meters per cell away.
std::vector<float> Cone(int width, int height, double cx, double cy, float top, float fall)
{
  std::vector<float> heights;
  for (int row = 0; row < height; ++row) {
    for (int col = 0; col < width; ++col) {
      heights.push_back(top - fall * static_cast<float>(std::hypot(col - cx, row - cy)));
    }
  }
  return heights;
}

const ContourLine * LineAt(const std::vector<ContourLine> & lines, double height)
{
  for (const ContourLine & line : lines) {
    if (line.height == height) {
      return &line;
    }
  }
  return nullptr;
}
}  // namespace

TEST(AutoContourInterval, RoundsUpToOneTwoOrFive)
{
  EXPECT_DOUBLE_EQ(1.0, AutoContourInterval(0.1));
  EXPECT_DOUBLE_EQ(2.0, AutoContourInterval(1.0));
  EXPECT_DOUBLE_EQ(5.0, AutoContourInterval(2.0));
  EXPECT_DOUBLE_EQ(10.0, AutoContourInterval(4.2));
  EXPECT_DOUBLE_EQ(20.0, AutoContourInterval(10.0));
  EXPECT_DOUBLE_EQ(200.0, AutoContourInterval(100.0));
  EXPECT_DOUBLE_EQ(500.0, AutoContourInterval(300.0));
}

TEST(AutoContourInterval, BadScaleGivesOneMeter)
{
  EXPECT_DOUBLE_EQ(1.0, AutoContourInterval(0.0));
  EXPECT_DOUBLE_EQ(1.0, AutoContourInterval(-3.0));
  EXPECT_DOUBLE_EQ(1.0, AutoContourInterval(std::numeric_limits<double>::quiet_NaN()));
}

TEST(TraceContours, SlopeGivesOneStraightLinePerInterval)
{
  // 100 m at column 0, rising 1 m per column to 119 m: lines at 100 (column
  // 0, ground at the height counts as above it), 105, 110 and 115.
  const int width = 20;
  const int height = 10;
  std::vector<ContourLine> lines;
  ASSERT_TRUE(
    TraceContours(
      RisingEast(width, height, 100.0f, 1.0f), width, height, 5.0,
      NO_LIMIT, lines));
  ASSERT_EQ(3u, lines.size());
  for (double h : {105.0, 110.0, 115.0}) {
    const ContourLine * line = LineAt(lines, h);
    ASSERT_NE(nullptr, line) << h;
    EXPECT_FALSE(line->closed);
    // One point on each row's horizontal edge, all at the same column.
    ASSERT_EQ(static_cast<size_t>(height), line->points.size());
    for (const GridPoint & point : line->points) {
      EXPECT_NEAR(h - 100.0, point.x, 1e-9);
    }
    EXPECT_NEAR(
      static_cast<double>(height - 1),
      std::abs(line->points.back().y - line->points.front().y), 1e-9);
  }
}

TEST(TraceContours, MarksEveryFifthLineAsIndex)
{
  const int width = 60;
  const int height = 4;
  std::vector<ContourLine> lines;
  ASSERT_TRUE(
    TraceContours(
      RisingEast(width, height, 0.5f, 1.0f), width, height, 2.0,
      NO_LIMIT, lines));
  for (const ContourLine & line : lines) {
    const auto k = std::lround(line.height / 2.0);
    EXPECT_EQ(k % 5 == 0, line.index) << line.height;
  }
  EXPECT_NE(nullptr, LineAt(lines, 10.0));
  EXPECT_TRUE(LineAt(lines, 10.0)->index);
  EXPECT_FALSE(LineAt(lines, 12.0)->index);
}

TEST(TraceContours, HillGivesClosedRings)
{
  const int size = 41;
  std::vector<ContourLine> lines;
  ASSERT_TRUE(
    TraceContours(
      Cone(size, size, 20.0, 20.0, 100.0f, 1.0f), size, size, 5.0,
      NO_LIMIT, lines));
  // The rings at 90 and 85 m (10 and 15 cells out) are inside the grid.
  for (double h : {90.0, 85.0}) {
    const ContourLine * line = LineAt(lines, h);
    ASSERT_NE(nullptr, line) << h;
    EXPECT_TRUE(line->closed) << h;
    const double radius = 100.0 - h;
    for (const GridPoint & point : line->points) {
      EXPECT_NEAR(radius, std::hypot(point.x - 20.0, point.y - 20.0), 0.2) << h;
    }
  }
}

TEST(TraceContours, MissingDataBreaksLines)
{
  const int width = 20;
  const int height = 10;
  std::vector<float> heights = RisingEast(width, height, 100.0f, 1.0f);
  // A gap across the middle row.
  for (int col = 0; col < width; ++col) {
    heights[5 * width + col] = std::numeric_limits<float>::quiet_NaN();
  }
  std::vector<ContourLine> lines;
  ASSERT_TRUE(TraceContours(heights, width, height, 5.0, NO_LIMIT, lines));
  // Each height is now two lines, above and below the gap.
  size_t at_110 = 0;
  for (const ContourLine & line : lines) {
    if (line.height == 110.0) {
      ++at_110;
      for (const GridPoint & point : line.points) {
        EXPECT_TRUE(point.y <= 4.0 || point.y >= 6.0);
      }
    }
  }
  EXPECT_EQ(2u, at_110);
}

TEST(TraceContours, SaddleGivesTwoSeparateLines)
{
  // High corners at northwest and southeast, low ones at the others.
  const std::vector<float> heights = {10.0f, 0.0f, 0.0f, 10.0f};
  std::vector<ContourLine> lines;
  // Only a line at 6 m, below the 5 m average of the corners.
  ASSERT_TRUE(TraceContours(heights, 2, 2, 6.0, NO_LIMIT, lines));
  ASSERT_EQ(2u, lines.size());
  for (const ContourLine & line : lines) {
    EXPECT_EQ(2u, line.points.size());
    EXPECT_FALSE(line.closed);
  }
}

TEST(TraceContours, RefusesTooManySegments)
{
  const int width = 50;
  const int height = 50;
  std::vector<ContourLine> lines;
  EXPECT_FALSE(
    TraceContours(
      RisingEast(width, height, 0.0f, 10.0f), width, height, 1.0,
      1000, lines));
  EXPECT_TRUE(lines.empty());
}

TEST(TraceContours, RejectsBadInput)
{
  std::vector<ContourLine> lines;
  EXPECT_FALSE(TraceContours({1.0f, 2.0f, 3.0f}, 2, 2, 1.0, NO_LIMIT, lines));
  EXPECT_FALSE(TraceContours({1.0f, 2.0f, 3.0f, 4.0f}, 2, 2, 0.0, NO_LIMIT, lines));
  EXPECT_FALSE(TraceContours({1.0f, 2.0f}, 2, 1, 1.0, NO_LIMIT, lines));
}

TEST(TraceContours, FlatGroundAtALineHeightHasNoLine)
{
  const std::vector<float> heights(25, 20.0f);
  std::vector<ContourLine> lines;
  ASSERT_TRUE(TraceContours(heights, 5, 5, 10.0, NO_LIMIT, lines));
  EXPECT_TRUE(lines.empty());
}

TEST(LabelSpots, SpacedAlongStraightIndexLines)
{
  ContourLine line{50.0, true, false, {}};
  for (int i = 0; i <= 1000; ++i) {
    line.points.push_back({5.0, static_cast<double>(i)});
  }
  ContourLine ordinary = line;
  ordinary.index = false;
  ordinary.height = 40.0;

  const std::vector<LabelSpot> spots = LabelSpots({line, ordinary}, 200.0, 20.0);
  // At 100, 300, 500, 700 and 900 along the line.
  ASSERT_EQ(5u, spots.size());
  for (size_t i = 0; i < spots.size(); ++i) {
    EXPECT_DOUBLE_EQ(50.0, spots[i].height);
    EXPECT_NEAR(100.0 + 200.0 * i, spots[i].position.y, 1e-9);
    EXPECT_NEAR(40.0, std::abs(spots[i].after.y - spots[i].before.y), 1e-9);
  }
}

TEST(LabelSpots, NoneOnShortOrTightlyCurvedLines)
{
  ContourLine short_line{50.0, true, false, {{0.0, 0.0}, {0.0, 30.0}}};
  // A ring 8 cells across: far too curved for a label 40 cells long.
  ContourLine ring{100.0, true, true, {}};
  for (int i = 0; i < 64; ++i) {
    const double angle = 2.0 * PI * i / 64.0;
    ring.points.push_back({4.0 * std::cos(angle), 4.0 * std::sin(angle)});
  }
  EXPECT_TRUE(LabelSpots({short_line, ring}, 200.0, 20.0).empty());
}

TEST(LabelSpots, OneOnAShortClosedLoop)
{
  // A ring 200 cells around: shorter than the spacing, long enough for one.
  ContourLine ring{100.0, true, true, {}};
  const double radius = 200.0 / (2.0 * PI);
  for (int i = 0; i < 400; ++i) {
    const double angle = 2.0 * PI * i / 400.0;
    ring.points.push_back({radius * std::cos(angle), radius * std::sin(angle)});
  }
  EXPECT_EQ(1u, LabelSpots({ring}, 400.0, 5.0).size());
}

TEST(ContourLayer, BlockCoversTheView)
{
  // 2 m per pixel at 29.45 N: level 15 pixels are about 4.2 m, the deepest
  // level the data set has.
  const ContourLayer::Block block =
    ContourLayer::BlockFor(29.452743, -98.609970, 2.0, 1600, 1000);
  EXPECT_EQ(tile_map::terrarium::MAX_ZOOM, block.level);
  EXPECT_GT(block.columns, 0);
  EXPECT_GT(block.rows, 0);
  EXPECT_LE(static_cast<size_t>(block.columns * block.rows), ContourLayer::MAX_TILES);

  // The view's center is inside the block.
  double north;
  double west;
  double south;
  double east;
  ContourLayer::GridToLatLon(block, -0.5, -0.5, north, west);
  ContourLayer::GridToLatLon(
    block, block.columns * 256.0 - 0.5, block.rows * 256.0 - 0.5, south, east);
  EXPECT_GT(north, 29.452743);
  EXPECT_LT(south, 29.452743);
  EXPECT_LT(west, -98.609970);
  EXPECT_GT(east, -98.609970);
}

TEST(ContourLayer, BlockPixelsNoSmallerThanTheScreens)
{
  for (double scale : {10.0, 50.0, 300.0}) {
    const ContourLayer::Block block =
      ContourLayer::BlockFor(29.452743, -98.609970, scale, 800, 600);
    ASSERT_GT(block.columns, 0) << scale;
    const double cell = tile_map::MetersPerPixel(block.level, 29.452743);
    EXPECT_GE(cell, scale) << scale;
    EXPECT_LT(cell, 2.0 * scale + 1e-9) << scale;
  }
}

TEST(ContourLayer, BigCanvasDropsToACoarserLevel)
{
  const ContourLayer::Block small =
    ContourLayer::BlockFor(29.452743, -98.609970, 20.0, 800, 600);
  const ContourLayer::Block big =
    ContourLayer::BlockFor(29.452743, -98.609970, 20.0, 8000, 6000);
  ASSERT_GT(big.columns, 0);
  EXPECT_LT(big.level, small.level);
  EXPECT_LE(static_cast<size_t>(big.columns * big.rows), ContourLayer::MAX_TILES);
}

TEST(ContourLayer, JoinedTilesLineUpAndLeaveGapsForMissingOnes)
{
  ContourLayer::Block block;
  block.level = 10;
  block.x = 100;
  block.y = 200;
  block.columns = 2;
  block.rows = 2;
  auto tile = [](float value) {
      return std::make_shared<const std::vector<float>>(256 * 256, value);
    };
  const std::vector<float> grid =
    ContourLayer::JoinTiles(block, {tile(1.0f), tile(2.0f), tile(3.0f), nullptr});
  ASSERT_EQ(512u * 512u, grid.size());
  EXPECT_EQ(1.0f, grid[0]);
  EXPECT_EQ(1.0f, grid[255 * 512 + 255]);
  EXPECT_EQ(2.0f, grid[256]);
  EXPECT_EQ(3.0f, grid[256 * 512]);
  EXPECT_TRUE(std::isnan(grid[256 * 512 + 256]));
  EXPECT_TRUE(std::isnan(grid[511 * 512 + 511]));
}

TEST(ContourLayer, LineRunsOnAcrossTileEdges)
{
  ContourLayer::Block block;
  block.level = 12;
  block.x = 0;
  block.y = 0;
  block.columns = 2;
  block.rows = 1;
  // One slope, rising east across both tiles.
  auto tile = [](int first_column) {
      auto heights = std::make_shared<std::vector<float>>();
      for (int row = 0; row < 256; ++row) {
        for (int col = 0; col < 256; ++col) {
          heights->push_back(static_cast<float>(first_column + col) * 0.1f);
        }
      }
      return std::shared_ptr<const std::vector<float>>(heights);
    };
  const std::vector<float> grid = ContourLayer::JoinTiles(block, {tile(0), tile(256)});
  // The 25.55 m line runs between the last column of the first tile
  // (25.5 m) and the first column of the second (25.6 m), the whole way down.
  std::vector<ContourLine> lines;
  ASSERT_TRUE(TraceContours(grid, 512, 256, 0.05, NO_LIMIT, lines));
  const ContourLine * seam = LineAt(lines, 511 * 0.05);
  ASSERT_NE(nullptr, seam);
  EXPECT_NEAR(255.5, seam->points.front().x, 1e-3);
  EXPECT_EQ(256u, seam->points.size());
}
