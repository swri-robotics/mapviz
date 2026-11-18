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
#include <QImage>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include <tf2/LinearMath/Vector3.hpp>
#include <tile_map/buildings.hpp>
#include <tile_map/scene_25d.hpp>
#include <tile_map/terrain.hpp>
#include <tile_map/tree_layer.hpp>
#include <tile_map/trees.hpp>

using tile_map::Crown;
using tile_map::PixelCrown;

namespace
{
const QRgb CANOPY = qRgb(60, 90, 50);
const QRgb GRASS = qRgb(150, 170, 120);
const QRgb ROOF = qRgb(128, 128, 128);

/// A disc of canopy pixels in an otherwise empty mask.
std::vector<uint8_t> DiscMask(int size, double cx, double cy, double radius)
{
  std::vector<uint8_t> mask(static_cast<size_t>(size) * size, 0);
  for (int row = 0; row < size; ++row) {
    for (int col = 0; col < size; ++col) {
      if (std::hypot(col + 0.5 - cx, row + 0.5 - cy) <= radius) {
        mask[static_cast<size_t>(row) * size + col] = 1;
      }
    }
  }
  return mask;
}

int Count(const std::vector<uint8_t> & mask, int size, int x0, int y0, int x1, int y1)
{
  int count = 0;
  for (int row = y0; row < y1; ++row) {
    for (int col = x0; col < x1; ++col) {
      count += mask[static_cast<size_t>(row) * size + col];
    }
  }
  return count;
}

tile_map::ProjectedBuilding Square(double x, double y, double size, double height)
{
  tile_map::ProjectedBuilding building;
  building.outline = {
    {x, y, 0.0}, {x + size, y, 0.0}, {x + size, y + size, 0.0}, {x, y + size, 0.0}};
  building.walls.assign(4, true);
  building.triangles = {0, 1, 2, 0, 2, 3};
  building.height = height;
  building.counterclockwise = true;
  return building;
}

/// A level 10 tile is 1/1024 of the mercator map; mercator y grows southward,
/// so a square's corners there run the other way round from the drawn ones.
constexpr int32_t LEVEL = 10;
constexpr double TILE = 1.0 / 1024.0;

/// A square roof from (x, y) to (x + size, y + size) in tiles at LEVEL,
/// drawn one unit per tile.
tile_map::ProjectedBuilding MappedSquare(double x, double y, double size)
{
  tile_map::ProjectedBuilding building = Square(x, -y - size, size, 10.0);
  // Drawn y is up; mercator y is down.
  building.mercator = {
    x * TILE, (y + size) * TILE, (x + size) * TILE, (y + size) * TILE,
    (x + size) * TILE, y * TILE, x * TILE, y * TILE};
  return building;
}

/// Area covered by the triangles of the batch's runs with `texture`.
double TexturedArea(const tile_map::Batch25d & batch, uint32_t texture)
{
  double area = 0.0;
  const std::vector<double> & v = batch.Vertices();
  for (const auto & run : batch.Runs()) {
    if (run.texture != texture) {
      continue;
    }
    for (size_t i = run.first; i + 3 <= run.first + run.count; i += 3) {
      const double ax = v[2 * i];
      const double ay = v[2 * i + 1];
      const double bx = v[2 * i + 2];
      const double by = v[2 * i + 3];
      const double cx = v[2 * i + 4];
      const double cy = v[2 * i + 5];
      area += std::abs((bx - ax) * (cy - ay) - (by - ay) * (cx - ax)) / 2.0;
    }
  }
  return area;
}
}  // namespace

TEST(Canopy, FindsDarkGreenCanopyOnly)
{
  QImage image(64, 64, QImage::Format_RGB32);
  image.fill(GRASS);
  for (int y = 10; y < 30; ++y) {
    for (int x = 10; x < 30; ++x) {
      image.setPixel(x, y, CANOPY);
    }
  }
  for (int y = 40; y < 60; ++y) {
    for (int x = 40; x < 60; ++x) {
      image.setPixel(x, y, ROOF);
    }
  }
  // A lone dark pixel is a speck, not a tree.
  image.setPixel(50, 5, CANOPY);

  const std::vector<uint8_t> mask = tile_map::CanopyMask(image);
  ASSERT_EQ(64u * 64u, mask.size());
  EXPECT_EQ(400, Count(mask, 64, 10, 10, 30, 30));
  EXPECT_EQ(400, Count(mask, 64, 0, 0, 64, 64));
}

TEST(Canopy, MapStylesHaveNoCanopy)
{
  // Light, map-style woods and parks (OpenStreetMap Carto's forest, Carto's
  // park green) are too bright to be canopy.
  for (QRgb color : {qRgb(173, 209, 158), qRgb(216, 232, 200), qRgb(200, 250, 204)}) {
    QImage image(16, 16, QImage::Format_RGB32);
    image.fill(color);
    const std::vector<uint8_t> mask = tile_map::CanopyMask(image);
    EXPECT_EQ(0, Count(mask, 16, 0, 0, 16, 16));
  }
}

TEST(Crowns, FillsOneTreeWithOneCrown)
{
  const std::vector<uint8_t> mask = DiscMask(40, 20.0, 20.0, 4.0);
  const std::vector<PixelCrown> crowns = tile_map::PlaceCrowns(mask, 40, 40, 1.0, 1);
  ASSERT_EQ(1u, crowns.size());
  EXPECT_NEAR(20.0, crowns[0].x, 1.0);
  EXPECT_NEAR(20.0, crowns[0].y, 1.0);
  EXPECT_NEAR(4.0, crowns[0].radius, 1.0);
  EXPECT_GE(crowns[0].height, 3.0);
}

TEST(Crowns, SplitsAForestIntoTreesThatBarelyOverlap)
{
  const std::vector<uint8_t> forest(100 * 100, 1);
  const std::vector<PixelCrown> crowns = tile_map::PlaceCrowns(forest, 100, 100, 1.0, 7);
  ASSERT_GT(crowns.size(), 50u);
  for (size_t i = 0; i < crowns.size(); ++i) {
    EXPECT_LE(crowns[i].radius, 6.0);
    EXPECT_GE(crowns[i].height, 3.0);
    EXPECT_LE(crowns[i].height, 25.0);
    for (size_t j = i + 1; j < crowns.size(); ++j) {
      const double gap = std::hypot(crowns[i].x - crowns[j].x, crowns[i].y - crowns[j].y);
      EXPECT_GE(gap + 1e-9, 0.75 * (crowns[i].radius + crowns[j].radius));
    }
  }

  // The same canopy always gives the same trees.
  const std::vector<PixelCrown> again = tile_map::PlaceCrowns(forest, 100, 100, 1.0, 7);
  ASSERT_EQ(crowns.size(), again.size());
  EXPECT_EQ(crowns.front().height, again.front().height);
}

TEST(Crowns, ScalesWithTheGroundSizeOfAPixel)
{
  // The same 4 pixel disc is a 4 m tree at 1 m per pixel and an 8 m one at 2.
  const std::vector<uint8_t> mask = DiscMask(40, 20.0, 20.0, 4.0);
  const std::vector<PixelCrown> fine = tile_map::PlaceCrowns(mask, 40, 40, 1.0, 1);
  const std::vector<PixelCrown> coarse = tile_map::PlaceCrowns(mask, 40, 40, 2.0, 1);
  ASSERT_FALSE(fine.empty());
  ASSERT_FALSE(coarse.empty());
  EXPECT_GT(coarse.front().radius, fine.front().radius);
}

TEST(Crowns, RejectsUnusableInput)
{
  EXPECT_TRUE(tile_map::PlaceCrowns(std::vector<uint8_t>(100, 0), 10, 10, 1.0, 1).empty());
  EXPECT_TRUE(tile_map::PlaceCrowns(std::vector<uint8_t>(99, 1), 10, 10, 1.0, 1).empty());
  EXPECT_TRUE(tile_map::PlaceCrowns(std::vector<uint8_t>(100, 1), 10, 10, 0.0, 1).empty());
}

TEST(Woodland, SpreadsTreesThroughTheWood)
{
  // About 100 m square.
  const double south = 29.45;
  const double west = -98.62;
  const double dlat = 100.0 / 111320.0;
  const double dlon = 100.0 / (111320.0 * std::cos(29.45 * 3.14159265358979 / 180.0));
  const std::vector<tile_map::LatLon> square = {
    {south, west}, {south, west + dlon}, {south + dlat, west + dlon}, {south + dlat, west}};

  const std::vector<Crown> crowns =
    tile_map::CrownsInWoodland(square, south - 1.0, west - 1.0, south + 1.0, west + 1.0, 3);
  // One tree per 6 m grid cell.
  EXPECT_NEAR((100.0 / 6.0) * (100.0 / 6.0), static_cast<double>(crowns.size()), 40.0);
  for (const Crown & crown : crowns) {
    EXPECT_GE(crown.center.latitude, south);
    EXPECT_LE(crown.center.latitude, south + dlat);
    EXPECT_GE(crown.center.longitude, west);
    EXPECT_LE(crown.center.longitude, west + dlon);
  }

  // Only the part of the wood inside the box gets trees.
  const std::vector<Crown> half = tile_map::CrownsInWoodland(
    square, south - 1.0, west - 1.0, south + dlat / 2.0, west + 1.0, 3);
  EXPECT_NEAR(crowns.size() / 2.0, static_cast<double>(half.size()), 20.0);
  EXPECT_TRUE(
    tile_map::CrownsInWoodland(square, south + 1.0, west, south + 2.0, west + 1.0, 3).empty());
}

TEST(Woodland, ReadsMappedTrees)
{
  const tile_map::MapTree tagged = tile_map::TreeFromTags(
    {29.45, -98.6}, {{"natural", "tree"}, {"height", "12"}, {"diameter_crown", "8 m"}});
  EXPECT_DOUBLE_EQ(12.0, tagged.height);
  EXPECT_DOUBLE_EQ(4.0, tagged.crown_radius);

  const tile_map::MapTree plain = tile_map::TreeFromTags({29.45, -98.6}, {{"natural", "tree"}});
  EXPECT_DOUBLE_EQ(8.0, plain.height);
  EXPECT_DOUBLE_EQ(3.0, plain.crown_radius);
}

TEST(TreeLayer, FindsTreesInAnImageryTile)
{
  QImage image(256, 256, QImage::Format_RGB32);
  image.fill(GRASS);
  for (int y = 100; y < 140; ++y) {
    for (int x = 60; x < 100; ++x) {
      image.setPixel(x, y, CANOPY);
    }
  }
  const int32_t level = 17;
  const int64_t x = 29622;
  const int64_t y = 54308;
  const std::vector<Crown> crowns = tile_map::TreeLayer::CrownsFromImage(image, level, x, y);
  ASSERT_FALSE(crowns.empty());

  // Every crown is over the canopy block, in this tile.
  const double n = std::pow(2.0, level);
  const double west = (x + 60.0 / 256.0) / n * 360.0 - 180.0;
  const double east = (x + 100.0 / 256.0) / n * 360.0 - 180.0;
  const double north = tile_map::TileRowLatitude(level, y + 100.0 / 256.0);
  const double south = tile_map::TileRowLatitude(level, y + 140.0 / 256.0);
  for (const Crown & crown : crowns) {
    EXPECT_GT(crown.center.longitude, west);
    EXPECT_LT(crown.center.longitude, east);
    EXPECT_GT(crown.center.latitude, south);
    EXPECT_LT(crown.center.latitude, north);
  }
}

TEST(Scene25d, LeansRoofsUpTheScreen)
{
  const tf2::Vector3 up(0.0, 1.0, 0.0);
  const tf2::Vector3 right(1.0, 0.0, 0.0);
  const tile_map::ProjectedBuilding building = Square(0.0, 0.0, 10.0, 10.0);

  tile_map::Batch25d batch;
  batch.AddBuilding(building, up, right, 0.5);
  // Three walls (the back one faces away) of two triangles, then the roof.
  ASSERT_EQ(3u * 6u + 6u, batch.VertexCount());
  // The roof is the last six vertices, 10 m * 0.5 up the screen.
  const std::vector<double> & v = batch.Vertices();
  double lowest_roof_y = 1e9;
  for (size_t i = 18; i < 24; ++i) {
    lowest_roof_y = std::min(lowest_roof_y, v[2 * i + 1]);
  }
  EXPECT_DOUBLE_EQ(5.0, lowest_roof_y);

  // No lean: flat, no walls.
  batch.Clear();
  batch.AddBuilding(building, up, right, 0.0);
  EXPECT_EQ(6u, batch.VertexCount());
}

TEST(Scene25d, DrawsCrownsAsRoundTops)
{
  const tf2::Vector3 up(0.0, 1.0, 0.0);
  const tf2::Vector3 right(1.0, 0.0, 0.0);
  tile_map::ProjectedCrown crown;
  crown.center = tf2::Vector3(0.0, 0.0, 0.0);
  crown.radius = 3.0;
  crown.height = 10.0;

  tile_map::Batch25d batch;
  batch.AddCrown(crown, up, right, 0.0);
  const size_t flat = batch.VertexCount();
  EXPECT_GT(flat, 0u);
  batch.Clear();
  batch.AddCrown(crown, up, right, 0.6);
  // Standing up adds a trunk and the crown's underside and sides.
  EXPECT_GT(batch.VertexCount(), flat);
}

TEST(Scene25d, DrawsFromTheTopOfTheScreenDown)
{
  const tf2::Vector3 up(0.0, 1.0, 0.0);
  const tf2::Vector3 right(1.0, 0.0, 0.0);
  // A building higher up the screen than a tree, so behind it.
  const tile_map::ProjectedBuilding far = Square(0.0, 100.0, 10.0, 10.0);
  tile_map::ProjectedCrown near;
  near.center = tf2::Vector3(5.0, 0.0, 0.0);
  near.radius = 3.0;
  near.height = 8.0;

  EXPECT_GT(tile_map::FrontEdge(far, up), tile_map::FrontEdge(near, up));

  tile_map::Batch25d batch;
  tile_map::BuildBatch({&far}, {&near}, up, right, 0.5, batch);
  ASSERT_GT(batch.VertexCount(), 0u);
  // The building comes first.
  EXPECT_GE(batch.Vertices()[1], 100.0);
  // The tree comes last.
  EXPECT_LT(batch.Vertices()[batch.Vertices().size() - 1], 50.0);
}

TEST(Scene25d, DrapesTheImageryOverARoof)
{
  const tf2::Vector3 up(0.0, 1.0, 0.0);
  const tf2::Vector3 right(1.0, 0.0, 0.0);
  tile_map::Batch25d batch;
  batch.SetImagery({{LEVEL, 100, 200, 7}});
  // A roof in the middle of tile (100, 200), with no lean so it stays put.
  batch.AddBuilding(MappedSquare(100.25, 200.25, 0.5), up, right, 0.0);

  ASSERT_EQ(1u, batch.Runs().size());
  EXPECT_EQ(7u, batch.Runs()[0].texture);
  EXPECT_NEAR(0.25, TexturedArea(batch, 7), 1e-9);
  // Texture coordinates are the roof's place in the tile, with v up.
  const std::vector<float> & uv = batch.TexCoords();
  for (size_t i = 0; i < batch.VertexCount(); ++i) {
    EXPECT_NEAR(batch.Vertices()[2 * i] - 100.0, uv[2 * i], 1e-6);
    EXPECT_NEAR(1.0 - (-batch.Vertices()[2 * i + 1] - 200.0), uv[2 * i + 1], 1e-6);
  }
}

TEST(Scene25d, CutsARoofAlongTileEdges)
{
  const tf2::Vector3 up(0.0, 1.0, 0.0);
  const tf2::Vector3 right(1.0, 0.0, 0.0);
  tile_map::Batch25d batch;
  batch.SetImagery({{LEVEL, 100, 200, 7}, {LEVEL, 101, 200, 8}});
  // Half in each of two tiles side by side.
  batch.AddBuilding(MappedSquare(100.75, 200.25, 0.5), up, right, 0.0);

  EXPECT_NEAR(0.125, TexturedArea(batch, 7), 1e-9);
  EXPECT_NEAR(0.125, TexturedArea(batch, 8), 1e-9);
  EXPECT_NEAR(0.0, TexturedArea(batch, 0), 1e-9);
}

TEST(Scene25d, DrawsRoofsPlainWhereTheImageryIsMissing)
{
  const tf2::Vector3 up(0.0, 1.0, 0.0);
  const tf2::Vector3 right(1.0, 0.0, 0.0);
  tile_map::Batch25d batch;
  // Only one of the two tiles the roof needs has loaded.
  batch.SetImagery({{LEVEL, 100, 200, 7}});
  batch.AddBuilding(MappedSquare(100.75, 200.25, 0.5), up, right, 0.0);
  // Each roof triangle is either fully draped or fully plain; none is left
  // half done.
  EXPECT_NEAR(0.25, TexturedArea(batch, 7) + TexturedArea(batch, 0), 1e-9);
  EXPECT_LT(TexturedArea(batch, 7), 0.25 - 1e-9);

  // With no imagery, everything is plain.
  tile_map::Batch25d plain;
  plain.AddBuilding(MappedSquare(100.75, 200.25, 0.5), up, right, 0.0);
  ASSERT_EQ(1u, plain.Runs().size());
  EXPECT_EQ(0u, plain.Runs()[0].texture);
}

TEST(Scene25d, PrefersTheFinerImagery)
{
  const tf2::Vector3 up(0.0, 1.0, 0.0);
  const tf2::Vector3 right(1.0, 0.0, 0.0);
  tile_map::Batch25d batch;
  // The coarser tile (50, 100) at level 9 also covers the roof.
  batch.SetImagery({{LEVEL - 1, 50, 100, 3}, {LEVEL, 100, 200, 7}});
  batch.AddBuilding(MappedSquare(100.25, 200.25, 0.5), up, right, 0.0);
  EXPECT_NEAR(0.25, TexturedArea(batch, 7), 1e-9);
  EXPECT_NEAR(0.0, TexturedArea(batch, 3), 1e-9);
}

TEST(Scene25d, DrapesTheImageryOverTreetops)
{
  const tf2::Vector3 up(0.0, 1.0, 0.0);
  const tf2::Vector3 right(1.0, 0.0, 0.0);
  tile_map::ProjectedCrown crown;
  crown.center = tf2::Vector3(0.0, 0.0, 0.0);
  crown.radius = 3.0;
  crown.height = 10.0;
  // In the middle of tile (100, 200) at LEVEL, a few metres across.
  crown.mercator_x = 100.5 * TILE;
  crown.mercator_y = 200.5 * TILE;
  crown.mercator_per_meter = 1e-8;

  tile_map::Batch25d batch;
  batch.SetImagery({{LEVEL, 100, 200, 7}});
  batch.AddCrown(crown, up, right, 0.6);
  // The trunk and sides are plain; the top is draped.
  ASSERT_GE(batch.Runs().size(), 2u);
  EXPECT_EQ(0u, batch.Runs().front().texture);
  EXPECT_EQ(7u, batch.Runs().back().texture);

  // Off the loaded imagery, the whole top is plain.
  crown.mercator_x = 300.5 * TILE;
  tile_map::Batch25d plain;
  plain.SetImagery({{LEVEL, 100, 200, 7}});
  plain.AddCrown(crown, up, right, 0.6);
  for (const auto & run : plain.Runs()) {
    EXPECT_EQ(0u, run.texture);
  }
}
