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
#include <QByteArray>
#include <QString>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <tile_map/building_layer.hpp>
#include <tile_map/buildings.hpp>
#include <tile_map/terrain.hpp>

using tile_map::Building;
using tile_map::BuildingLayer;

namespace
{
// ---------------------------------------------------------------------------
// Just enough of a protocol buffer encoder to build vector tiles for the
// decoder to read.

std::string Varint(uint64_t value)
{
  std::string out;
  while (value >= 0x80) {
    out.push_back(static_cast<char>((value & 0x7f) | 0x80));
    value >>= 7;
  }
  out.push_back(static_cast<char>(value));
  return out;
}

std::string Key(uint32_t field, uint32_t wire_type)
{
  return Varint((static_cast<uint64_t>(field) << 3) | wire_type);
}

std::string Bytes(uint32_t field, const std::string & bytes)
{
  return Key(field, 2) + Varint(bytes.size()) + bytes;
}

std::string VarintField(uint32_t field, uint64_t value)
{
  return Key(field, 0) + Varint(value);
}

std::string Packed(uint32_t field, const std::vector<uint32_t> & values)
{
  std::string packed;
  for (uint32_t value : values) {
    packed += Varint(value);
  }
  return Bytes(field, packed);
}

std::string DoubleValue(double value)
{
  std::string bits(8, '\0');
  std::memcpy(&bits[0], &value, 8);
  return Key(3, 1) + bits;
}

uint32_t ZigZag(int32_t value)
{
  return (static_cast<uint32_t>(value) << 1) ^ static_cast<uint32_t>(value >> 31);
}

/// Geometry commands for one closed ring.
std::vector<uint32_t> Ring(const std::vector<std::pair<int, int>> & points)
{
  std::vector<uint32_t> commands;
  int x = 0;
  int y = 0;
  commands.push_back((1u << 3) | 1u);  // MoveTo 1
  commands.push_back(ZigZag(points[0].first - x));
  commands.push_back(ZigZag(points[0].second - y));
  x = points[0].first;
  y = points[0].second;
  commands.push_back((static_cast<uint32_t>(points.size() - 1) << 3) | 2u);  // LineTo
  for (size_t i = 1; i < points.size(); ++i) {
    commands.push_back(ZigZag(points[i].first - x));
    commands.push_back(ZigZag(points[i].second - y));
    x = points[i].first;
    y = points[i].second;
  }
  commands.push_back((1u << 3) | 7u);  // ClosePath
  return commands;
}

std::string Feature(
  uint32_t type, const std::vector<uint32_t> & tags,
  const std::vector<uint32_t> & geometry)
{
  return Packed(2, tags) + VarintField(3, type) + Packed(4, geometry);
}

/// A tile with a "water" layer to skip and a "building" layer.
QByteArray TestTile()
{
  // Keys: 0 render_height, 1 render_min_height, 2 hide_3d.
  // Values: 0 -> 20.0, 1 -> 5.0, 2 -> true.
  const std::string values =
    Bytes(4, DoubleValue(20.0)) + Bytes(4, DoubleValue(5.0)) + Bytes(4, VarintField(7, 1));
  const std::string keys = Bytes(3, "render_height") + Bytes(3, "render_min_height") +
    Bytes(3, "hide_3d");

  // Exterior rings run clockwise on screen, which is a positive area in tile
  // coordinates (y down).
  const std::string square = Feature(
    3, {0, 0, 1, 1}, Ring({{1000, 1000}, {2000, 1000}, {2000, 2000}, {1000, 2000}}));
  const std::string across_edge = Feature(
    3, {0, 0}, Ring({{3000, 1000}, {5000, 1000}, {5000, 2000}, {3000, 2000}}));
  const std::string hidden = Feature(
    3, {0, 0, 2, 2}, Ring({{100, 100}, {200, 100}, {200, 200}, {100, 200}}));
  const std::string point = Feature(1, {0, 0}, {(1u << 3) | 1u, ZigZag(10), ZigZag(10)});

  const std::string building_layer = VarintField(15, 2) + Bytes(1, "building") +
    Bytes(2, square) + Bytes(2, across_edge) + Bytes(2, hidden) + Bytes(2, point) +
    keys + values + VarintField(5, 4096);
  const std::string water_layer = VarintField(15, 2) + Bytes(1, "water") +
    Bytes(2, square) + keys + values + VarintField(5, 4096);

  const std::string tile = Bytes(3, water_layer) + Bytes(3, building_layer);
  return QByteArray(tile.data(), static_cast<int>(tile.size()));
}

/// A tile with a "landcover" layer holding a wood and a field.
QByteArray LandcoverTile()
{
  // Key 0 "class"; values 0 "wood", 1 "farmland".
  const std::string values = Bytes(4, Bytes(1, "wood")) + Bytes(4, Bytes(1, "farmland"));
  const std::string keys = Bytes(3, "class");
  const std::string wood = Feature(
    3, {0, 0}, Ring({{100, 100}, {900, 100}, {900, 900}, {100, 900}}));
  const std::string field = Feature(
    3, {0, 1}, Ring({{1000, 1000}, {1900, 1000}, {1900, 1900}, {1000, 1900}}));
  const std::string layer = VarintField(15, 2) + Bytes(1, "landcover") +
    Bytes(2, wood) + Bytes(2, field) + keys + values + VarintField(5, 4096);
  const std::string tile = Bytes(3, layer);
  return QByteArray(tile.data(), static_cast<int>(tile.size()));
}

double TriangleArea(const std::vector<double> & points, const std::vector<int> & triangles)
{
  double area = 0.0;
  for (size_t i = 0; i + 2 < triangles.size(); i += 3) {
    const double ax = points[2 * triangles[i]];
    const double ay = points[2 * triangles[i] + 1];
    const double bx = points[2 * triangles[i + 1]];
    const double by = points[2 * triangles[i + 1] + 1];
    const double cx = points[2 * triangles[i + 2]];
    const double cy = points[2 * triangles[i + 2] + 1];
    area += std::abs((bx - ax) * (cy - ay) - (by - ay) * (cx - ax)) / 2.0;
  }
  return area;
}
}  // namespace

TEST(Buildings, ParsesLengths)
{
  EXPECT_DOUBLE_EQ(12.0, tile_map::ParseLength("12"));
  EXPECT_DOUBLE_EQ(12.5, tile_map::ParseLength("12.5"));
  EXPECT_DOUBLE_EQ(12.5, tile_map::ParseLength("12,5"));
  EXPECT_DOUBLE_EQ(7.0, tile_map::ParseLength("7 m"));
  EXPECT_DOUBLE_EQ(7.0, tile_map::ParseLength("7m"));
  EXPECT_NEAR(12.192, tile_map::ParseLength("40 ft"), 1e-9);
  EXPECT_NEAR(12.192, tile_map::ParseLength("40'"), 1e-9);
  EXPECT_NEAR(3.81, tile_map::ParseLength("12'6\""), 1e-9);
  EXPECT_LT(tile_map::ParseLength(""), 0.0);
  EXPECT_LT(tile_map::ParseLength("tall"), 0.0);
  EXPECT_LT(tile_map::ParseLength("-3"), 0.0);
}

TEST(Buildings, WorksOutHeightsFromTags)
{
  double height = 0.0;
  double min_height = 0.0;

  tile_map::BuildingHeightFromTags({{"building", "yes"}, {"height", "21"}}, height, min_height);
  EXPECT_DOUBLE_EQ(21.0, height);
  EXPECT_DOUBLE_EQ(0.0, min_height);

  tile_map::BuildingHeightFromTags(
    {{"building", "yes"}, {"building:levels", "4"}}, height, min_height);
  EXPECT_DOUBLE_EQ(15.0, height);

  // "3;4" is how OSM lists a building with parts of different heights.
  tile_map::BuildingHeightFromTags(
    {{"building", "yes"}, {"building:levels", "3;4"}}, height, min_height);
  EXPECT_DOUBLE_EQ(11.5, height);

  tile_map::BuildingHeightFromTags({{"building", "apartments"}}, height, min_height);
  EXPECT_DOUBLE_EQ(12.0, height);
  tile_map::BuildingHeightFromTags({{"building", "shed"}}, height, min_height);
  EXPECT_DOUBLE_EQ(2.5, height);
  tile_map::BuildingHeightFromTags({{"building", "yes"}}, height, min_height);
  EXPECT_DOUBLE_EQ(6.0, height);

  tile_map::BuildingHeightFromTags(
    {{"building", "yes"}, {"height", "30"}, {"min_height", "10"}}, height, min_height);
  EXPECT_DOUBLE_EQ(10.0, min_height);
  tile_map::BuildingHeightFromTags(
    {{"building", "yes"}, {"height", "30"}, {"building:min_level", "2"}}, height, min_height);
  EXPECT_DOUBLE_EQ(7.0, min_height);
  // A base above the top is clamped to it.
  tile_map::BuildingHeightFromTags(
    {{"building", "yes"}, {"height", "5"}, {"min_height", "10"}}, height, min_height);
  EXPECT_DOUBLE_EQ(5.0, min_height);
}

TEST(Buildings, BuildsTheOverpassQuery)
{
  const QString query = tile_map::OverpassFeatureQuery(29.4, -98.7, 29.5, -98.6);
  const QString box = "(29.4000000,-98.7000000,29.5000000,-98.6000000)";
  EXPECT_TRUE(query.startsWith("[out:json]"));
  EXPECT_TRUE(query.contains("way[\"building\"]" + box));
  EXPECT_TRUE(query.contains("relation[\"building\"][\"type\"=\"multipolygon\"]" + box));
  EXPECT_TRUE(query.contains("way[\"natural\"=\"wood\"]" + box));
  EXPECT_TRUE(query.contains("way[\"landuse\"=\"forest\"]" + box));
  EXPECT_TRUE(query.contains("node[\"natural\"=\"tree\"]" + box));
  EXPECT_TRUE(query.endsWith("out tags geom;"));
}

TEST(Buildings, ParsesMappedWoodsAndTrees)
{
  const QByteArray json =
    R"({
    "elements": [
      {"type": "node", "id": 1, "lat": 29.45, "lon": -98.6,
       "tags": {"natural": "tree", "height": "15"}},
      {"type": "way", "id": 2, "tags": {"natural": "wood"},
       "geometry": [{"lat": 0, "lon": 0}, {"lat": 0, "lon": 1}, {"lat": 1, "lon": 1},
                    {"lat": 0, "lon": 0}]},
      {"type": "relation", "id": 3, "tags": {"landuse": "forest", "type": "multipolygon"},
       "members": [
         {"type": "way", "role": "outer",
          "geometry": [{"lat": 5, "lon": 5}, {"lat": 5, "lon": 6}, {"lat": 6, "lon": 6},
                       {"lat": 5, "lon": 5}]}]},
      {"type": "way", "id": 4, "tags": {"building": "yes", "natural": "wood"},
       "geometry": [{"lat": 2, "lon": 2}, {"lat": 2, "lon": 3}, {"lat": 3, "lon": 3},
                    {"lat": 2, "lon": 2}]}
    ]})";

  QString error;
  const tile_map::MapFeatures features = tile_map::ParseOverpassFeatures(json, &error);
  EXPECT_TRUE(error.isEmpty());
  ASSERT_EQ(1u, features.trees.size());
  EXPECT_DOUBLE_EQ(15.0, features.trees[0].height);
  EXPECT_DOUBLE_EQ(29.45, features.trees[0].position.latitude);
  EXPECT_EQ(2u, features.woodlands.size());
  // Tagged as both, it is a building.
  ASSERT_EQ(1u, features.buildings.size());
  EXPECT_EQ(4, features.buildings[0].id);
}

TEST(Buildings, ParsesOverpassResponses)
{
  const QByteArray json =
    R"({
    "elements": [
      {"type": "way", "id": 11, "tags": {"building": "yes", "building:levels": "2"},
       "geometry": [{"lat": 0, "lon": 0}, {"lat": 0, "lon": 1}, {"lat": 1, "lon": 1},
                    {"lat": 0, "lon": 0}]},
      {"type": "way", "id": 12, "tags": {"building": "yes"},
       "geometry": [{"lat": 0, "lon": 0}, {"lat": 0, "lon": 1}, {"lat": 1, "lon": 1}]},
      {"type": "way", "id": 13, "tags": {"building": "roof"},
       "geometry": [{"lat": 2, "lon": 2}, {"lat": 2, "lon": 3}, {"lat": 3, "lon": 3},
                    {"lat": 2, "lon": 2}]},
      {"type": "relation", "id": 11, "tags": {"building": "yes", "height": "30"},
       "members": [
         {"type": "way", "role": "outer",
          "geometry": [{"lat": 5, "lon": 5}, {"lat": 5, "lon": 6}, {"lat": 6, "lon": 6},
                       {"lat": 6, "lon": 5}, {"lat": 5, "lon": 5}]},
         {"type": "way", "role": "inner",
          "geometry": [{"lat": 5.2, "lon": 5.2}, {"lat": 5.2, "lon": 5.4},
                       {"lat": 5.4, "lon": 5.4}, {"lat": 5.2, "lon": 5.2}]}]}
    ]})";

  QString error;
  const std::vector<Building> buildings = tile_map::ParseOverpassBuildings(json, &error);
  EXPECT_TRUE(error.isEmpty());
  // The way that doesn't close and the inner ring are left out.
  ASSERT_EQ(3u, buildings.size());

  EXPECT_EQ(11, buildings[0].id);
  EXPECT_EQ(3u, buildings[0].outline.size());
  EXPECT_DOUBLE_EQ(8.0, buildings[0].height);
  EXPECT_EQ(std::vector<bool>(3, true), buildings[0].walls);

  // A free-standing roof has no walls.
  EXPECT_EQ(13, buildings[1].id);
  EXPECT_EQ(std::vector<bool>(3, false), buildings[1].walls);

  // Relation ids are kept apart from way ids.
  EXPECT_EQ(-11, buildings[2].id);
  EXPECT_EQ(4u, buildings[2].outline.size());
  EXPECT_DOUBLE_EQ(30.0, buildings[2].height);
}

TEST(Buildings, RejectsUnusableOverpassResponses)
{
  QString error;
  EXPECT_TRUE(tile_map::ParseOverpassBuildings("<html>Too busy</html>", &error).empty());
  EXPECT_FALSE(error.isEmpty());

  // A query that ran out of time comes back with partial data and a remark.
  error.clear();
  EXPECT_TRUE(
    tile_map::ParseOverpassBuildings(
      R"({"elements": [], "remark": "runtime error: Query timed out"})", &error).empty());
  EXPECT_TRUE(error.contains("timed out"));

  // An empty area is fine.
  error.clear();
  EXPECT_TRUE(tile_map::ParseOverpassBuildings(R"({"elements": []})", &error).empty());
  EXPECT_TRUE(error.isEmpty());
}

TEST(Buildings, DecodesVectorTileBuildings)
{
  const int32_t level = 14;
  const int64_t x = 3703;
  const int64_t y = 6788;
  QString error;
  const std::vector<Building> buildings =
    tile_map::DecodeVectorTileBuildings(TestTile(), level, x, y, &error);
  EXPECT_TRUE(error.isEmpty()) << error.toStdString();
  // The hidden building, the point and the water layer are skipped.
  ASSERT_EQ(2u, buildings.size());

  const Building & square = buildings[0];
  EXPECT_DOUBLE_EQ(20.0, square.height);
  EXPECT_DOUBLE_EQ(5.0, square.min_height);
  ASSERT_EQ(4u, square.outline.size());
  EXPECT_EQ(std::vector<bool>(4, true), square.walls);
  const double n = std::pow(2.0, level);
  EXPECT_NEAR(
    (static_cast<double>(x) + 1000.0 / 4096.0) / n * 360.0 - 180.0,
    square.outline[0].longitude, 1e-12);
  EXPECT_NEAR(
    tile_map::TileRowLatitude(level, static_cast<double>(y) + 1000.0 / 4096.0),
    square.outline[0].latitude, 1e-12);

  // Cut at the east edge of the tile, with no wall along the cut.
  const Building & cut = buildings[1];
  ASSERT_EQ(4u, cut.outline.size());
  const double east = (static_cast<double>(x) + 1.0) / n * 360.0 - 180.0;
  int on_edge = 0;
  for (size_t i = 0; i < cut.outline.size(); ++i) {
    EXPECT_LE(cut.outline[i].longitude, east + 1e-12);
    const auto & a = cut.outline[i];
    const auto & b = cut.outline[(i + 1) % cut.outline.size()];
    const bool along_cut = std::abs(a.longitude - east) < 1e-12 &&
      std::abs(b.longitude - east) < 1e-12;
    EXPECT_EQ(!along_cut, cut.walls[i]) << "edge " << i;
    on_edge += along_cut ? 1 : 0;
  }
  EXPECT_EQ(1, on_edge);
}

TEST(Buildings, DecodesVectorTileWoods)
{
  QString error;
  const tile_map::MapFeatures features =
    tile_map::DecodeVectorTileFeatures(LandcoverTile(), 14, 3703, 6788, &error);
  EXPECT_TRUE(error.isEmpty()) << error.toStdString();
  EXPECT_TRUE(features.buildings.empty());
  // The wood, not the field.
  ASSERT_EQ(1u, features.woodlands.size());
  EXPECT_EQ(4u, features.woodlands[0].outline.size());
}

TEST(Buildings, RejectsCorruptVectorTiles)
{
  QString error;
  QByteArray corrupt = TestTile();
  corrupt.truncate(corrupt.size() / 2);
  EXPECT_TRUE(tile_map::DecodeVectorTileBuildings(corrupt, 14, 0, 0, &error).empty());
  EXPECT_FALSE(error.isEmpty());

  error.clear();
  EXPECT_TRUE(tile_map::DecodeVectorTileBuildings(QByteArray(), 14, 0, 0, &error).empty());
  EXPECT_TRUE(error.isEmpty());
}

TEST(Buildings, TriangulatesConvexAndConcaveOutlines)
{
  const std::vector<double> square = {0, 0, 1, 0, 1, 1, 0, 1};
  std::vector<int> triangles = tile_map::Triangulate(square);
  EXPECT_EQ(6u, triangles.size());
  EXPECT_NEAR(1.0, TriangleArea(square, triangles), 1e-12);

  // An L shape, given clockwise.
  const std::vector<double> l_shape = {0, 0, 0, 2, 1, 2, 1, 1, 2, 1, 2, 0};
  triangles = tile_map::Triangulate(l_shape);
  EXPECT_EQ(12u, triangles.size());
  EXPECT_NEAR(3.0, TriangleArea(l_shape, triangles), 1e-12);

  // A point in the middle of a straight edge adds no triangle.
  const std::vector<double> with_midpoint = {0, 0, 0.5, 0, 1, 0, 1, 1, 0, 1};
  triangles = tile_map::Triangulate(with_midpoint);
  EXPECT_NEAR(1.0, TriangleArea(with_midpoint, triangles), 1e-12);

  EXPECT_TRUE(tile_map::Triangulate({0, 0, 1, 1}).empty());
}

TEST(BuildingLayer, CoversTheViewWithZ14Tiles)
{
  // Around SwRI at 0.5 m per pixel: a small block of tiles that includes the
  // one under the centre.
  const auto tiles = BuildingLayer::TilesInView(29.45197, -98.61371, 0.5, 1200, 900);
  ASSERT_FALSE(tiles.empty());
  EXPECT_LE(tiles.size(), 4u);
  EXPECT_NE(
    std::find(tiles.begin(), tiles.end(), BuildingLayer::TileKey(3703, 6788)), tiles.end());

  // Zoomed out too far, or a view too large for the tile limit.
  EXPECT_TRUE(BuildingLayer::TilesInView(29.45, -98.61, 4.0, 1200, 900).empty());
  EXPECT_TRUE(BuildingLayer::TilesInView(29.45, -98.61, 3.0, 8000, 8000).empty());
}

TEST(BuildingLayer, CachesEachSourceSeparately)
{
  EXPECT_EQ(
    "/cache/overpass2/14/3703/6788.json",
    BuildingLayer::CachePath("/cache", tile_map::BuildingSource::Overpass, {3703, 6788}));
  EXPECT_EQ(
    "/cache/stadia/14/3703/6788.pbf",
    BuildingLayer::CachePath("/cache", tile_map::BuildingSource::Stadia, {3703, 6788}));
}
