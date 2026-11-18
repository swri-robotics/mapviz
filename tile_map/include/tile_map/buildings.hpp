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

#ifndef TILE_MAP__BUILDINGS_HPP_
#define TILE_MAP__BUILDINGS_HPP_

#include <QByteArray>
#include <QString>

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace tile_map
{
struct LatLon
{
  double latitude;
  double longitude;
};

/// A building footprint and how tall it is.
struct Building
{
  /// The OpenStreetMap way or relation id, or 0 when the data doesn't say.
  /// Overpass returns a building in every area it touches, so the id is what
  /// keeps it from being drawn twice.
  int64_t id = 0;
  /// The outline, without repeating the first point at the end.
  std::vector<LatLon> outline;
  /// Whether to draw a wall along the edge from outline[i] to outline[i + 1]
  /// (the last entry is the closing edge).  False where a vector tile cut the
  /// building at its edge, and everywhere for a free-standing roof.
  std::vector<bool> walls;
  /// Top of the building above the ground, in metres.
  double height = 0.0;
  /// Bottom of the building above the ground, for parts that overhang.
  double min_height = 0.0;
};

/// An area OpenStreetMap maps as wood or forest.
struct Woodland
{
  /// The outline, without repeating the first point at the end.
  std::vector<LatLon> outline;
};

/// A single tree OpenStreetMap maps as a point.
struct MapTree
{
  LatLon position;
  /// Metres above the ground.
  double height;
  /// Radius of the crown, in metres.
  double crown_radius;
};

/// Everything the 2.5D layers read from one area of map data.
struct MapFeatures
{
  std::vector<Building> buildings;
  std::vector<Woodland> woodlands;
  std::vector<MapTree> trees;
};

/// Metres in a length tag such as "12", "12.5 m", "40 ft" or "12'6\"", or a
/// negative number if it can't be read.
double ParseLength(const std::string & text);

/**
 * Height and base height of a building from its OpenStreetMap tags: "height"
 * (or "building:height"), else 3.5 m per "building:levels" plus 1 m for the
 * roof, else a typical height for its "building" type.  The base comes from
 * "min_height", else 3.5 m per "building:min_level".
 */
void BuildingHeightFromTags(
  const std::map<std::string, std::string> & tags, double & height, double & min_height);

/// A mapped tree's height and crown radius from its OpenStreetMap tags:
/// "height" and "diameter_crown", else typical values.
MapTree TreeFromTags(const LatLon & position, const std::map<std::string, std::string> & tags);

/// The Overpass QL query for every building, wood, forest and single tree in
/// a latitude/longitude box.
QString OverpassFeatureQuery(double south, double west, double north, double east);

/**
 * The features in an Overpass API JSON response to OverpassFeatureQuery().
 * Buildings, woods and forests are closed ways and the closed outer ways of
 * multipolygon relations; trees are nodes.
 *
 * @param[out] error Set to a description when the response can't be read.
 */
MapFeatures ParseOverpassFeatures(const QByteArray & json, QString * error = nullptr);

/// Just the buildings from ParseOverpassFeatures().
std::vector<Building> ParseOverpassBuildings(const QByteArray & json, QString * error = nullptr);

/**
 * The features in an OpenMapTiles vector tile (the schema Stadia Maps
 * serves): buildings from the "building" layer, with "render_height" and
 * "render_min_height" as their heights, and woods from the "landcover" layer
 * (class "wood").  Outlines are clipped to the tile, and the cut edges get no
 * wall, so a building split across tiles draws as one.  OpenMapTiles has no
 * single trees.
 *
 * @param[out] error Set to a description when the tile can't be decoded.
 */
MapFeatures DecodeVectorTileFeatures(
  const QByteArray & tile, int32_t level, int64_t x, int64_t y, QString * error = nullptr);

/// Just the buildings from DecodeVectorTileFeatures().
std::vector<Building> DecodeVectorTileBuildings(
  const QByteArray & tile, int32_t level, int64_t x, int64_t y, QString * error = nullptr);

/**
 * Splits a simple polygon (any winding, no holes) into triangles by ear
 * clipping.  Returns three indices per triangle into `points`, given as
 * alternating x, y coordinates.  Falls back to a fan if the outline crosses
 * itself.
 */
std::vector<int> Triangulate(const std::vector<double> & points);
}  // namespace tile_map

#endif  // TILE_MAP__BUILDINGS_HPP_
