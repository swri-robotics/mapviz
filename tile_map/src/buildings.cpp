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

#include <tile_map/buildings.hpp>

#include <QRegularExpression>
#include <json/json.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <list>
#include <string_view>
#include <utility>

#include <tile_map/terrain.hpp>

namespace tile_map
{
namespace
{
/// Storey height used to turn a level count into metres.
constexpr double LEVEL_HEIGHT = 3.5;
/// Added to a height made from a level count, for the roof.
constexpr double ROOF_ALLOWANCE = 1.0;
/// Used when a building has no height, levels or recognised type.
constexpr double DEFAULT_HEIGHT = 6.0;

/// Typical heights for common "building" values, in metres.
double TypicalHeight(const std::string & type)
{
  static const std::map<std::string, double> heights = {
    {"apartments", 12.0}, {"dormitory", 12.0}, {"hotel", 15.0}, {"hospital", 12.0},
    {"office", 12.0}, {"commercial", 8.0}, {"retail", 6.0}, {"supermarket", 7.0},
    {"school", 8.0}, {"university", 10.0}, {"college", 10.0}, {"public", 8.0},
    {"civic", 8.0}, {"government", 8.0}, {"church", 10.0}, {"cathedral", 20.0},
    {"industrial", 9.0}, {"warehouse", 9.0}, {"factory", 10.0}, {"hangar", 12.0},
    {"house", 6.0}, {"detached", 6.0}, {"semidetached_house", 6.0}, {"terrace", 7.0},
    {"residential", 6.0}, {"bungalow", 4.5}, {"farm", 6.0}, {"barn", 7.0},
    {"garage", 3.0}, {"garages", 3.0}, {"shed", 2.5}, {"carport", 2.5}, {"hut", 2.5},
    {"cabin", 3.5}, {"kiosk", 3.0}, {"service", 3.0}, {"grandstand", 8.0}, {"roof", 4.0},
    {"parking", 9.0}, {"train_station", 10.0}, {"transportation", 8.0}, {"stadium", 20.0},
  };
  const auto it = heights.find(type);
  return it == heights.end() ? DEFAULT_HEIGHT : it->second;
}

/// The first number in `text`, or a negative number if there is none.
double FirstNumber(const std::string & text)
{
  static const QRegularExpression number("(\\d+(?:\\.\\d+)?)");
  const QRegularExpressionMatch match = number.match(QString::fromStdString(text));
  return match.hasMatch() ? match.captured(1).toDouble() : -1.0;
}

std::map<std::string, std::string> Tags(const Json::Value & tags)
{
  std::map<std::string, std::string> result;
  if (tags.isObject()) {
    for (const std::string & key : tags.getMemberNames()) {
      result[key] = tags[key].asString();
    }
  }
  return result;
}

/// A closed Overpass geometry as an outline, or an empty one if it isn't
/// closed or is too short to have an area.
std::vector<LatLon> ClosedOutline(const Json::Value & geometry)
{
  std::vector<LatLon> outline;
  if (!geometry.isArray()) {
    return outline;
  }
  for (const Json::Value & node : geometry) {
    if (!node["lat"].isNumeric() || !node["lon"].isNumeric()) {
      return {};
    }
    outline.push_back({node["lat"].asDouble(), node["lon"].asDouble()});
  }
  if (outline.size() < 4 ||
    outline.front().latitude != outline.back().latitude ||
    outline.front().longitude != outline.back().longitude)
  {
    return {};
  }
  outline.pop_back();
  return outline;
}

Building MakeBuilding(
  int64_t id, std::vector<LatLon> outline, const std::map<std::string, std::string> & tags)
{
  Building building;
  building.id = id;
  building.outline = std::move(outline);
  BuildingHeightFromTags(tags, building.height, building.min_height);
  // A free-standing roof (a canopy, a fuel station roof) has no walls.
  const auto type = tags.find("building");
  const bool roof_only = type != tags.end() && type->second == "roof";
  building.walls.assign(building.outline.size(), !roof_only);
  return building;
}

// ---------------------------------------------------------------------------
// A minimal reader for the parts of the protocol buffer encoding that Mapbox
// vector tiles use.

class PbfReader
{
public:
  PbfReader(const char * data, size_t size)
  : data_(reinterpret_cast<const uint8_t *>(data)), end_(data_ + size) {}
  explicit PbfReader(std::string_view bytes)
  : PbfReader(bytes.data(), bytes.size()) {}

  bool Ok() const {return ok_;}

  /// Moves to the next field; false at the end of the message or on an error.
  bool Next()
  {
    if (!ok_ || data_ >= end_) {
      return false;
    }
    const uint64_t key = Varint();
    field_ = static_cast<uint32_t>(key >> 3);
    wire_type_ = static_cast<uint32_t>(key & 7);
    return ok_;
  }

  uint32_t Field() const {return field_;}
  uint32_t WireType() const {return wire_type_;}

  uint64_t Varint()
  {
    uint64_t value = 0;
    for (int shift = 0; shift < 64; shift += 7) {
      if (data_ >= end_) {
        ok_ = false;
        return 0;
      }
      const uint8_t byte = *data_++;
      value |= static_cast<uint64_t>(byte & 0x7f) << shift;
      if ((byte & 0x80) == 0) {
        return value;
      }
    }
    ok_ = false;
    return 0;
  }

  std::string_view Bytes()
  {
    const uint64_t length = Varint();
    if (!ok_ || length > static_cast<uint64_t>(end_ - data_)) {
      ok_ = false;
      return {};
    }
    std::string_view bytes(reinterpret_cast<const char *>(data_), length);
    data_ += length;
    return bytes;
  }

  double Fixed(size_t size)
  {
    if (static_cast<size_t>(end_ - data_) < size) {
      ok_ = false;
      return 0.0;
    }
    uint64_t bits = 0;
    for (size_t i = 0; i < size; ++i) {
      bits |= static_cast<uint64_t>(data_[i]) << (8 * i);
    }
    data_ += size;
    if (size == 4) {
      float value;
      const uint32_t bits32 = static_cast<uint32_t>(bits);
      std::memcpy(&value, &bits32, sizeof(value));
      return value;
    }
    double value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
  }

  void Skip()
  {
    switch (wire_type_) {
      case 0: Varint(); break;
      case 1: Fixed(8); break;
      case 2: Bytes(); break;
      case 5: Fixed(4); break;
      default: ok_ = false; break;
    }
  }

  /// A packed repeated varint field.
  std::vector<uint32_t> Packed()
  {
    std::vector<uint32_t> values;
    PbfReader packed(Bytes());
    while (packed.data_ < packed.end_ && packed.ok_) {
      values.push_back(static_cast<uint32_t>(packed.Varint()));
    }
    ok_ = ok_ && packed.ok_;
    return values;
  }

private:
  const uint8_t * data_;
  const uint8_t * end_;
  bool ok_ = true;
  uint32_t field_ = 0;
  uint32_t wire_type_ = 0;
};

int32_t ZigZag(uint32_t value)
{
  return static_cast<int32_t>((value >> 1) ^ (~(value & 1) + 1));
}

/// The parts of a vector tile Value this decoder uses.
struct TileValue
{
  bool is_number = false;
  double number = 0.0;
  bool boolean = false;
};

TileValue ReadValue(std::string_view bytes)
{
  TileValue value;
  PbfReader reader(bytes);
  while (reader.Next()) {
    switch (reader.Field()) {
      case 2: value.is_number = true; value.number = reader.Fixed(4); break;
      case 3: value.is_number = true; value.number = reader.Fixed(8); break;
      case 4: value.is_number = true; value.number = static_cast<int64_t>(reader.Varint()); break;
      case 5: value.is_number = true; value.number = static_cast<double>(reader.Varint()); break;
      case 6: {
          const uint64_t raw = reader.Varint();
          value.is_number = true;
          value.number = static_cast<double>(static_cast<int64_t>((raw >> 1) ^ (~(raw & 1) + 1)));
          break;
        }
      case 7: value.boolean = reader.Varint() != 0; break;
      default: reader.Skip(); break;
    }
  }
  return value;
}

struct TilePoint
{
  double x;
  double y;
};

double RingArea(const std::vector<TilePoint> & ring)
{
  double area = 0.0;
  for (size_t i = 0; i < ring.size(); ++i) {
    const TilePoint & a = ring[i];
    const TilePoint & b = ring[(i + 1) % ring.size()];
    area += a.x * b.y - b.x * a.y;
  }
  return 0.5 * area;
}

/// Clips a ring to the half-plane where `inside` holds, as one step of
/// Sutherland-Hodgman clipping.
template<typename Inside, typename Intersect>
std::vector<TilePoint> ClipRing(
  const std::vector<TilePoint> & ring, Inside inside, Intersect intersect)
{
  std::vector<TilePoint> clipped;
  for (size_t i = 0; i < ring.size(); ++i) {
    const TilePoint & current = ring[i];
    const TilePoint & previous = ring[(i + ring.size() - 1) % ring.size()];
    const bool current_in = inside(current);
    const bool previous_in = inside(previous);
    if (current_in) {
      if (!previous_in) {
        clipped.push_back(intersect(previous, current));
      }
      clipped.push_back(current);
    } else if (previous_in) {
      clipped.push_back(intersect(previous, current));
    }
  }
  return clipped;
}

std::vector<TilePoint> ClipToTile(std::vector<TilePoint> ring, double extent)
{
  auto at_x = [](double x) {
      return [x](const TilePoint & a, const TilePoint & b) {
               const double t = (x - a.x) / (b.x - a.x);
               return TilePoint{x, a.y + t * (b.y - a.y)};
             };
    };
  auto at_y = [](double y) {
      return [y](const TilePoint & a, const TilePoint & b) {
               const double t = (y - a.y) / (b.y - a.y);
               return TilePoint{a.x + t * (b.x - a.x), y};
             };
    };
  ring = ClipRing(ring, [](const TilePoint & p) {return p.x >= 0.0;}, at_x(0.0));
  ring = ClipRing(ring, [extent](const TilePoint & p) {return p.x <= extent;}, at_x(extent));
  ring = ClipRing(ring, [](const TilePoint & p) {return p.y >= 0.0;}, at_y(0.0));
  ring = ClipRing(ring, [extent](const TilePoint & p) {return p.y <= extent;}, at_y(extent));
  return ring;
}

/// True if both ends of an edge lie along the same side of the tile, which
/// means the edge is where the tile cut the building rather than a real wall.
bool OnTileEdge(const TilePoint & a, const TilePoint & b, double extent)
{
  constexpr double tolerance = 1e-6;
  auto same = [](double u, double v, double w) {
      return std::abs(u - w) < tolerance && std::abs(v - w) < tolerance;
    };
  return same(a.x, b.x, 0.0) || same(a.x, b.x, extent) ||
         same(a.y, b.y, 0.0) || same(a.y, b.y, extent);
}

/// Decodes a feature's geometry commands into rings, in tile coordinates.
std::vector<std::vector<TilePoint>> DecodeRings(const std::vector<uint32_t> & geometry)
{
  std::vector<std::vector<TilePoint>> rings;
  std::vector<TilePoint> ring;
  int64_t x = 0;
  int64_t y = 0;
  size_t i = 0;
  while (i < geometry.size()) {
    const uint32_t command = geometry[i] & 0x7;
    const uint32_t count = geometry[i] >> 3;
    ++i;
    if (command == 1 || command == 2) {  // MoveTo, LineTo
      if (command == 1 && !ring.empty()) {
        ring.clear();
      }
      for (uint32_t n = 0; n < count; ++n) {
        if (i + 1 >= geometry.size()) {
          return rings;  // Truncated: keep the rings that were complete.
        }
        x += ZigZag(geometry[i]);
        y += ZigZag(geometry[i + 1]);
        i += 2;
        ring.push_back({static_cast<double>(x), static_cast<double>(y)});
      }
    } else if (command == 7) {  // ClosePath
      if (ring.size() >= 3) {
        rings.push_back(ring);
      }
      ring.clear();
    } else {
      return rings;
    }
  }
  return rings;
}
}  // namespace

double ParseLength(const std::string & text)
{
  const QString value = QString::fromStdString(text).trimmed().toLower().replace(',', '.');

  static const QRegularExpression feet_inches(
    "^(\\d+(?:\\.\\d+)?)\\s*'\\s*(?:(\\d+(?:\\.\\d+)?)\\s*\")?$");
  QRegularExpressionMatch match = feet_inches.match(value);
  if (match.hasMatch()) {
    const double inches = match.captured(2).isEmpty() ? 0.0 : match.captured(2).toDouble();
    return match.captured(1).toDouble() * 0.3048 + inches * 0.0254;
  }

  static const QRegularExpression number_unit(
    "^(\\d+(?:\\.\\d+)?)\\s*(m|meters?|metres?|ft|feet|foot)?$");
  match = number_unit.match(value);
  if (!match.hasMatch()) {
    return -1.0;
  }
  const double number = match.captured(1).toDouble();
  const QString unit = match.captured(2);
  if (unit == "ft" || unit == "feet" || unit == "foot") {
    return number * 0.3048;
  }
  return number;
}

void BuildingHeightFromTags(
  const std::map<std::string, std::string> & tags, double & height, double & min_height)
{
  auto tag = [&tags](const char * key) {
      const auto it = tags.find(key);
      return it == tags.end() ? std::string() : it->second;
    };

  height = ParseLength(tag("height"));
  if (height < 0.0) {
    height = ParseLength(tag("building:height"));
  }
  if (height < 0.0) {
    const double levels = FirstNumber(tag("building:levels"));
    if (levels >= 0.0) {
      height = levels * LEVEL_HEIGHT + ROOF_ALLOWANCE;
    }
  }
  if (height < 0.0) {
    height = TypicalHeight(tag("building"));
  }

  min_height = ParseLength(tag("min_height"));
  if (min_height < 0.0) {
    const double min_level = FirstNumber(tag("building:min_level"));
    min_height = min_level >= 0.0 ? min_level * LEVEL_HEIGHT : 0.0;
  }
  min_height = std::min(min_height, height);
}

QString OverpassBuildingQuery(double south, double west, double north, double east)
{
  const QString box = QString("(%1,%2,%3,%4)")
    .arg(south, 0, 'f', 7).arg(west, 0, 'f', 7).arg(north, 0, 'f', 7).arg(east, 0, 'f', 7);
  return "[out:json][timeout:90];(way[\"building\"]" + box +
         ";relation[\"building\"][\"type\"=\"multipolygon\"]" + box + ";);out tags geom;";
}

std::vector<Building> ParseOverpassBuildings(const QByteArray & json, QString * error)
{
  std::vector<Building> buildings;
  Json::Reader reader;
  Json::Value root;
  if (!reader.parse(json.toStdString(), root) || !root.isObject() ||
    !root["elements"].isArray())
  {
    if (error) {
      *error = "Overpass returned something other than building data";
    }
    return buildings;
  }
  // Overpass reports a query that ran out of time or memory in "remark",
  // alongside whatever partial result it has.  Partial data would leave gaps
  // that look like missing buildings, so treat it as a failure.
  if (root["remark"].isString() &&
    root["remark"].asString().find("error") != std::string::npos)
  {
    if (error) {
      *error = QString::fromStdString("Overpass: " + root["remark"].asString());
    }
    return buildings;
  }

  for (const Json::Value & element : root["elements"]) {
    const std::string type = element["type"].asString();
    const std::map<std::string, std::string> tags = Tags(element["tags"]);
    if (type == "way") {
      std::vector<LatLon> outline = ClosedOutline(element["geometry"]);
      if (!outline.empty()) {
        buildings.push_back(MakeBuilding(element["id"].asInt64(), std::move(outline), tags));
      }
    } else if (type == "relation" && element["members"].isArray()) {
      for (const Json::Value & member : element["members"]) {
        if (member["type"].asString() != "way" || member["role"].asString() != "outer") {
          continue;
        }
        std::vector<LatLon> outline = ClosedOutline(member["geometry"]);
        if (!outline.empty()) {
          // Relation and way ids are separate number spaces in OSM.
          buildings.push_back(MakeBuilding(-element["id"].asInt64(), std::move(outline), tags));
        }
      }
    }
  }
  return buildings;
}

std::vector<Building> DecodeVectorTileBuildings(
  const QByteArray & tile, int32_t level, int64_t x, int64_t y, QString * error)
{
  std::vector<Building> buildings;
  PbfReader tile_reader(tile.constData(), static_cast<size_t>(tile.size()));
  while (tile_reader.Next()) {
    if (tile_reader.Field() != 3 || tile_reader.WireType() != 2) {
      tile_reader.Skip();
      continue;
    }

    // One layer.  Its keys and values come after its features in practice,
    // but the encoding doesn't promise that, so read everything first.
    PbfReader layer(tile_reader.Bytes());
    std::string name;
    double extent = 4096.0;
    std::vector<std::string> keys;
    std::vector<TileValue> values;
    std::vector<std::string_view> features;
    while (layer.Next()) {
      switch (layer.Field()) {
        case 1: name = std::string(layer.Bytes()); break;
        case 2: features.push_back(layer.Bytes()); break;
        case 3: keys.emplace_back(layer.Bytes()); break;
        case 4: values.push_back(ReadValue(layer.Bytes())); break;
        case 5: extent = static_cast<double>(layer.Varint()); break;
        default: layer.Skip(); break;
      }
    }
    if (!layer.Ok()) {
      if (error) {
        *error = "The vector tile is corrupt";
      }
      return {};
    }
    if (name != "building" || extent <= 0.0) {
      continue;
    }

    for (std::string_view feature_bytes : features) {
      PbfReader feature(feature_bytes);
      uint32_t geometry_type = 0;
      std::vector<uint32_t> tags;
      std::vector<uint32_t> geometry;
      while (feature.Next()) {
        switch (feature.Field()) {
          case 2: tags = feature.Packed(); break;
          case 3: geometry_type = static_cast<uint32_t>(feature.Varint()); break;
          case 4: geometry = feature.Packed(); break;
          default: feature.Skip(); break;
        }
      }
      if (!feature.Ok() || geometry_type != 3) {  // 3 is POLYGON
        continue;
      }

      double height = -1.0;
      double min_height = 0.0;
      bool hide = false;
      for (size_t i = 0; i + 1 < tags.size(); i += 2) {
        if (tags[i] >= keys.size() || tags[i + 1] >= values.size()) {
          continue;
        }
        const std::string & key = keys[tags[i]];
        const TileValue & value = values[tags[i + 1]];
        if (key == "render_height" && value.is_number) {
          height = value.number;
        } else if (key == "render_min_height" && value.is_number) {
          min_height = value.number;
        } else if (key == "hide_3d") {
          hide = value.boolean;
        }
      }
      if (hide) {
        continue;
      }
      if (height < 0.0) {
        height = DEFAULT_HEIGHT;
      }

      for (const std::vector<TilePoint> & ring : DecodeRings(geometry)) {
        // Exterior rings have a positive area in tile coordinates; the
        // negative ones are courtyards, which are drawn roofed over.
        if (RingArea(ring) <= 0.0) {
          continue;
        }
        const std::vector<TilePoint> clipped = ClipToTile(ring, extent);
        if (clipped.size() < 3) {
          continue;
        }
        // Clipped pieces of one building are separate outlines in separate
        // tiles, so there is nothing to deduplicate and no id is kept.
        Building building;
        building.height = height;
        building.min_height = std::min(min_height, height);
        for (size_t i = 0; i < clipped.size(); ++i) {
          const TilePoint & point = clipped[i];
          const double fx = static_cast<double>(x) + point.x / extent;
          const double fy = static_cast<double>(y) + point.y / extent;
          building.outline.push_back(
            {TileRowLatitude(level, fy), fx / std::pow(2.0, level) * 360.0 - 180.0});
          building.walls.push_back(!OnTileEdge(point, clipped[(i + 1) % clipped.size()], extent));
        }
        buildings.push_back(std::move(building));
      }
    }
  }
  if (!tile_reader.Ok()) {
    if (error) {
      *error = "The vector tile is corrupt";
    }
    return {};
  }
  return buildings;
}

std::vector<int> Triangulate(const std::vector<double> & points)
{
  const int count = static_cast<int>(points.size() / 2);
  std::vector<int> triangles;
  if (count < 3) {
    return triangles;
  }
  auto px = [&points](int i) {return points[2 * i];};
  auto py = [&points](int i) {return points[2 * i + 1];};

  double area = 0.0;
  for (int i = 0; i < count; ++i) {
    const int j = (i + 1) % count;
    area += px(i) * py(j) - px(j) * py(i);
  }

  // Work counterclockwise.
  std::list<int> remaining;
  for (int i = 0; i < count; ++i) {
    remaining.push_back(area >= 0.0 ? i : count - 1 - i);
  }

  auto cross = [&](int a, int b, int c) {
      return (px(b) - px(a)) * (py(c) - py(b)) - (py(b) - py(a)) * (px(c) - px(b));
    };
  auto inside = [&](int p, int a, int b, int c) {
      // On an edge counts as inside, so an ear never cuts through a vertex.
      return cross(a, b, p) >= 0.0 && cross(b, c, p) >= 0.0 && cross(c, a, p) >= 0.0;
    };

  auto next = [&remaining](std::list<int>::iterator it) {
      ++it;
      return it == remaining.end() ? remaining.begin() : it;
    };
  auto prev = [&remaining](std::list<int>::iterator it) {
      return it == remaining.begin() ? std::prev(remaining.end()) : std::prev(it);
    };

  auto it = remaining.begin();
  size_t misses = 0;
  while (remaining.size() > 3) {
    const int a = *prev(it);
    const int b = *it;
    const int c = *next(it);
    const double turn = cross(a, b, c);
    bool ear = false;
    if (std::abs(turn) < 1e-12) {
      // A straight run (or a repeated point): drop the middle point.
      it = remaining.erase(it);
      if (it == remaining.end()) {
        it = remaining.begin();
      }
      misses = 0;
      continue;
    }
    if (turn > 0.0) {
      ear = true;
      for (int p : remaining) {
        if (p == a || p == b || p == c) {
          continue;
        }
        if ((px(p) == px(a) && py(p) == py(a)) || (px(p) == px(b) && py(p) == py(b)) ||
          (px(p) == px(c) && py(p) == py(c)))
        {
          continue;
        }
        if (inside(p, a, b, c)) {
          ear = false;
          break;
        }
      }
    }
    if (ear) {
      triangles.insert(triangles.end(), {a, b, c});
      it = remaining.erase(it);
      if (it == remaining.end()) {
        it = remaining.begin();
      }
      misses = 0;
    } else {
      it = next(it);
      if (++misses > remaining.size()) {
        // No ear anywhere: the outline crosses itself.  A fan still fills it.
        const int first = remaining.front();
        auto fan = std::next(remaining.begin());
        while (std::next(fan) != remaining.end()) {
          triangles.insert(triangles.end(), {first, *fan, *std::next(fan)});
          ++fan;
        }
        return triangles;
      }
    }
  }
  if (remaining.size() == 3) {
    auto last = remaining.begin();
    const int a = *last++;
    const int b = *last++;
    const int c = *last;
    if (std::abs(cross(a, b, c)) >= 1e-12) {
      triangles.insert(triangles.end(), {a, b, c});
    }
  }
  return triangles;
}
}  // namespace tile_map
