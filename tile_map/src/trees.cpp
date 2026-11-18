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

#include <tile_map/trees.hpp>

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <utility>

#include "swri_math_util/constants.h"

namespace tile_map
{
namespace
{
/// Crowns smaller than this are not placed.
constexpr double MIN_CROWN_RADIUS = 1.2;
/// Wider canopy is split into several crowns of at most this radius.
constexpr double MAX_CROWN_RADIUS = 6.0;
/// How much neighbouring crowns may overlap, as a fraction of their radii.
constexpr double CROWN_OVERLAP = 0.25;
/// Spacing of the trees spread through a mapped wood, in metres.
constexpr double WOODLAND_SPACING = 6.0;
/// The most trees spread through one wood in one area.
constexpr size_t MAX_WOODLAND_CROWNS = 40000;
constexpr double METERS_PER_DEGREE = 111320.0;

/// A well mixed 64 bit hash (splitmix64), for repeatable variation.
uint64_t Mix(uint64_t value)
{
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31);
}

/// A repeatable number in [0, 1) for a position and a seed.
double Unit(uint64_t seed, int64_t a, int64_t b, uint64_t salt)
{
  const uint64_t h = Mix(seed ^ Mix(static_cast<uint64_t>(a) * 0x100000001b3ULL ^
    Mix(static_cast<uint64_t>(b) + salt)));
  return static_cast<double>(h >> 11) / static_cast<double>(1ULL << 53);
}

/// A tree's height from its crown radius: wider crowns on taller trees.
double HeightForRadius(double radius, double variation)
{
  return std::clamp((2.5 + 1.8 * radius) * (0.85 + 0.3 * variation), 3.0, 25.0);
}

bool InsidePolygon(const std::vector<std::pair<double, double>> & polygon, double x, double y)
{
  bool inside = false;
  for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const auto & a = polygon[i];
    const auto & b = polygon[j];
    if ((a.second > y) != (b.second > y) &&
      x < (b.first - a.first) * (y - a.second) / (b.second - a.second) + a.first)
    {
      inside = !inside;
    }
  }
  return inside;
}
}  // namespace

std::vector<uint8_t> CanopyMask(const QImage & image)
{
  const int width = image.width();
  const int height = image.height();
  std::vector<uint8_t> mask(static_cast<size_t>(width) * height, 0);
  if (image.isNull()) {
    return mask;
  }

  const QImage rgb = image.convertToFormat(QImage::Format_RGB32);
  for (int row = 0; row < height; ++row) {
    const QRgb * line = reinterpret_cast<const QRgb *>(rgb.constScanLine(row));
    for (int col = 0; col < width; ++col) {
      const int r = qRed(line[col]);
      const int g = qGreen(line[col]);
      const int b = qBlue(line[col]);
      const int excess_green = 2 * g - r - b;
      const int luminance = (r + g + b) / 3;
      // Lit canopy is dark green; canopy in its own shade is darker still
      // and only slightly green.  Lawns and fields are too bright.
      mask[static_cast<size_t>(row) * width + col] =
        ((excess_green > 18 && luminance < 112) || (luminance < 72 && excess_green > 4)) ? 1 : 0;
    }
  }

  auto count_neighbours = [&](const std::vector<uint8_t> & source, int row, int col) {
      int count = 0;
      for (int dr = -1; dr <= 1; ++dr) {
        for (int dc = -1; dc <= 1; ++dc) {
          const int r = row + dr;
          const int c = col + dc;
          if ((dr != 0 || dc != 0) && r >= 0 && r < height && c >= 0 && c < width) {
            count += source[static_cast<size_t>(r) * width + c];
          }
        }
      }
      return count;
    };

  // Drop specks with almost no canopy around them, and fill pixels that are
  // mostly surrounded by canopy.
  const std::vector<uint8_t> raw = mask;
  for (int row = 0; row < height; ++row) {
    for (int col = 0; col < width; ++col) {
      const int neighbours = count_neighbours(raw, row, col);
      uint8_t & cell = mask[static_cast<size_t>(row) * width + col];
      if (cell && neighbours < 2) {
        cell = 0;
      } else if (!cell && neighbours >= 6) {
        cell = 1;
      }
    }
  }
  return mask;
}

std::vector<PixelCrown> PlaceCrowns(
  const std::vector<uint8_t> & mask, int width, int height, double cell_size, uint64_t seed)
{
  std::vector<PixelCrown> crowns;
  if (width <= 0 || height <= 0 || cell_size <= 0.0 ||
    mask.size() != static_cast<size_t>(width) * height)
  {
    return crowns;
  }

  // Distance from each canopy pixel to the nearest pixel that isn't canopy
  // (or the edge of the image), by a two pass 3-4 chamfer transform.
  const int far = 1 << 28;
  std::vector<int> distance(mask.size());
  for (size_t i = 0; i < mask.size(); ++i) {
    distance[i] = mask[i] ? far : 0;
  }
  auto at = [&](int row, int col) -> int {
      if (row < 0 || row >= height || col < 0 || col >= width) {
        return 0;
      }
      return distance[static_cast<size_t>(row) * width + col];
    };
  for (int row = 0; row < height; ++row) {
    for (int col = 0; col < width; ++col) {
      int & d = distance[static_cast<size_t>(row) * width + col];
      if (d == 0) {
        continue;
      }
      d = std::min({d, at(row, col - 1) + 3, at(row - 1, col) + 3,
            at(row - 1, col - 1) + 4, at(row - 1, col + 1) + 4});
    }
  }
  for (int row = height - 1; row >= 0; --row) {
    for (int col = width - 1; col >= 0; --col) {
      int & d = distance[static_cast<size_t>(row) * width + col];
      if (d == 0) {
        continue;
      }
      d = std::min({d, at(row, col + 1) + 3, at(row + 1, col) + 3,
            at(row + 1, col + 1) + 4, at(row + 1, col - 1) + 4});
    }
  }

  // Candidate centres, widest first.
  std::vector<std::pair<double, int>> candidates;
  for (size_t i = 0; i < distance.size(); ++i) {
    const double radius = distance[i] / 3.0 * cell_size;
    if (radius >= MIN_CROWN_RADIUS) {
      candidates.emplace_back(radius, static_cast<int>(i));
    }
  }
  std::stable_sort(
    candidates.begin(), candidates.end(),
    [](const auto & a, const auto & b) {return a.first > b.first;});

  // Accept a candidate when it doesn't crowd the crowns already placed,
  // checking only nearby ones through a grid of buckets.
  const double bucket = 2.0 * MAX_CROWN_RADIUS / cell_size;
  std::unordered_map<int64_t, std::vector<size_t>> buckets;
  auto bucket_key = [bucket](double x, double y) {
      return (static_cast<int64_t>(std::floor(x / bucket)) << 32) ^
             static_cast<int64_t>(std::floor(y / bucket) + 0x7fffffff);
    };
  for (const auto & candidate : candidates) {
    const double radius = std::min(candidate.first, MAX_CROWN_RADIUS);
    const double x = (candidate.second % width) + 0.5;
    const double y = (candidate.second / width) + 0.5;
    bool crowded = false;
    for (int dx = -1; dx <= 1 && !crowded; ++dx) {
      for (int dy = -1; dy <= 1 && !crowded; ++dy) {
        auto found = buckets.find(bucket_key(x + dx * bucket, y + dy * bucket));
        if (found == buckets.end()) {
          continue;
        }
        for (size_t index : found->second) {
          const PixelCrown & other = crowns[index];
          const double gap = std::hypot(other.x - x, other.y - y) * cell_size;
          if (gap < (1.0 - CROWN_OVERLAP) * (other.radius + radius)) {
            crowded = true;
            break;
          }
        }
      }
    }
    if (crowded) {
      continue;
    }
    const double variation = Unit(seed, static_cast<int64_t>(x), static_cast<int64_t>(y), 1);
    buckets[bucket_key(x, y)].push_back(crowns.size());
    crowns.push_back({x, y, radius, HeightForRadius(radius, variation)});
  }
  return crowns;
}

std::vector<Crown> CrownsInWoodland(
  const std::vector<LatLon> & outline,
  double south, double west, double north, double east,
  uint64_t seed)
{
  std::vector<Crown> crowns;
  if (outline.size() < 3 || south >= north || west >= east) {
    return crowns;
  }

  // Work in metres around the box's south-west corner.
  const double cos_lat =
    std::cos(0.5 * (south + north) * static_cast<double>(swri_math_util::_deg_2_rad));
  const double meters_per_lon = METERS_PER_DEGREE * cos_lat;
  std::vector<std::pair<double, double>> polygon;
  double min_x = 1e300;
  double max_x = -1e300;
  double min_y = 1e300;
  double max_y = -1e300;
  for (const LatLon & point : outline) {
    const double px = (point.longitude - west) * meters_per_lon;
    const double py = (point.latitude - south) * METERS_PER_DEGREE;
    polygon.emplace_back(px, py);
    min_x = std::min(min_x, px);
    max_x = std::max(max_x, px);
    min_y = std::min(min_y, py);
    max_y = std::max(max_y, py);
  }
  // Only the part of the wood inside the box.
  min_x = std::max(min_x, 0.0);
  min_y = std::max(min_y, 0.0);
  max_x = std::min(max_x, (east - west) * meters_per_lon);
  max_y = std::min(max_y, (north - south) * METERS_PER_DEGREE);
  if (min_x >= max_x || min_y >= max_y) {
    return crowns;
  }

  // Grid cells are numbered from the box corner, so neighbouring areas line
  // up and the same wood always gets the same trees.
  const int64_t first_col = static_cast<int64_t>(std::floor(min_x / WOODLAND_SPACING));
  const int64_t last_col = static_cast<int64_t>(std::floor(max_x / WOODLAND_SPACING));
  const int64_t first_row = static_cast<int64_t>(std::floor(min_y / WOODLAND_SPACING));
  const int64_t last_row = static_cast<int64_t>(std::floor(max_y / WOODLAND_SPACING));
  for (int64_t row = first_row; row <= last_row; ++row) {
    for (int64_t col = first_col; col <= last_col; ++col) {
      const double x = (col + 0.15 + 0.7 * Unit(seed, col, row, 2)) * WOODLAND_SPACING;
      const double y = (row + 0.15 + 0.7 * Unit(seed, col, row, 3)) * WOODLAND_SPACING;
      if (x < min_x || x > max_x || y < min_y || y > max_y || !InsidePolygon(polygon, x, y)) {
        continue;
      }
      const double radius = 2.2 + 1.6 * Unit(seed, col, row, 4);
      crowns.push_back(
        {{south + y / METERS_PER_DEGREE, west + x / meters_per_lon},
          radius, HeightForRadius(radius, Unit(seed, col, row, 5))});
      if (crowns.size() >= MAX_WOODLAND_CROWNS) {
        return crowns;
      }
    }
  }
  return crowns;
}
}  // namespace tile_map
