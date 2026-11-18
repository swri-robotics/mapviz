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

#include <tile_map/terrain.hpp>

#include <QRegularExpression>

#include <algorithm>
#include <cmath>

#include "swri_math_util/constants.h"
#include "swri_transform_util/earth_constants.h"

namespace tile_map
{
namespace terrarium
{
const char URL_TEMPLATE[] =
  "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{level}/{x}/{y}.png";
const char ATTRIBUTION[] = "Elevation: Terrain Tiles (Mapzen, AWS Open Data)";

std::vector<float> DecodeImage(const QImage & image)
{
  std::vector<float> heights;
  if (image.isNull()) {
    return heights;
  }

  const QImage rgb = image.convertToFormat(QImage::Format_RGB32);
  heights.reserve(static_cast<size_t>(rgb.width()) * rgb.height());
  for (int row = 0; row < rgb.height(); ++row) {
    const QRgb * line = reinterpret_cast<const QRgb *>(rgb.constScanLine(row));
    for (int col = 0; col < rgb.width(); ++col) {
      heights.push_back(DecodePixel(line[col]));
    }
  }
  return heights;
}

bool ParseUrl(const QString & url, int32_t & level, int64_t & x, int64_t & y)
{
  static const QRegularExpression tile_path("/terrarium/(\\d+)/(\\d+)/(\\d+)\\.png$");
  const QRegularExpressionMatch match = tile_path.match(url);
  if (!match.hasMatch()) {
    return false;
  }
  level = match.captured(1).toInt();
  x = match.captured(2).toLongLong();
  y = match.captured(3).toLongLong();
  return true;
}
}  // namespace terrarium

double MetersPerPixel(int32_t level, double latitude)
{
  return swri_transform_util::_earth_equator_circumference *
         std::cos(latitude * swri_math_util::_deg_2_rad) / std::pow(2.0, level + 8);
}

void LatLonToMercator(double latitude, double longitude, double & x, double & y)
{
  const double lat =
    std::clamp(latitude, -85.0511, 85.0511) * static_cast<double>(swri_math_util::_deg_2_rad);
  x = (longitude + 180.0) / 360.0;
  y = (1.0 - std::asinh(std::tan(lat)) / static_cast<double>(swri_math_util::_pi)) / 2.0;
}

double TileRowLatitude(int32_t level, double y)
{
  const double r = swri_math_util::_pi - swri_math_util::_2pi * y / std::pow(2.0, level);
  return swri_math_util::_rad_2_deg * std::atan(std::sinh(r));
}

std::vector<std::pair<int64_t, int64_t>> TilesAround(
  int32_t level, double latitude, double longitude, double radius, size_t max_tiles)
{
  std::vector<std::pair<int64_t, int64_t>> tiles;
  const double meters_per_degree = swri_transform_util::_earth_equator_circumference / 360.0;
  const double cos_lat = std::max(
    0.01, static_cast<double>(std::cos(latitude * swri_math_util::_deg_2_rad)));
  const double dlat = radius / meters_per_degree;
  const double dlon = radius / (meters_per_degree * cos_lat);

  const double n = std::pow(2.0, level);
  const int64_t max_index = static_cast<int64_t>(n) - 1;
  auto to_x = [n, max_index](double lon) {
      return std::clamp(
        static_cast<int64_t>(std::floor((lon + 180.0) / 360.0 * n)), int64_t{0}, max_index);
    };
  auto to_y = [n, max_index](double lat) {
      lat = std::clamp(lat, -85.0511, 85.0511) * static_cast<double>(swri_math_util::_deg_2_rad);
      const double pi = static_cast<double>(swri_math_util::_pi);
      const double y = (1.0 - std::asinh(std::tan(lat)) / pi) / 2.0 * n;
      return std::clamp(static_cast<int64_t>(std::floor(y)), int64_t{0}, max_index);
    };
  const int64_t x0 = to_x(longitude - dlon);
  const int64_t x1 = to_x(longitude + dlon);
  const int64_t y0 = to_y(latitude + dlat);
  const int64_t y1 = to_y(latitude - dlat);
  if (static_cast<size_t>((x1 - x0 + 1) * (y1 - y0 + 1)) > max_tiles) {
    return tiles;
  }
  for (int64_t y = y0; y <= y1; ++y) {
    for (int64_t x = x0; x <= x1; ++x) {
      tiles.emplace_back(x, y);
    }
  }
  return tiles;
}

bool ShadingParameters::operator==(const ShadingParameters & other) const
{
  return strength == other.strength && exaggeration == other.exaggeration;
}

QImage SlopeShade(
  const std::vector<float> & heights,
  int width,
  int height,
  double cell_size,
  const ShadingParameters & parameters)
{
  if (width <= 0 || height <= 0 || cell_size <= 0.0 ||
    heights.size() != static_cast<size_t>(width) * height)
  {
    return QImage();
  }

  const double strength = std::clamp(parameters.strength, 0.0, 1.0);
  const double scale = parameters.exaggeration / (8.0 * cell_size);

  // Neighbours past the edge reuse the edge cell.
  auto at = [&](int row, int col) {
      row = std::clamp(row, 0, height - 1);
      col = std::clamp(col, 0, width - 1);
      return static_cast<double>(heights[static_cast<size_t>(row) * width + col]);
    };

  QImage overlay(width, height, QImage::Format_ARGB32);
  for (int row = 0; row < height; ++row) {
    QRgb * line = reinterpret_cast<QRgb *>(overlay.scanLine(row));
    for (int col = 0; col < width; ++col) {
      // Horn's method: the gradient from the eight surrounding cells.
      const double a = at(row - 1, col - 1);
      const double b = at(row - 1, col);
      const double c = at(row - 1, col + 1);
      const double d = at(row, col - 1);
      const double f = at(row, col + 1);
      const double g = at(row + 1, col - 1);
      const double h = at(row + 1, col);
      const double i = at(row + 1, col + 1);
      const double dzdx = ((c + 2.0 * f + i) - (a + 2.0 * d + g)) * scale;
      const double dzdy = ((g + 2.0 * h + i) - (a + 2.0 * b + c)) * scale;

      // sin(atan(g)) = g / sqrt(1 + g^2): 0 on flat ground, 1 on a cliff.
      const double gradient = std::hypot(dzdx, dzdy);
      const double steepness = gradient / std::sqrt(1.0 + gradient * gradient);
      const int alpha = static_cast<int>(std::lround(255.0 * strength * steepness));
      line[col] = qRgba(0, 0, 0, std::clamp(alpha, 0, 255));
    }
  }
  return overlay;
}
}  // namespace tile_map
