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
#include <QRegularExpressionMatch>
#include <QStringList>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "swri_math_util/constants.h"
#include "swri_transform_util/earth_constants.h"

namespace tile_map
{
namespace
{
/// A box in degrees around an area a regional data set covers.
struct Area
{
  double south;
  double west;
  double north;
  double east;
};

bool Overlaps(const ViewBounds & view, const std::vector<Area> & areas)
{
  for (const Area & area : areas) {
    if (view.south <= area.north && view.north >= area.south &&
      view.west <= area.east && view.east >= area.west)
    {
      return true;
    }
  }
  return false;
}

struct RegionalSource
{
  std::vector<Area> areas;
  const char * credit;
};

// Boxes around the areas the Terrain Tiles' regional data sets cover, from
// tilezen/joerd docs/data-sources.md, and the credit each asks for, from
// docs/attribution.md.  A box may reach a little past its area along a
// border, so a credit may show near its area but never goes missing inside
// it.  make_elevation_set has the same table; keep the two in step.
const std::vector<Area> & UnitedStates()
{
  static const std::vector<Area> areas = {
    {31.3, -125.0, 49.0, -104.4}, {29.5, -106.6, 31.3, -104.4}, {28.9, -104.4, 49.0, -95.2},
    {25.8, -100.6, 29.0, -97.0}, {24.3, -97.0, 31.0, -79.9}, {31.0, -95.2, 49.4, -89.5},
    {31.0, -89.5, 47.5, -84.8}, {31.0, -84.8, 45.9, -82.4}, {31.0, -82.4, 42.3, -79.8},
    {31.0, -79.8, 43.4, -76.3}, {31.0, -76.3, 45.1, -73.3}, {31.0, -73.3, 45.4, -69.0},
    {43.0, -71.1, 47.5, -66.9}, {51.0, -180.0, 71.6, -141.0}, {54.6, -141.0, 60.4, -129.9},
    {51.0, 172.0, 53.1, 180.0}, {18.8, -160.4, 22.4, -154.6}, {17.6, -67.4, 18.6, -64.4},
    {13.1, 144.5, 20.7, 146.2}, {-14.7, -171.2, -11.0, -168.0}
  };
  return areas;
}

const std::vector<RegionalSource> & RegionalSources()
{
  static const std::vector<RegionalSource> sources = {
    {{{60.0, -180.0, 90.0, 180.0}},
      "ArcticDEM terrain data DEM(s) were created from DigitalGlobe, Inc., imagery and funded "
      "under National Science Foundation awards 1043681, 1559691, and 1542736"},
    {{{-39.3, 129.0, -10.8, 150.1}},
      "Australia terrain data \u00a9 Commonwealth of Australia (Geoscience Australia) 2017"},
    {{{46.3, 9.5, 47.8, 13.0}, {46.3, 12.9, 49.1, 17.2}},
      "Austria terrain data \u00a9 offene Daten \u00d6sterreichs \u2013 Digitales "
      "Gel\u00e4ndemodell (DGM) \u00d6sterreich"},
    {{{48.3, -141.1, 83.2, -95.1}, {48.0, -95.1, 83.2, -52.5}, {46.2, -90.0, 48.0, -84.0},
      {45.3, -84.0, 46.2, -81.5}, {41.6, -83.5, 43.4, -78.9}, {43.2, -82.6, 46.2, -74.3},
      {45.0, -74.3, 48.0, -52.5}, {43.3, -67.0, 45.0, -59.5}},
      "Canada terrain data contains information licensed under the Open Government Licence "
      "\u2013 Canada"},
    {{{36.0, -25.0, 71.5, 24.2}, {60.5, 24.2, 70.1, 31.6}, {59.7, 24.2, 60.5, 28.0},
      {53.9, 24.2, 59.7, 28.3}, {34.8, 19.0, 42.0, 28.3}, {42.0, 22.0, 48.3, 29.8},
      {34.5, 32.2, 35.8, 34.7}, {35.7, 14.1, 36.1, 14.6}},
      "Europe terrain data produced using Copernicus data and information funded by the "
      "European Union - EU-DEM layers"},
    {{{22.0, -118.5, 32.75, -114.7}, {22.0, -114.8, 32.5, -111.0}, {14.5, -111.0, 31.4, -108.2},
      {14.5, -108.2, 31.8, -106.4}, {14.5, -106.4, 30.9, -104.4}, {14.5, -104.4, 29.9, -102.3},
      {14.5, -102.3, 29.9, -100.6}, {14.5, -100.6, 28.7, -99.5}, {14.5, -99.5, 27.7, -98.5},
      {14.5, -98.5, 26.4, -97.1}, {17.8, -97.1, 25.9, -86.7}, {14.5, -97.1, 17.8, -92.2},
      {16.0, -92.2, 17.8, -90.4}},
      "Mexico terrain data source: INEGI, Continental relief, 2016"},
    {{{-47.4, 166.3, -34.3, 178.7}},
      "New Zealand terrain data Copyright 2011 Crown copyright (c) Land Information New Zealand"
      " and the New Zealand Government (All rights reserved)"},
    {{{57.9, 4.4, 63.0, 12.9}, {63.0, 7.0, 66.0, 14.6}, {66.0, 11.0, 68.6, 18.2},
      {68.6, 15.0, 71.3, 31.2}},
      "Norway terrain data \u00a9 Kartverket"},
    {{{49.8, -5.8, 55.9, 1.8}, {54.0, -8.2, 55.4, -5.4}, {54.6, -7.7, 60.9, -0.7}},
      "United Kingdom terrain data \u00a9 Environment Agency copyright and/or database right "
      "2015. All rights reserved"},
  };
  return sources;
}

// The regional data sets are used from level 7 (Canada) or 9 and 10 (the
// rest); coarser levels use only the global ones.
constexpr int32_t REGIONAL_LEVEL = 7;
}  // namespace

namespace terrarium
{
const char URL_TEMPLATE[] =
  "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{level}/{x}/{y}.png";

QString Attribution(const ViewBounds & view)
{
  const bool regional = view.level >= REGIONAL_LEVEL;
  const bool united_states = regional && Overlaps(view, UnitedStates());
  QStringList credits;
  credits << "Elevation: Mapzen Terrain Tiles";
  credits << QString(united_states ? "3DEP, SRTM" : "SRTM") +
    " and GMTED2010 courtesy of the U.S. Geological Survey";
  credits << "ETOPO1 terrain data U.S. NOAA";
  for (const RegionalSource & source : RegionalSources()) {
    if (regional && Overlaps(view, source.areas)) {
      credits << QString::fromUtf8(source.credit);
    }
  }
  return credits.join("; ");
}

std::vector<float> DecodeImage(const QImage & image)
{
  std::vector<float> heights;
  if (image.isNull()) {
    return heights;
  }

  // Elevation sets mark cells with no data by leaving them transparent; the
  // online tiles are opaque.
  const QImage argb = image.convertToFormat(QImage::Format_ARGB32);
  heights.reserve(static_cast<size_t>(argb.width()) * argb.height());
  for (int row = 0; row < argb.height(); ++row) {
    const QRgb * line = reinterpret_cast<const QRgb *>(argb.constScanLine(row));
    for (int col = 0; col < argb.width(); ++col) {
      heights.push_back(
        qAlpha(line[col]) == 0 ? std::numeric_limits<float>::quiet_NaN() :
        DecodePixel(line[col]));
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

  // Neighbors past the edge reuse the edge cell.
  auto at = [&](int row, int col) {
      row = std::clamp(row, 0, height - 1);
      col = std::clamp(col, 0, width - 1);
      return static_cast<double>(heights[static_cast<size_t>(row) * width + col]);
    };

  QImage overlay(width, height, QImage::Format_ARGB32);
  for (int row = 0; row < height; ++row) {
    QRgb * line = reinterpret_cast<QRgb *>(overlay.scanLine(row));
    for (int col = 0; col < width; ++col) {
      // Cells with no data are left clear, and count as level with their
      // neighbors when shading those.
      const double e = at(row, col);
      if (std::isnan(e)) {
        line[col] = qRgba(0, 0, 0, 0);
        continue;
      }
      auto near = [&](int r, int c) {
          const double value = at(r, c);
          return std::isnan(value) ? e : value;
        };
      // Horn's method: the gradient from the eight surrounding cells.
      const double a = near(row - 1, col - 1);
      const double b = near(row - 1, col);
      const double c = near(row - 1, col + 1);
      const double d = near(row, col - 1);
      const double f = near(row, col + 1);
      const double g = near(row + 1, col - 1);
      const double h = near(row + 1, col);
      const double i = near(row + 1, col + 1);
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
