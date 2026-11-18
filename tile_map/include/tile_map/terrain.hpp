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

#ifndef TILE_MAP__TERRAIN_HPP_
#define TILE_MAP__TERRAIN_HPP_

#include <QImage>
#include <QRgb>
#include <QString>

#include <cstdint>
#include <vector>

namespace tile_map
{
/// Where the 2.5D plugin gets elevation from: the AWS Terrain Tiles open data
/// set, in its "Terrarium" PNG encoding.  No API key is needed.
namespace terrarium
{
/// The tile URL, with the same placeholders a WMTS source uses.
extern const char URL_TEMPLATE[];
/// The deepest level the data set has tiles for.
constexpr int32_t MAX_ZOOM = 15;
/// Credit line for the data set.
extern const char ATTRIBUTION[];

/// Height in metres encoded in one Terrarium pixel.
inline float DecodePixel(QRgb pixel)
{
  return static_cast<float>(qRed(pixel)) * 256.0f + static_cast<float>(qGreen(pixel)) +
         static_cast<float>(qBlue(pixel)) / 256.0f - 32768.0f;
}

/// Heights in metres for a whole tile, row by row from the north edge.
std::vector<float> DecodeImage(const QImage & image);

/// Reads the level and tile coordinates back out of a Terrarium tile URL.
bool ParseUrl(const QString & url, int32_t & level, int64_t & x, int64_t & y);
}  // namespace terrarium

/// Ground distance covered by one pixel of a 256 pixel web mercator tile.
double MetersPerPixel(int32_t level, double latitude);

/// Latitude of the north edge of tile row y (fractional rows are allowed).
double TileRowLatitude(int32_t level, double y);

struct ShadingParameters
{
  /// How dark the steepest slopes get, from 0 (no shading) to 1 (black).
  double strength = 0.6;
  /// Multiplies heights, so gentle terrain still shows some relief.
  double exaggeration = 2.0;

  bool operator==(const ShadingParameters & other) const;
  bool operator!=(const ShadingParameters & other) const {return !(*this == other);}
};

/**
 * A slope shading overlay for a grid of heights.
 *
 * The scene is lit evenly from every direction, so there is no light source
 * and nothing casts a shadow: each cell is darkened in proportion to how steep
 * it is (the sine of its slope, from Horn's method), whichever way it faces.
 * Flat ground is left fully transparent, so the overlay can be blended over
 * imagery without changing its colors where the ground is level.
 *
 * @param heights      Heights in metres, row by row, row 0 at the north edge
 * @param width        Number of columns
 * @param height       Number of rows
 * @param cell_size    Ground distance between neighbouring cells, in metres
 * @param parameters   Strength and height exaggeration
 * @return An ARGB32 image of width x height, or a null image if the input is
 *   empty or the wrong size
 */
QImage SlopeShade(
  const std::vector<float> & heights,
  int width,
  int height,
  double cell_size,
  const ShadingParameters & parameters);
}  // namespace tile_map

#endif  // TILE_MAP__TERRAIN_HPP_
