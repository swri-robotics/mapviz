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

#ifndef TILE_MAP__TREES_HPP_
#define TILE_MAP__TREES_HPP_

#include <QImage>

#include <cstdint>
#include <vector>

#include <tile_map/buildings.hpp>

namespace tile_map
{
/// A tree crown to draw: where it is and how big.
struct Crown
{
  LatLon center;
  /// Radius in metres.
  double radius;
  /// Height of the top of the crown above the ground, in metres.
  double height;
};

/// A crown placed in an image, in pixel coordinates.
struct PixelCrown
{
  double x;
  double y;
  /// Radius and height in metres.
  double radius;
  double height;
};

/**
 * Which pixels of an aerial photo are tree canopy: dark green, or dark and
 * slightly green (canopy in its own shade).  Map-style imagery fails the
 * darkness test, so it yields no canopy.  Lone pixels are dropped and small
 * gaps in the canopy are filled.
 *
 * @return 1 for canopy and 0 for anything else, row by row.
 */
std::vector<uint8_t> CanopyMask(const QImage & image);

/**
 * Packs round crowns into a canopy mask, largest first: each crown is as big
 * as the canopy around its centre allows (up to a limit, so a forest becomes
 * many trees), and crowns overlap only a little.  Taller trees have wider
 * crowns, with some variation that depends only on where the tree is, so the
 * same canopy always gives the same trees.
 *
 * @param mask      As returned by CanopyMask()
 * @param cell_size Ground distance covered by one pixel, in metres
 * @param seed      Mixed into the height variation, e.g. the tile's position
 */
std::vector<PixelCrown> PlaceCrowns(
  const std::vector<uint8_t> & mask, int width, int height, double cell_size, uint64_t seed);

/**
 * Crowns spread through a mapped wood on a jittered grid, limited to a
 * latitude/longitude box so that a wood spanning several areas is filled
 * once per area rather than once for each.
 */
std::vector<Crown> CrownsInWoodland(
  const std::vector<LatLon> & outline,
  double south, double west, double north, double east,
  uint64_t seed);
}  // namespace tile_map

#endif  // TILE_MAP__TREES_HPP_
