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
#include <QRgb>
#include <QSize>

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <tile_map/map_25d.hpp>
#include <tile_map/terrain.hpp>

using tile_map::SlopeShade;
using tile_map::Map25d;
using tile_map::ShadingParameters;

namespace
{
/// Heights for a width x height grid that rises `rise` meters per cell going
/// east (and nothing going north or south).
std::vector<float> RisingEast(int width, int height, float rise)
{
  std::vector<float> heights;
  for (int row = 0; row < height; ++row) {
    for (int col = 0; col < width; ++col) {
      heights.push_back(100.0f + rise * col);
    }
  }
  return heights;
}

/// The same, rising `rise` meters per cell going north (row 0 is north).
std::vector<float> RisingNorth(int width, int height, float rise)
{
  std::vector<float> heights;
  for (int row = 0; row < height; ++row) {
    for (int col = 0; col < width; ++col) {
      heights.push_back(100.0f + rise * (height - 1 - row));
    }
  }
  return heights;
}

/// True if every pixel is fully transparent.
bool Transparent(const QImage & image)
{
  for (int y = 0; y < image.height(); ++y) {
    for (int x = 0; x < image.width(); ++x) {
      if (qAlpha(image.pixel(x, y)) != 0) {
        return false;
      }
    }
  }
  return true;
}

ShadingParameters FullStrength()
{
  ShadingParameters shading;
  shading.strength = 1.0;
  shading.exaggeration = 1.0;
  return shading;
}
}  // namespace

TEST(Terrarium, DecodesPixels)
{
  // R * 256 + G + B / 256 - 32768
  EXPECT_FLOAT_EQ(0.0f, tile_map::terrarium::DecodePixel(qRgb(128, 0, 0)));
  EXPECT_FLOAT_EQ(233.5f, tile_map::terrarium::DecodePixel(qRgb(128, 233, 128)));
  EXPECT_FLOAT_EQ(-10.0f, tile_map::terrarium::DecodePixel(qRgb(127, 246, 0)));
}

TEST(Terrarium, DecodesImagesRowByRowFromTheNorth)
{
  QImage image(2, 2, QImage::Format_RGB32);
  image.setPixel(0, 0, qRgb(128, 1, 0));
  image.setPixel(1, 0, qRgb(128, 2, 0));
  image.setPixel(0, 1, qRgb(128, 3, 0));
  image.setPixel(1, 1, qRgb(128, 4, 0));

  const std::vector<float> expected = {1.0f, 2.0f, 3.0f, 4.0f};
  EXPECT_EQ(expected, tile_map::terrarium::DecodeImage(image));
  EXPECT_TRUE(tile_map::terrarium::DecodeImage(QImage()).empty());
}

TEST(Terrarium, ParsesTileUrls)
{
  int32_t level = -1;
  int64_t x = -1;
  int64_t y = -1;
  ASSERT_TRUE(
    tile_map::terrarium::ParseUrl(
      "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/15/7404/13577.png",
      level, x, y));
  EXPECT_EQ(15, level);
  EXPECT_EQ(7404, x);
  EXPECT_EQ(13577, y);

  EXPECT_FALSE(
    tile_map::terrarium::ParseUrl("https://tile.openstreetmap.org/3/1/2.png", level, x, y));
  EXPECT_FALSE(
    tile_map::terrarium::ParseUrl(
      "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{level}/{x}/{y}.png",
      level, x, y));
}

TEST(Terrain, WebMercatorGeometry)
{
  // A 256 pixel tile at level 0 spans the equator's circumference.
  EXPECT_NEAR(156543.03, tile_map::MetersPerPixel(0, 0.0), 0.01);
  EXPECT_NEAR(156543.03 / 2.0, tile_map::MetersPerPixel(0, 60.0), 0.01);
  EXPECT_NEAR(156543.03 / 1024.0, tile_map::MetersPerPixel(10, 0.0), 0.001);

  EXPECT_NEAR(85.0511, tile_map::TileRowLatitude(0, 0.0), 1e-4);
  EXPECT_NEAR(0.0, tile_map::TileRowLatitude(1, 1.0), 1e-9);
  EXPECT_NEAR(-85.0511, tile_map::TileRowLatitude(0, 1.0), 1e-4);
}

TEST(SlopeShade, LeavesFlatGroundUntouched)
{
  const std::vector<float> flat(16 * 16, 230.0f);
  const QImage overlay = SlopeShade(flat, 16, 16, 5.0, FullStrength());
  ASSERT_EQ(QSize(16, 16), overlay.size());
  EXPECT_TRUE(Transparent(overlay));
}

TEST(SlopeShade, DarkensSlopesWhicheverWayTheyFace)
{
  // Lit evenly from every direction: only steepness matters.
  const QRgb east = SlopeShade(RisingEast(8, 8, 2.0f), 8, 8, 2.0, FullStrength()).pixel(4, 4);
  const QRgb north = SlopeShade(RisingNorth(8, 8, 2.0f), 8, 8, 2.0, FullStrength()).pixel(4, 4);
  EXPECT_EQ(qRgba(0, 0, 0, qAlpha(east)), east);
  EXPECT_GT(qAlpha(east), 0);
  EXPECT_EQ(east, north);

  // A 45 degree slope (rising 2 m per 2 m cell) is sin(45) dark.
  EXPECT_NEAR(255.0 * std::sqrt(0.5), qAlpha(east), 1.0);
}

TEST(SlopeShade, SteeperSlopesAndExaggerationShadeMore)
{
  const QImage gentle = SlopeShade(RisingEast(8, 8, 0.5f), 8, 8, 2.0, FullStrength());
  const QImage steep = SlopeShade(RisingEast(8, 8, 2.0f), 8, 8, 2.0, FullStrength());
  EXPECT_LT(qAlpha(gentle.pixel(4, 4)), qAlpha(steep.pixel(4, 4)));

  ShadingParameters exaggerated = FullStrength();
  exaggerated.exaggeration = 4.0;
  const QImage four_times = SlopeShade(RisingEast(8, 8, 0.5f), 8, 8, 2.0, exaggerated);
  EXPECT_EQ(qAlpha(steep.pixel(4, 4)), qAlpha(four_times.pixel(4, 4)));
}

TEST(SlopeShade, StrengthScalesTheDarkening)
{
  ShadingParameters off = FullStrength();
  off.strength = 0.0;
  EXPECT_TRUE(Transparent(SlopeShade(RisingEast(8, 8, 5.0f), 8, 8, 1.0, off)));

  ShadingParameters half = FullStrength();
  half.strength = 0.5;
  const int full_alpha =
    qAlpha(SlopeShade(RisingEast(8, 8, 2.0f), 8, 8, 2.0, FullStrength()).pixel(4, 4));
  const int half_alpha = qAlpha(SlopeShade(RisingEast(8, 8, 2.0f), 8, 8, 2.0, half).pixel(4, 4));
  EXPECT_NEAR(full_alpha / 2.0, half_alpha, 1.0);
}

TEST(SlopeShade, RejectsMismatchedInput)
{
  EXPECT_TRUE(SlopeShade({}, 0, 0, 1.0, FullStrength()).isNull());
  EXPECT_TRUE(SlopeShade(std::vector<float>(10, 0.0f), 4, 4, 1.0, FullStrength()).isNull());
  EXPECT_TRUE(SlopeShade(std::vector<float>(16, 0.0f), 4, 4, 0.0, FullStrength()).isNull());
}

TEST(Map25d, ShadesOnlyTerrariumTiles)
{
  // A flat tile, as Terrarium encodes 0 m.
  QImage tile(256, 256, QImage::Format_RGB32);
  tile.fill(qRgb(128, 0, 0));

  const QImage overlay = Map25d::ShadeTerrariumTile(
    tile, "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/15/7404/13577.png",
    ShadingParameters());
  ASSERT_EQ(QSize(256, 256), overlay.size());
  EXPECT_TRUE(Transparent(overlay));

  EXPECT_TRUE(
    Map25d::ShadeTerrariumTile(
      tile, "https://tile.openstreetmap.org/15/7404/13577.png", ShadingParameters()).isNull());
}
