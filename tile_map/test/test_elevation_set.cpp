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

#include <QDir>
#include <QFile>
#include <QImage>
#include <QRgb>
#include <QString>
#include <QTemporaryDir>
#include <QTextStream>

#include <cmath>
#include <memory>
#include <vector>

#include <tile_map/contour_layer.hpp>
#include <tile_map/elevation_set.hpp>
#include <tile_map/terrain.hpp>

using tile_map::ContourLayer;
using tile_map::ElevationLibrary;
using tile_map::ElevationSet;

namespace
{
/// A Terrarium pixel for a height, or a transparent one for NaN.
QRgb Encode(double height)
{
  if (std::isnan(height)) {
    return qRgba(0, 0, 0, 0);
  }
  const auto steps = static_cast<int64_t>(std::lround((height + 32768.0) * 256.0));
  return qRgba(
    static_cast<int>(steps >> 16), static_cast<int>((steps >> 8) & 255),
    static_cast<int>(steps & 255), 255);
}

void WriteText(const QString & path, const QString & text)
{
  QFile file(path);
  ASSERT_TRUE(file.open(QIODevice::WriteOnly | QIODevice::Text));
  QTextStream(&file) << text;
}

/// Writes a wgs84 set 512 x 300 pixels with 256 pixel tiles, 0.0001 degrees
/// per pixel from (29.47 N, 98.63 W), whose heights rise 1 m per column.
/// Only layer 0's first tile is written; its last row has no data.
QString WriteWgs84Set(const QString & folder, const QString & name, double degrees = 0.0001)
{
  QDir(folder).mkpath(name + "/layer0");
  QImage tile(256, 256, QImage::Format_ARGB32);
  for (int row = 0; row < 256; ++row) {
    for (int col = 0; col < 256; ++col) {
      tile.setPixel(col, row, Encode(row == 255 ? NAN : 100.0 + col));
    }
  }
  tile.save(folder + "/" + name + "/layer0/tile00000x00000.png");
  const double west = -98.63;
  const double north = 29.47;
  WriteText(
    folder + "/" + name + ".geo",
    QString(
      "image_path: %1\nimage_width: 512\nimage_height: 300\ntile_size: 256\n"
      "extension: png\nencoding: terrarium\ndatum: wgs84\nprojection: wgs84\n"
      "attribution: \"Test data\"\ntiepoints:\n"
      "  - point: [0, 0, %2, %3]\n  - point: [512, 0, %4, %3]\n"
      "  - point: [0, 300, %2, %5]\n  - point: [512, 300, %4, %5]\n")
    .arg(name)
    .arg(west, 0, 'f', 8).arg(north, 0, 'f', 8)
    .arg(west + 512 * degrees, 0, 'f', 8).arg(north - 300 * degrees, 0, 'f', 8));
  return folder + "/" + name + ".geo";
}
}  // namespace

TEST(ElevationSet, LoadsAWgs84Set)
{
  QTemporaryDir folder;
  QString error;
  const auto set = ElevationSet::Load(WriteWgs84Set(folder.path(), "site"), &error);
  ASSERT_NE(nullptr, set) << error.toStdString();
  EXPECT_EQ(512, set->Width());
  EXPECT_EQ(300, set->Height());
  EXPECT_EQ(256, set->TileSize());
  // 512 / 256 = 2, so two layers, as multires_image counts them.
  EXPECT_EQ(2, set->LayerCount());
  EXPECT_EQ(2, set->Columns(0));
  EXPECT_EQ(2, set->Rows(0));
  EXPECT_EQ(1, set->Columns(1));
  EXPECT_EQ(1, set->Rows(1));
  EXPECT_EQ("Test data", set->Attribution().toStdString());

  double latitude;
  double longitude;
  set->PixelToLatLon(0.0, 0.0, latitude, longitude);
  EXPECT_NEAR(29.47, latitude, 1e-9);
  EXPECT_NEAR(-98.63, longitude, 1e-9);
  set->PixelToLatLon(512.0, 300.0, latitude, longitude);
  EXPECT_NEAR(29.44, latitude, 1e-9);
  EXPECT_NEAR(-98.5788, longitude, 1e-9);

  double x;
  double y;
  set->LatLonToPixel(29.455, -98.60, x, y);
  EXPECT_NEAR(300.0, x, 1e-6);
  EXPECT_NEAR(150.0, y, 1e-6);
  EXPECT_TRUE(set->Contains(29.455, -98.60));
  EXPECT_FALSE(set->Contains(29.50, -98.60));
  EXPECT_FALSE(set->Contains(29.455, -98.70));

  // 0.0001 degrees is about 11.1 m north-south and 9.7 m east-west here.
  EXPECT_NEAR(11.13, set->PixelHeightMeters(), 0.05);
  EXPECT_NEAR(9.69, set->PixelWidthMeters(), 0.05);
}

TEST(ElevationSet, ReadsTilesWithGapsAndMissingFiles)
{
  QTemporaryDir folder;
  const auto set = ElevationSet::Load(WriteWgs84Set(folder.path(), "site"));
  ASSERT_NE(nullptr, set);
  const auto tile = set->ReadTile(0, 0, 0);
  ASSERT_NE(nullptr, tile);
  ASSERT_EQ(256u * 256u, tile->size());
  EXPECT_FLOAT_EQ(100.0f, (*tile)[0]);
  EXPECT_FLOAT_EQ(355.0f, (*tile)[255]);
  EXPECT_TRUE(std::isnan((*tile)[255 * 256]));
  // The same tile again comes from memory.
  EXPECT_EQ(tile, set->ReadTile(0, 0, 0));
  EXPECT_EQ(nullptr, set->ReadTile(0, 1, 1));
  EXPECT_EQ(
    folder.path().toStdString() + "/site/layer1/tile00002x00003.png",
    set->TilePath(1, 2, 3).toStdString());
}

TEST(ElevationSet, LoadsAUtmSet)
{
  QTemporaryDir folder;
  // 10 m pixels in zone 14 north, near 29.45 N, 98.61 W.
  WriteText(
    folder.path() + "/utm.geo",
    "image_path: utm\nimage_width: 400\nimage_height: 300\nencoding: terrarium\n"
    "datum: wgs84\nprojection: utm\nutm_zone: 14\nutm_band: R\ntiepoints:\n"
    "  - point: [0, 0, 536000.0, 3259000.0]\n"
    "  - point: [400, 0, 540000.0, 3259000.0]\n"
    "  - point: [0, 300, 536000.0, 3256000.0]\n");
  QString error;
  const auto set = ElevationSet::Load(folder.path() + "/utm.geo", &error);
  ASSERT_NE(nullptr, set) << error.toStdString();
  EXPECT_NEAR(10.0, set->PixelWidthMeters(), 0.1);
  EXPECT_NEAR(10.0, set->PixelHeightMeters(), 0.1);

  double latitude;
  double longitude;
  set->PixelToLatLon(200.0, 150.0, latitude, longitude);
  EXPECT_NEAR(29.45, latitude, 0.02);
  EXPECT_NEAR(-98.61, longitude, 0.02);
  double x;
  double y;
  set->LatLonToPixel(latitude, longitude, x, y);
  EXPECT_NEAR(200.0, x, 1e-3);
  EXPECT_NEAR(150.0, y, 1e-3);
}

TEST(ElevationSet, ImageryIsNotAnElevationSet)
{
  QTemporaryDir folder;
  WriteText(
    folder.path() + "/imagery.geo",
    "image_path: tiles\nimage_width: 100\nimage_height: 100\ntile_size: 512\n"
    "datum: wgs84\nprojection: wgs84\ntiepoints:\n  - point: [0, 0, 1.0, 2.0]\n");
  QString error = "unchanged";
  EXPECT_EQ(nullptr, ElevationSet::Load(folder.path() + "/imagery.geo", &error));
  EXPECT_TRUE(error.isEmpty());
}

TEST(ElevationSet, ReportsProblems)
{
  QTemporaryDir folder;
  auto load = [&folder](const QString & text) {
      WriteText(folder.path() + "/bad.geo", text);
      QString error;
      EXPECT_EQ(nullptr, ElevationSet::Load(folder.path() + "/bad.geo", &error));
      return error.toStdString();
    };
  const QString base =
    "image_path: x\nimage_width: 100\nimage_height: 100\ntiepoints:\n"
    "  - point: [0, 0, 1.0, 2.0]\n  - point: [100, 0, 2.0, 2.0]\n";
  EXPECT_NE("", load("encoding: rgb\n" + base + "  - point: [0, 100, 1.0, 1.0]\n"));
  EXPECT_NE("", load("encoding: terrarium\n" + base));
  EXPECT_NE("", load("encoding: terrarium\n" + base + "  - point: [50, 0, 1.5, 2.0]\n"));
  EXPECT_NE(
    "", load("encoding: terrarium\nprojection: utm\n" + base + "  - point: [0, 100, 1.0, 1.0]\n"));
  EXPECT_NE("", load("encoding: terrarium\n" + QString("image_path: [unclosed\n")));
}

TEST(ElevationLibrary, PicksTheFinestSetHere)
{
  QTemporaryDir folder;
  QDir(folder.path()).mkpath("fine");
  WriteWgs84Set(folder.path(), "coarse", 0.001);
  WriteWgs84Set(folder.path() + "/fine", "fine", 0.0001);
  WriteText(folder.path() + "/broken.geo", "encoding: terrarium\nimage_path: x\n");

  ElevationLibrary library;
  library.SetDirectory(folder.path());
  EXPECT_EQ(2u, library.Sets().size());
  ASSERT_EQ(1u, library.Problems().size());
  EXPECT_NE(std::string::npos, library.Problems()[0].toStdString().find("broken.geo"));

  const auto here = library.SetAt(29.455, -98.60);
  ASSERT_NE(nullptr, here);
  EXPECT_NE(std::string::npos, here->GeoPath().toStdString().find("fine.geo"));
  // Only the coarse set reaches this far.
  const auto there = library.SetAt(29.3, -98.4);
  ASSERT_NE(nullptr, there);
  EXPECT_NE(std::string::npos, there->GeoPath().toStdString().find("coarse.geo"));
  EXPECT_EQ(nullptr, library.SetAt(10.0, 10.0));

  library.SetDirectory("");
  EXPECT_TRUE(library.Sets().empty());
}

TEST(ContourLayer, LocalBlockPicksALayerForTheScale)
{
  QTemporaryDir folder;
  const std::shared_ptr<const ElevationSet> set =
    ElevationSet::Load(WriteWgs84Set(folder.path(), "site"));
  ASSERT_NE(nullptr, set);

  // Zoomed in closer than the set's 10 m pixels: layer 0, around the point.
  ContourLayer::Block block =
    ContourLayer::LocalBlockFor(set, 29.4685, -98.6285, 2.0, 400, 300);
  EXPECT_EQ(set, block.set);
  EXPECT_EQ(0, block.level);
  EXPECT_EQ(0, block.x);
  EXPECT_EQ(0, block.y);
  EXPECT_EQ(1, block.columns);
  EXPECT_EQ(1, block.rows);

  // At 15 m per pixel the 20 m pixels of layer 1 are the finest no smaller.
  block = ContourLayer::LocalBlockFor(set, 29.455, -98.60, 15.0, 400, 300);
  EXPECT_EQ(1, block.level);
  EXPECT_EQ(1, block.columns);

  // The middle of grid cell (0, 0) is the middle of the set's first pixel.
  block = ContourLayer::LocalBlockFor(set, 29.4685, -98.6285, 2.0, 400, 300);
  double latitude;
  double longitude;
  ContourLayer::GridToLatLon(block, 0.0, 0.0, latitude, longitude);
  EXPECT_NEAR(29.47 - 0.00005, latitude, 1e-9);
  EXPECT_NEAR(-98.63 + 0.00005, longitude, 1e-9);

  // A view nowhere near the set.
  EXPECT_EQ(0, ContourLayer::LocalBlockFor(set, 10.0, 10.0, 2.0, 400, 300).columns);
}

// The same places as test_make_elevation_set.py's credit test, so the two
// copies of the regional table stay in step.
TEST(Terrarium, CreditsTheDataSetsInView)
{
  const QString usgs = "Elevation: Mapzen Terrain Tiles; 3DEP, SRTM and GMTED2010 courtesy "
    "of the U.S. Geological Survey; ETOPO1 terrain data U.S. NOAA";
  // San Antonio.
  EXPECT_EQ(usgs, tile_map::terrarium::Attribution({15, 29.40, -98.70, 29.50, -98.55}));

  // Oslo, zoomed in and out.
  const QString oslo = tile_map::terrarium::Attribution({15, 59.85, 10.60, 59.95, 10.85});
  EXPECT_TRUE(oslo.startsWith("Elevation: Mapzen Terrain Tiles; SRTM and GMTED2010"));
  EXPECT_TRUE(oslo.contains("EU-DEM layers"));
  EXPECT_TRUE(oslo.contains("Norway terrain data \u00a9 Kartverket"));
  EXPECT_FALSE(oslo.contains("ArcticDEM"));
  EXPECT_FALSE(oslo.contains("Canada"));
  EXPECT_FALSE(
    tile_map::terrarium::Attribution({5, 59.85, 10.60, 59.95, 10.85}).contains("Kartverket"));

  // Along borders, the credit goes with the side of the border the view is on.
  auto credit = [](double south, double west, double north, double east) {
      return tile_map::terrarium::Attribution({12, south, west, north, east});
    };
  const QString toronto = credit(43.60, -79.50, 43.75, -79.30);
  EXPECT_TRUE(toronto.contains("Canada") && !toronto.contains("3DEP"));
  const QString seattle = credit(47.55, -122.40, 47.65, -122.25);
  EXPECT_TRUE(seattle.contains("3DEP") && !seattle.contains("Canada"));
  const QString munich = credit(48.10, 11.50, 48.20, 11.65);
  EXPECT_TRUE(munich.contains("EU-DEM") && !munich.contains("Austria"));
  const QString dublin = credit(53.30, -6.35, 53.40, -6.20);
  EXPECT_TRUE(dublin.contains("EU-DEM") && !dublin.contains("United Kingdom"));
  const QString monterrey = credit(25.60, -100.40, 25.75, -100.20);
  EXPECT_TRUE(monterrey.contains("INEGI") && !monterrey.contains("3DEP"));
  EXPECT_FALSE(credit(14.55, -90.60, 14.70, -90.45).contains("INEGI"));  // Guatemala City
  EXPECT_FALSE(credit(50.40, 30.40, 50.50, 30.60).contains("EU-DEM"));  // Kyiv

  // Vienna.
  const QString vienna = tile_map::terrarium::Attribution({12, 48.1, 16.2, 48.3, 16.5});
  EXPECT_TRUE(vienna.contains("Gel\u00e4ndemodell"));
  EXPECT_TRUE(vienna.contains("EU-DEM layers"));

  // The whole world, as before the view is known.
  EXPECT_EQ(
    "Elevation: Mapzen Terrain Tiles; SRTM and GMTED2010 courtesy of the U.S. Geological "
    "Survey; ETOPO1 terrain data U.S. NOAA",
    tile_map::terrarium::Attribution({}));
}

TEST(Terrarium, TransparentPixelsHaveNoHeight)
{
  QImage image(2, 1, QImage::Format_ARGB32);
  image.setPixel(0, 0, Encode(12.5));
  image.setPixel(1, 0, Encode(NAN));
  const std::vector<float> heights = tile_map::terrarium::DecodeImage(image);
  ASSERT_EQ(2u, heights.size());
  EXPECT_FLOAT_EQ(12.5f, heights[0]);
  EXPECT_TRUE(std::isnan(heights[1]));
}

TEST(SlopeShade, LeavesCellsWithNoDataClear)
{
  std::vector<float> heights(9, 100.0f);
  heights[4] = NAN;
  heights[0] = 150.0f;
  const QImage shade = tile_map::SlopeShade(heights, 3, 3, 1.0, tile_map::ShadingParameters());
  ASSERT_FALSE(shade.isNull());
  EXPECT_EQ(0, qAlpha(shade.pixel(1, 1)));
  EXPECT_GT(qAlpha(shade.pixel(0, 0)), 0);
  EXPECT_EQ(0, qAlpha(shade.pixel(2, 2)));
}
