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
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QImage>
#include <QString>
#include <QTemporaryDir>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <multires_image/tile_set.hpp>
#include <multires_image/tile_set_layer.hpp>

namespace
{
constexpr int kTileSize = 256;

/// A tile set on disk, laid out the way multires_image expects: a .geo file
/// next to a directory holding layer0 (full resolution), layer1 (half), and so
/// on, each holding tileRRRRRxCCCCC.png files.
class TileSetOnDisk
{
public:
  TileSetOnDisk(int width, int height)
  {
    const int layers =
      static_cast<int>(std::ceil(std::log2(std::max(width, height) / double{kTileSize}))) + 1;
    QImage tile(kTileSize, kTileSize, QImage::Format_RGB32);
    tile.fill(Qt::gray);

    for (int layer = 0; layer < layers; layer++) {
      const QString layer_dir = Tiles() + "/layer" + QString::number(layer);
      QDir().mkpath(layer_dir);
      const int columns = TileCount(width, layer);
      const int rows = TileCount(height, layer);
      for (int row = 0; row < rows; row++) {
        for (int column = 0; column < columns; column++) {
          tile.save(
            layer_dir + QString("/tile%1x%2.png")
            .arg(row, 5, 10, QChar('0'))
            .arg(column, 5, 10, QChar('0')));
        }
      }
    }

    QFile geo(QString::fromStdString(GeoFile()));
    EXPECT_TRUE(geo.open(QIODevice::WriteOnly | QIODevice::Text));
    geo.write(
      QString(
        "image_path: tiles\n"
        "image_width: %1\n"
        "image_height: %2\n"
        "tile_size: %3\n"
        "extension: png\n"
        "datum: WGS84\n"
        "projection: utm\n"
        "tiepoints:\n"
        "  - point: [0, 0, 1000.0, 2000.0]\n"
        "  - point: [%1, 0, %4, 2000.0]\n"
        "  - point: [0, %2, 1000.0, %5]\n")
      .arg(width).arg(height).arg(kTileSize)
      .arg(1000.0 + width).arg(2000.0 - height)
      .toUtf8());
  }

  /// Tiles along one edge of a layer, which halves the resolution per layer.
  static int TileCount(int pixels, int layer)
  {
    const double layer_pixels = std::ceil(pixels / std::pow(2.0, layer));
    return static_cast<int>(std::ceil(layer_pixels / kTileSize));
  }

  std::string GeoFile() const {return (dir_.path() + "/image.geo").toStdString();}
  QString Tiles() const {return dir_.path() + "/tiles";}

private:
  QTemporaryDir dir_;
};

std::unique_ptr<multires_image::TileSet> Load(const TileSetOnDisk & on_disk, bool & loaded)
{
  auto tile_set = std::make_unique<multires_image::TileSet>(on_disk.GeoFile());
  loaded = tile_set->Load();
  return tile_set;
}
}  // namespace

TEST(TileSet, LoadsEveryLayer)
{
  const TileSetOnDisk on_disk(1024, 1024);
  bool loaded = false;
  auto tile_set = Load(on_disk, loaded);
  ASSERT_TRUE(loaded);

  // 1024 pixels at 256 per tile is 4 tiles across; each coarser layer halves
  // that, down to a single tile (#814).
  ASSERT_EQ(3, tile_set->LayerCount());
  for (int layer = 0; layer < tile_set->LayerCount(); layer++) {
    SCOPED_TRACE("layer " + std::to_string(layer));
    multires_image::TileSetLayer * tile_layer = tile_set->GetLayer(layer);
    EXPECT_EQ(4 >> layer, tile_layer->ColumnCount());
    EXPECT_EQ(4 >> layer, tile_layer->RowCount());
  }
}

TEST(TileSet, EachLayerHasAQuarterOfTheTilesOfTheOneBelow)
{
  const TileSetOnDisk on_disk(2048, 2048);
  bool loaded = false;
  auto tile_set = Load(on_disk, loaded);
  ASSERT_TRUE(loaded);

  ASSERT_EQ(4, tile_set->LayerCount());
  for (int layer = 1; layer < tile_set->LayerCount(); layer++) {
    SCOPED_TRACE("layer " + std::to_string(layer));
    multires_image::TileSetLayer * below = tile_set->GetLayer(layer - 1);
    multires_image::TileSetLayer * above = tile_set->GetLayer(layer);
    EXPECT_EQ(
      below->ColumnCount() * below->RowCount(),
      4 * above->ColumnCount() * above->RowCount());
  }
}

TEST(TileSet, RoundsPartialTilesUp)
{
  // Neither edge is a multiple of the tile size, and the image isn't square.
  const TileSetOnDisk on_disk(1000, 600);
  bool loaded = false;
  auto tile_set = Load(on_disk, loaded);
  ASSERT_TRUE(loaded);

  ASSERT_EQ(3, tile_set->LayerCount());
  for (int layer = 0; layer < tile_set->LayerCount(); layer++) {
    SCOPED_TRACE("layer " + std::to_string(layer));
    EXPECT_EQ(TileSetOnDisk::TileCount(1000, layer), tile_set->GetLayer(layer)->ColumnCount());
    EXPECT_EQ(TileSetOnDisk::TileCount(600, layer), tile_set->GetLayer(layer)->RowCount());
  }
}

TEST(TileSet, FailsWhenATileIsMissing)
{
  const TileSetOnDisk on_disk(1024, 1024);
  ASSERT_TRUE(QFile::remove(on_disk.Tiles() + "/layer1/tile00001x00001.png"));

  bool loaded = true;
  auto tile_set = Load(on_disk, loaded);
  EXPECT_FALSE(loaded);
}

TEST(TileSet, FailsWhenALayerIsMissing)
{
  const TileSetOnDisk on_disk(1024, 1024);
  ASSERT_TRUE(QDir(on_disk.Tiles() + "/layer2").removeRecursively());

  bool loaded = true;
  auto tile_set = Load(on_disk, loaded);
  EXPECT_FALSE(loaded);
}

int main(int argc, char ** argv)
{
  QCoreApplication app(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
