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
#include <QString>

#include <cstdint>
#include <set>

#include <tile_map/bing_source.hpp>
#include <tile_map/wmts_source.hpp>

using tile_map::BingSource;
using tile_map::WmtsSource;

namespace
{
/// GenerateQuadKey() is protected.
class TestBingSource : public BingSource
{
public:
  TestBingSource()
  : BingSource("test") {}

  using BingSource::GenerateQuadKey;
};

WmtsSource Wmts(const QString & base_url)
{
  return WmtsSource("test", base_url, true, 20);
}
}  // namespace

TEST(WmtsSource, SubstitutesTileCoordinates)
{
  EXPECT_EQ(
    "https://tiles.example.com/5/10/20.png",
    Wmts("https://tiles.example.com/{level}/{x}/{y}.png").GenerateTileUrl(5, 10, 20));
}

TEST(WmtsSource, AcceptsZAsLevel)
{
  // Most tile servers document the zoom level as {z}.
  EXPECT_EQ(
    "http://localhost/tiles/7/1/2.png",
    Wmts("http://localhost/tiles/{z}/{x}/{y}.png").GenerateTileUrl(7, 1, 2));
}

TEST(WmtsSource, HandlesLargeCoordinates)
{
  // Level 20 has more than a million tiles along each edge.
  EXPECT_EQ(
    "http://localhost/20/1048575/524288.png",
    Wmts("http://localhost/{level}/{x}/{y}.png").GenerateTileUrl(20, 1048575, 524288));
}

TEST(WmtsSource, ValidatesBaseUrl)
{
  EXPECT_TRUE(WmtsSource::ValidateBaseUrl("http://localhost/{level}/{x}/{y}.png").isEmpty());
  EXPECT_TRUE(WmtsSource::ValidateBaseUrl("http://localhost/{z}/{x}/{y}.png").isEmpty());

  // Without a placeholder every tile resolves to the same image (#909).
  EXPECT_TRUE(WmtsSource::ValidateBaseUrl("http://localhost/{x}/{y}.png").contains("{level}"));
  EXPECT_TRUE(WmtsSource::ValidateBaseUrl("http://localhost/{z}/{y}.png").contains("{x}"));
  EXPECT_TRUE(WmtsSource::ValidateBaseUrl("http://localhost/{z}/{x}.png").contains("{y}"));
  EXPECT_FALSE(WmtsSource::ValidateBaseUrl("http://localhost/tile.png").isEmpty());
  EXPECT_FALSE(WmtsSource::ValidateBaseUrl("").isEmpty());
  EXPECT_FALSE(WmtsSource::ValidateBaseUrl("   ").isEmpty());
}

TEST(WmtsSource, HashesEveryTileDifferently)
{
  // The hash keys the image cache, so a collision shows one tile's image in
  // another tile's place.
  WmtsSource source = Wmts("http://localhost/{level}/{x}/{y}.png");
  std::set<size_t> hashes;
  size_t tiles = 0;
  for (int32_t level = 1; level <= 4; level++) {
    for (int64_t x = 0; x < (int64_t{1} << level); x++) {
      for (int64_t y = 0; y < (int64_t{1} << level); y++) {
        hashes.insert(source.GenerateTileHash(level, x, y));
        tiles++;
      }
    }
  }
  EXPECT_EQ(tiles, hashes.size());
}

TEST(WmtsSource, HashDependsOnTheSource)
{
  // Switching sources must not reuse the previous source's cached tiles.
  EXPECT_NE(
    Wmts("http://a.example.com/{level}/{x}/{y}.png").GenerateTileHash(3, 1, 2),
    Wmts("http://b.example.com/{level}/{x}/{y}.png").GenerateTileHash(3, 1, 2));
}

TEST(BingSource, GeneratesPublishedQuadKeys)
{
  TestBingSource source;

  // The worked example from Microsoft's "Bing Maps Tile System" article:
  // tile X 3, Y 5 at level 3 is quadkey "213".
  EXPECT_EQ("213", source.GenerateQuadKey(3, 3, 5));

  // One digit per level; each digit is (x bit) + 2 * (y bit).
  EXPECT_EQ("", source.GenerateQuadKey(0, 0, 0));
  EXPECT_EQ("0", source.GenerateQuadKey(1, 0, 0));
  EXPECT_EQ("1", source.GenerateQuadKey(1, 1, 0));
  EXPECT_EQ("2", source.GenerateQuadKey(1, 0, 1));
  EXPECT_EQ("3", source.GenerateQuadKey(1, 1, 1));
  EXPECT_EQ("0000", source.GenerateQuadKey(4, 0, 0));
  EXPECT_EQ("3333", source.GenerateQuadKey(4, 15, 15));
}

int main(int argc, char ** argv)
{
  // BingSource owns a QNetworkAccessManager, which needs an application.
  QCoreApplication app(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
