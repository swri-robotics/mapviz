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
#include <QApplication>
#include <QImage>
#include <QPainter>
#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tile_map/tile_map_plugin.hpp>

namespace
{
rclcpp::Node::SharedPtr g_node;

/// Paint() and CurrentAttribution() are protected.
class TestTileMapPlugin : public tile_map::TileMapPlugin
{
public:
  using tile_map::TileMapPlugin::CurrentAttribution;
  using tile_map::TileMapPlugin::Paint;
};

/// Paints the plugin onto a transparent canvas of the given size.
QImage PaintCanvas(TestTileMapPlugin & plugin, int width, int height)
{
  QImage canvas(width, height, QImage::Format_ARGB32);
  canvas.fill(Qt::transparent);
  QPainter painter(&canvas);
  plugin.Paint(&painter, 0.0, 0.0, 1.0);
  painter.end();
  return canvas;
}

/// True if any pixel in the rectangle has been painted.
bool IsPainted(const QImage & canvas, const QRect & area)
{
  for (int y = area.top(); y <= area.bottom(); ++y) {
    for (int x = area.left(); x <= area.right(); ++x) {
      if (qAlpha(canvas.pixel(x, y)) != 0) {
        return true;
      }
    }
  }
  return false;
}

YAML::Node SaveAfterLoading(tile_map::TileMapPlugin & plugin, const std::string & yaml)
{
  plugin.LoadConfigPlugin(YAML::Load(yaml), "");

  YAML::Emitter emitter;
  emitter << YAML::BeginMap;
  plugin.SaveConfigPlugin(emitter, "");
  emitter << YAML::EndMap;

  return YAML::Load(emitter.c_str());
}

const char kCustomSource[] =
  "custom_sources:\n"
  "  - base_url: https://tile.openstreetmap.org/{level}/{x}/{y}.png\n"
  "    max_zoom: 15\n"
  "    name: OSM Basemap\n"
  "    type: wmts\n"
  "source: OSM Basemap\n";

/// Qt only warns when a connection made by name fails, so collect those
/// warnings.
std::vector<std::string> g_connect_warnings;

void CollectConnectWarnings(
  QtMsgType type, const QMessageLogContext & context, const QString & message)
{
  (void)context;
  if (type == QtWarningMsg && message.startsWith("QObject::connect")) {
    g_connect_warnings.push_back(message.toStdString());
  }
}
}  // namespace

TEST(TileMapSignals, ConnectsEverySignal)
{
  // A signal that doesn't exist, such as QComboBox::activated(QString), which
  // Qt 6 removed, leaves its control doing nothing when it is used.
  g_connect_warnings.clear();
  QtMessageHandler previous_handler = qInstallMessageHandler(CollectConnectWarnings);
  {
    tile_map::TileMapPlugin plugin;
    plugin.SetNode(*g_node);
  }
  qInstallMessageHandler(previous_handler);

  for (const std::string & warning : g_connect_warnings) {
    ADD_FAILURE() << warning;
  }
}

TEST(TileMapConfig, SelectsCustomSourceWithSpaceInName)
{
  tile_map::TileMapPlugin plugin;
  plugin.SetNode(*g_node);

  // A custom source whose name contained a space used to be replaced by the
  // default source on reload (#858).
  YAML::Node saved = SaveAfterLoading(plugin, kCustomSource);

  EXPECT_EQ("OSM Basemap", saved["source"].as<std::string>());
}

TEST(TileMapConfig, RoundTripsCustomSource)
{
  tile_map::TileMapPlugin plugin;
  plugin.SetNode(*g_node);

  YAML::Node saved = SaveAfterLoading(plugin, kCustomSource);

  ASSERT_EQ(1u, saved["custom_sources"].size());
  const YAML::Node source = saved["custom_sources"][0];
  EXPECT_EQ("OSM Basemap", source["name"].as<std::string>());
  EXPECT_EQ(
    "https://tile.openstreetmap.org/{level}/{x}/{y}.png",
    source["base_url"].as<std::string>());
  EXPECT_EQ(15, source["max_zoom"].as<int>());
  EXPECT_EQ("wmts", source["type"].as<std::string>());
}

TEST(TileMapConfig, SurvivesASecondReload)
{
  tile_map::TileMapPlugin first;
  first.SetNode(*g_node);
  YAML::Node saved = SaveAfterLoading(first, kCustomSource);

  // What was saved has to load back to the same selection.
  YAML::Emitter emitter;
  emitter << saved;
  tile_map::TileMapPlugin second;
  second.SetNode(*g_node);
  YAML::Node saved_again = SaveAfterLoading(second, emitter.c_str());

  EXPECT_EQ("OSM Basemap", saved_again["source"].as<std::string>());
  EXPECT_EQ(1u, saved_again["custom_sources"].size());
}

TEST(TileMapAttribution, CreditsEachBuiltInSource)
{
  // The credit lines required by each provider's terms of use.
  const std::vector<std::pair<std::string, QString>> expected = {
    {"Carto", "© OpenStreetMap contributors, © CARTO"},
    {"OpenStreetMap", "© OpenStreetMap contributors"},
    {"Stamen (terrain)", "© Stadia Maps © Stamen Design © OpenMapTiles © OpenStreetMap"},
    {"Stamen (toner)", "© Stadia Maps © Stamen Design © OpenMapTiles © OpenStreetMap"},
    {"Stamen (watercolor)", "© Stadia Maps © Stamen Design © OpenStreetMap"},
    {"USGS Satellite", "USDA, USGS The National Map: Orthoimagery"},
    // Bing's credits come from its metadata, which needs an API key.
    {"Bing Maps (terrain)", ""},
  };

  for (const auto & source : expected) {
    TestTileMapPlugin plugin;
    plugin.SetNode(*g_node);
    plugin.LoadConfigPlugin(YAML::Load("source: " + source.first), "");
    EXPECT_EQ(source.second, plugin.CurrentAttribution()) << source.first;
  }
}

TEST(TileMapAttribution, PaintsInTheBottomRightCorner)
{
  TestTileMapPlugin plugin;
  plugin.SetNode(*g_node);
  plugin.LoadConfigPlugin(YAML::Load("source: Carto"), "");

  const QImage canvas = PaintCanvas(plugin, 400, 300);

  // Just inside the margin at the bottom right, and nowhere near the top or
  // the left edge.
  EXPECT_TRUE(IsPainted(canvas, QRect(380, 280, 10, 10)));
  EXPECT_FALSE(IsPainted(canvas, QRect(0, 0, 400, 200)));
  EXPECT_FALSE(IsPainted(canvas, QRect(0, 0, 50, 300)));
  // The margin itself stays clear.
  EXPECT_FALSE(IsPainted(canvas, QRect(396, 0, 4, 300)));
  EXPECT_FALSE(IsPainted(canvas, QRect(0, 296, 400, 4)));
}

TEST(TileMapAttribution, WrapsOnANarrowCanvas)
{
  TestTileMapPlugin plugin;
  plugin.SetNode(*g_node);
  plugin.LoadConfigPlugin(YAML::Load("source: Stamen (terrain)"), "");

  // Too narrow for the credit on one line; it has to wrap instead of
  // running off the left edge.
  const QImage canvas = PaintCanvas(plugin, 120, 300);

  EXPECT_TRUE(IsPainted(canvas, QRect(100, 280, 10, 10)));
  EXPECT_FALSE(IsPainted(canvas, QRect(0, 0, 4, 300)));
}

TEST(TileMapAttribution, PaintsNothingWithoutAnAttribution)
{
  TestTileMapPlugin plugin;
  plugin.SetNode(*g_node);
  plugin.LoadConfigPlugin(YAML::Load(kCustomSource), "");

  const QImage canvas = PaintCanvas(plugin, 400, 300);

  EXPECT_FALSE(IsPainted(canvas, canvas.rect()));
}

int main(int argc, char ** argv)
{
  // The plugin builds a QWidget based config panel in its constructor, so a
  // QApplication has to exist first.  The test runs headless; see the ENV set
  // on this target in CMakeLists.txt.
  QApplication app(argc, argv);
  rclcpp::init(argc, argv);
  g_node = std::make_shared<rclcpp::Node>("test_tile_map_plugin");

  testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();

  g_node.reset();
  rclcpp::shutdown();
  return result;
}
