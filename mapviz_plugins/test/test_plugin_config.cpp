// *****************************************************************************
//
// Copyright (c) 2026, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#include <memory>
#include <string>

#include <gtest/gtest.h>

#include <QApplication>

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <mapviz_plugins/draw_marker_plugin.hpp>
#include <mapviz_plugins/speedometer_plugin.hpp>

namespace
{
  /// Shared by every test.  Plugins subscribe while loading their config, so
  /// they need a node just as they get one from Mapviz::CreateNewDisplay(),
  /// which calls SetNode() before LoadConfigPlugin().
  rclcpp::Node::SharedPtr g_node;

  /// Round trips a plugin's configuration the way mapviz does when a config
  /// file is opened and then saved again.
  YAML::Node SaveAfterLoading(mapviz::MapvizPlugin& plugin, const std::string& yaml)
  {
    plugin.LoadConfigPlugin(YAML::Load(yaml), "");

    YAML::Emitter emitter;
    emitter << YAML::BeginMap;
    plugin.SaveConfigPlugin(emitter, "");
    emitter << YAML::EndMap;

    return YAML::Load(emitter.c_str());
  }
}  // namespace

TEST(SpeedometerConfig, RoundTripsSettings)
{
  mapviz_plugins::SpeedometerPlugin plugin;
  plugin.SetNode(*g_node);

  YAML::Node saved = SaveAfterLoading(plugin,
    "topic: /vehicle/odom\n"
    "max_speed: 33.5\n"
    "color: '#ff00ff'\n"
    "x: 40\n"
    "y: 55\n"
    "width: 180\n"
    "height: 190\n");

  EXPECT_EQ("/vehicle/odom", saved["topic"].as<std::string>());
  EXPECT_DOUBLE_EQ(33.5, saved["max_speed"].as<double>());
  EXPECT_EQ("#ff00ff", saved["color"].as<std::string>());
  EXPECT_EQ(40, saved["x"].as<int>());
  EXPECT_EQ(55, saved["y"].as<int>());
  EXPECT_EQ(180, saved["width"].as<int>());
  EXPECT_EQ(190, saved["height"].as<int>());
}

TEST(SpeedometerConfig, UsesDefaultMaxSpeedWhenUnset)
{
  mapviz_plugins::SpeedometerPlugin plugin;
  plugin.SetNode(*g_node);

  YAML::Node saved = SaveAfterLoading(plugin, "topic: /odom\n");

  EXPECT_DOUBLE_EQ(40.0, saved["max_speed"].as<double>());
}

TEST(SpeedometerConfig, ClampsOutOfRangeMaxSpeed)
{
  mapviz_plugins::SpeedometerPlugin plugin;
  plugin.SetNode(*g_node);

  // A hand edited config must not be able to push the dial outside the range
  // the config widget allows.
  EXPECT_DOUBLE_EQ(1000.0,
    SaveAfterLoading(plugin, "max_speed: 99999999\n")["max_speed"].as<double>());
  EXPECT_DOUBLE_EQ(0.1,
    SaveAfterLoading(plugin, "max_speed: -5\n")["max_speed"].as<double>());
}

TEST(DrawMarkerConfig, RoundTripsSettings)
{
  mapviz_plugins::DrawMarkerPlugin plugin;
  plugin.SetNode(*g_node);

  YAML::Node saved = SaveAfterLoading(plugin,
    "frame: map\n"
    "topic: /drawn\n"
    "type: 2\n"
    "namespace: shapes\n"
    "id: 7\n"
    "color: '#00ffff'\n"
    "alpha: 0.5\n"
    "scale: 2.5\n");

  EXPECT_EQ("map", saved["frame"].as<std::string>());
  EXPECT_EQ("/drawn", saved["topic"].as<std::string>());
  EXPECT_EQ(2, saved["type"].as<int>());
  EXPECT_EQ("shapes", saved["namespace"].as<std::string>());
  EXPECT_EQ(7, saved["id"].as<int>());
  EXPECT_EQ("#00ffff", saved["color"].as<std::string>());
  EXPECT_DOUBLE_EQ(0.5, saved["alpha"].as<double>());
  EXPECT_DOUBLE_EQ(2.5, saved["scale"].as<double>());
}

TEST(DrawMarkerConfig, RoundTripsVertices)
{
  mapviz_plugins::DrawMarkerPlugin plugin;
  plugin.SetNode(*g_node);

  // Persisting the vertices is what lets a drawing outlive the session it was
  // made in, so the coordinates have to survive exactly.
  YAML::Node saved = SaveAfterLoading(plugin,
    "vertices:\n"
    "  - [1.5, -2.5]\n"
    "  - [3.0, 4.0]\n"
    "  - [-5.25, 6.125]\n");

  ASSERT_EQ(3u, saved["vertices"].size());
  EXPECT_DOUBLE_EQ(1.5, saved["vertices"][0][0].as<double>());
  EXPECT_DOUBLE_EQ(-2.5, saved["vertices"][0][1].as<double>());
  EXPECT_DOUBLE_EQ(3.0, saved["vertices"][1][0].as<double>());
  EXPECT_DOUBLE_EQ(4.0, saved["vertices"][1][1].as<double>());
  EXPECT_DOUBLE_EQ(-5.25, saved["vertices"][2][0].as<double>());
  EXPECT_DOUBLE_EQ(6.125, saved["vertices"][2][1].as<double>());
}

TEST(DrawMarkerConfig, HandlesConfigWithoutVertices)
{
  mapviz_plugins::DrawMarkerPlugin plugin;
  plugin.SetNode(*g_node);

  // Configs written before vertices were persisted have no such key.
  YAML::Node saved = SaveAfterLoading(plugin, "frame: map\n");

  ASSERT_TRUE(saved["vertices"]);
  EXPECT_EQ(0u, saved["vertices"].size());
}

TEST(DrawMarkerConfig, IgnoresMalformedVertices)
{
  mapviz_plugins::DrawMarkerPlugin plugin;
  plugin.SetNode(*g_node);

  // A vertex needs both coordinates; a short one is skipped rather than read
  // out of bounds.
  YAML::Node saved = SaveAfterLoading(plugin,
    "vertices:\n"
    "  - [1.0, 2.0]\n"
    "  - [3.0]\n"
    "  - [4.0, 5.0]\n");

  ASSERT_EQ(2u, saved["vertices"].size());
  EXPECT_DOUBLE_EQ(1.0, saved["vertices"][0][0].as<double>());
  EXPECT_DOUBLE_EQ(4.0, saved["vertices"][1][0].as<double>());
}

int main(int argc, char** argv)
{
  // The plugins build QWidget based config panels in their constructors, so a
  // QApplication has to exist first.  The test runs headless; see the ENV set
  // on this target in CMakeLists.txt.
  QApplication app(argc, argv);
  rclcpp::init(argc, argv);
  g_node = std::make_shared<rclcpp::Node>("test_plugin_config");

  testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();

  g_node.reset();
  rclcpp::shutdown();
  return result;
}
