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
#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>

#include <mapviz/mapviz_plugin.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace
{
rclcpp::Node::SharedPtr g_node;

/// The smallest concrete plugin, exposing the protected helpers that every
/// real plugin builds on.
class TestPlugin : public mapviz::MapvizPlugin
{
public:
  using MapvizPlugin::Initialize;
  using MapvizPlugin::LoadQosConfig;
  using MapvizPlugin::SaveQosConfig;
  using MapvizPlugin::Subscribe;
  using MapvizPlugin::TrimString;

  bool Initialize(QOpenGLWidget *) override {return true;}
  void Shutdown() override {}
  void PrintError(const std::string &) override {}
  void PrintInfo(const std::string &) override {}
  void PrintWarning(const std::string &) override {}

protected:
  void Draw(double, double, double) override {}
  void Transform() override {}
  void LoadConfig(const YAML::Node &, const std::string &) override {}
  void SaveConfig(YAML::Emitter &, const std::string &) override {}
};

/// A profile where every persisted field differs from rmw_qos_profile_default,
/// so a field that is dropped anywhere shows up as a mismatch.
rmw_qos_profile_t NonDefaultQos()
{
  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.depth = 7;
  qos.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
  qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  return qos;
}

YAML::Node SaveQos(
  const TestPlugin & plugin, const rmw_qos_profile_t & qos,
  const std::string & prefix)
{
  YAML::Emitter emitter;
  emitter << YAML::BeginMap;
  plugin.SaveQosConfig(emitter, qos, prefix);
  emitter << YAML::EndMap;
  return YAML::Load(emitter.c_str());
}

void ExpectSameQos(const rmw_qos_profile_t & expected, const rmw_qos_profile_t & actual)
{
  EXPECT_EQ(expected.depth, actual.depth);
  EXPECT_EQ(expected.history, actual.history);
  EXPECT_EQ(expected.reliability, actual.reliability);
  EXPECT_EQ(expected.durability, actual.durability);
}
}  // namespace

TEST(QosConfig, RoundTripsEveryField)
{
  TestPlugin plugin;
  const rmw_qos_profile_t saved = NonDefaultQos();

  rmw_qos_profile_t loaded = rmw_qos_profile_default;
  plugin.LoadQosConfig(SaveQos(plugin, saved, ""), loaded);

  ExpectSameQos(saved, loaded);
}

TEST(QosConfig, RoundTripsEveryFieldWithPrefix)
{
  TestPlugin plugin;
  const rmw_qos_profile_t saved = NonDefaultQos();

  // Plugins with two subscriptions, like the route plugin, keep them apart
  // with a prefix.
  rmw_qos_profile_t loaded = rmw_qos_profile_default;
  plugin.LoadQosConfig(SaveQos(plugin, saved, "position"), loaded, "position");

  ExpectSameQos(saved, loaded);
}

TEST(QosConfig, PrefixesDoNotCollide)
{
  TestPlugin plugin;
  const rmw_qos_profile_t saved = NonDefaultQos();

  rmw_qos_profile_t loaded = rmw_qos_profile_default;
  plugin.LoadQosConfig(SaveQos(plugin, saved, "route"), loaded, "position");

  ExpectSameQos(rmw_qos_profile_default, loaded);
}

TEST(QosConfig, ReadsTheKeysWrittenToConfigFiles)
{
  TestPlugin plugin;

  // Saved configs already use these key names, so load and save both have to
  // keep using them.  A misspelled key in either direction (see #822) makes
  // the setting silently fall back to its default.
  rmw_qos_profile_t loaded = rmw_qos_profile_default;
  plugin.LoadQosConfig(
    YAML::Load(
      "qos_depth: 7\n"
      "qos_history: 2\n"
      "qos_reliability: 2\n"
      "qos_durability: 1\n"),
    loaded);
  ExpectSameQos(NonDefaultQos(), loaded);

  YAML::Node saved = SaveQos(plugin, NonDefaultQos(), "");
  EXPECT_EQ(7, saved["qos_depth"].as<int>());
  EXPECT_EQ(2, saved["qos_history"].as<int>());
  EXPECT_EQ(2, saved["qos_reliability"].as<int>());
  EXPECT_EQ(1, saved["qos_durability"].as<int>());
}

TEST(QosConfig, KeepsCurrentValuesForMissingKeys)
{
  TestPlugin plugin;

  // Configs written before the QoS settings existed have none of these keys.
  rmw_qos_profile_t loaded = NonDefaultQos();
  plugin.LoadQosConfig(YAML::Load("topic: /fix\n"), loaded);

  ExpectSameQos(NonDefaultQos(), loaded);
}

TEST(Subscribe, AppliesEveryQosField)
{
  TestPlugin plugin;
  plugin.SetNode(*g_node);

  rmw_qos_profile_t requested = NonDefaultQos();
  // KEEP_ALL makes the middleware ignore depth; use KEEP_LAST so depth is
  // checked too.
  requested.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
  plugin.Subscribe<std_msgs::msg::String>(
    "/test_mapviz_plugin/qos", requested, sub,
    [](std_msgs::msg::String::ConstSharedPtr) {});
  ASSERT_TRUE(sub);

  // Before #826 only history and depth reached the subscription, so it came
  // up reliable and volatile whatever the user chose.
  ExpectSameQos(requested, sub->get_actual_qos().get_rmw_qos_profile());
}

TEST(TrimString, RemovesLeadingAndTrailingWhitespace)
{
  TestPlugin plugin;

  EXPECT_EQ("abc", plugin.TrimString("  abc  "));
  EXPECT_EQ("abc", plugin.TrimString("\t\nabc\r\n"));
  EXPECT_EQ("abc", plugin.TrimString("abc"));
}

TEST(TrimString, KeepsInteriorWhitespace)
{
  TestPlugin plugin;

  // Tile source names such as "OSM Basemap" are trimmed before they are saved
  // (see #858).
  EXPECT_EQ("OSM Basemap", plugin.TrimString(" OSM Basemap "));
  EXPECT_EQ("a  b\tc", plugin.TrimString("a  b\tc"));
}

TEST(TrimString, HandlesEmptyAndBlankStrings)
{
  TestPlugin plugin;

  EXPECT_EQ("", plugin.TrimString(""));
  EXPECT_EQ("", plugin.TrimString(" "));
  EXPECT_EQ("", plugin.TrimString(" \t\n "));
  EXPECT_EQ("x", plugin.TrimString(" x"));
  EXPECT_EQ("x", plugin.TrimString("x "));
}

int main(int argc, char ** argv)
{
  // MapvizPlugin is a QObject, and Subscribe() hands messages to the Qt event
  // loop, so an application object has to exist.
  QCoreApplication app(argc, argv);
  rclcpp::init(argc, argv);
  g_node = std::make_shared<rclcpp::Node>("test_mapviz_plugin");

  testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();

  g_node.reset();
  rclcpp::shutdown();
  return result;
}
