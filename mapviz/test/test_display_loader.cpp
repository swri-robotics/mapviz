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
#include <yaml-cpp/yaml.h>

#include <cstdio>
#include <fstream>
#include <functional>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

#include <mapviz/display_loader.hpp>
#include <pluginlib/exceptions.hpp>
#include <rclcpp/exceptions.hpp>
#include <rclcpp/logging.hpp>

using mapviz::DisplaySpec;
using mapviz::LoadConfigFile;
using mapviz::LoadDisplays;

namespace
{
const rclcpp::Logger kLogger = rclcpp::get_logger("test_display_loader");

void LoadsFine(const DisplaySpec &) {}

/// Loads @p yaml's displays, recording the ones that loaded and letting
/// @p fail throw for any display it chooses.
struct Load
{
  explicit Load(
    const std::string & yaml,
    std::function<void(const DisplaySpec &)> fail = LoadsFine)
  {
    failures = LoadDisplays(
      YAML::Load(yaml)["displays"],
      [this, &fail](const DisplaySpec & display) {
        fail(display);
        loaded.push_back(display);
      },
      kLogger);
  }

  std::vector<std::string> Names() const
  {
    std::vector<std::string> names;
    for (const DisplaySpec & display : loaded) {
      names.push_back(display.name);
    }
    return names;
  }

  std::vector<DisplaySpec> loaded;
  std::vector<std::string> failures;
};

const char kThreeDisplays[] =
  "displays:\n"
  "  - type: mapviz_plugins/grid\n"
  "    name: first\n"
  "    config: {visible: true, collapsed: false}\n"
  "  - type: mapviz_plugins/odometry\n"
  "    name: second\n"
  "    config: {visible: false, collapsed: true, topic: /odom}\n"
  "  - type: mapviz_plugins/pose\n"
  "    name: third\n"
  "    config: {visible: true, collapsed: true}\n";

const std::vector<std::string> kFirstAndThird = {"first", "third"};

/// A failure a display can hit while being created or configured.
struct FailureCase
{
  std::string name;
  std::function<void()> fail;
};

void PrintTo(const FailureCase & failure_case, std::ostream * os)
{
  *os << failure_case.name;
}

[[noreturn]] void ThrowLibraryLoadError()
{
  throw pluginlib::LibraryLoadException("no such plugin");
}

void ThrowYamlError()
{
  YAML::Node config = YAML::Load("{topic: /odom}");
  config["topic"].as<int>();
}

[[noreturn]] void ThrowRclError()
{
  rcl_error_state_t state{};
  std::snprintf(state.message, sizeof(state.message), "%s", "simulated failure");
  throw rclcpp::exceptions::RCLError(RCL_RET_ERROR, &state, "loading a display");
}

[[noreturn]] void ThrowInvalidTopicName()
{
  throw rclcpp::exceptions::InvalidTopicNameError("not a topic", "has a space", 3);
}

[[noreturn]] void ThrowNonStandardException()
{
  throw 42;
}
}  // namespace

TEST(DisplayLoader, LoadsEveryDisplay)
{
  const Load load(kThreeDisplays);

  EXPECT_TRUE(load.failures.empty());
  ASSERT_EQ(3u, load.loaded.size());
  EXPECT_EQ("mapviz_plugins/odometry", load.loaded[1].type);
  EXPECT_EQ("second", load.loaded[1].name);
  EXPECT_FALSE(load.loaded[1].visible);
  EXPECT_TRUE(load.loaded[1].collapsed);
  EXPECT_EQ("/odom", load.loaded[1].config["topic"].as<std::string>());
}

class DisplayLoaderFailure : public ::testing::TestWithParam<FailureCase> {};

TEST_P(DisplayLoaderFailure, KeepsLoadingTheOtherDisplays)
{
  // Any one display failing, however it fails, must not stop the displays
  // after it from loading (#835).
  const Load load(
    kThreeDisplays, [](const DisplaySpec & display) {
      if (display.name == "second") {
        GetParam().fail();
      }
    });

  EXPECT_EQ(kFirstAndThird, load.Names());
  ASSERT_EQ(1u, load.failures.size());
  EXPECT_EQ("mapviz_plugins/odometry (second)", load.failures[0]);
}

INSTANTIATE_TEST_SUITE_P(
  Failures, DisplayLoaderFailure,
  ::testing::Values(
    FailureCase{"missing_library", ThrowLibraryLoadError},
    FailureCase{"yaml_error", ThrowYamlError},
    // Used to be logged but left out of the "failed to load" dialog.
    FailureCase{"rcl_error", ThrowRclError},
    // An invalid topic name, which is not an RCLError, used to escape
    // Mapviz::Open() and skip every display after it.
    FailureCase{"invalid_topic_name", ThrowInvalidTopicName},
    FailureCase{"not_a_std_exception", ThrowNonStandardException}),
  [](const ::testing::TestParamInfo<FailureCase> & info) {return info.param.name;});

TEST(DisplayLoader, FillsInMissingSettings)
{
  // Missing settings used to throw before the display was even tried, and
  // that stopped the whole config from loading.
  const Load load(
    "displays:\n"
    "  - type: mapviz_plugins/grid\n");

  EXPECT_TRUE(load.failures.empty());
  ASSERT_EQ(1u, load.loaded.size());
  EXPECT_EQ("mapviz_plugins/grid", load.loaded[0].name);
  EXPECT_TRUE(load.loaded[0].visible);
  EXPECT_FALSE(load.loaded[0].collapsed);
  EXPECT_TRUE(load.loaded[0].config.IsMap());
}

TEST(DisplayLoader, ReportsADisplayWithoutAType)
{
  const Load load(
    "displays:\n"
    "  - name: nameless\n"
    "  - type: mapviz_plugins/grid\n"
    "    name: grid\n");

  EXPECT_EQ(std::vector<std::string>{"grid"}, load.Names());
  EXPECT_EQ(std::vector<std::string>{"display 1"}, load.failures);
}

TEST(DisplayLoader, ReportsAnUnreadableSetting)
{
  const Load load(
    "displays:\n"
    "  - type: mapviz_plugins/grid\n"
    "    name: grid\n"
    "  - type: mapviz_plugins/pose\n"
    "    name: pose\n"
    "    config: {visible: sometimes}\n"
    "  - type: mapviz_plugins/path\n"
    "    name: path\n");

  EXPECT_EQ((std::vector<std::string>{"grid", "path"}), load.Names());
  EXPECT_EQ(std::vector<std::string>{"display 2"}, load.failures);
}

TEST(DisplayLoader, ReportsDisplaysThatAreNotAList)
{
  const Load load("displays: mapviz_plugins/grid\n");

  EXPECT_TRUE(load.loaded.empty());
  EXPECT_EQ(1u, load.failures.size());
}

TEST(DisplayLoader, LoadsNothingFromAnEmptyList)
{
  const Load load("displays: []\n");

  EXPECT_TRUE(load.loaded.empty());
  EXPECT_TRUE(load.failures.empty());
}

/// A config file that is removed again when the test ends.
class ConfigFile
{
public:
  explicit ConfigFile(const std::string & contents)
  : path_(::testing::TempDir() + "test_display_loader.mvc")
  {
    std::ofstream(path_) << contents;
  }

  ~ConfigFile() {std::remove(path_.c_str());}

  const std::string & Path() const {return path_;}

private:
  std::string path_;
};

TEST(ConfigFile, ReadsAConfig)
{
  const ConfigFile file("fixed_frame: map\ndisplays: []\n");

  const std::optional<YAML::Node> config = LoadConfigFile(file.Path(), kLogger);

  ASSERT_TRUE(config);
  EXPECT_EQ("map", (*config)["fixed_frame"].as<std::string>());
}

TEST(ConfigFile, ReadsAnEmptyFileAsAnEmptyConfig)
{
  const ConfigFile file("");

  const std::optional<YAML::Node> config = LoadConfigFile(file.Path(), kLogger);

  ASSERT_TRUE(config);
  EXPECT_TRUE(config->IsMap());
  EXPECT_EQ(0u, config->size());
}

TEST(ConfigFile, ReportsAMissingFile)
{
  // Mapviz opens ~/.mapviz_config at startup, which the first run doesn't
  // have yet.  Throwing from there used to skip the rest of startup.
  EXPECT_FALSE(LoadConfigFile(::testing::TempDir() + "no_such_config.mvc", kLogger));
}

TEST(ConfigFile, ReportsInvalidYaml)
{
  const ConfigFile file("displays:\n  - type: mapviz_plugins/grid\n    name: [unclosed\n");

  EXPECT_FALSE(LoadConfigFile(file.Path(), kLogger));
}

TEST(ConfigFile, ReportsAFileThatIsNotAMapOfSettings)
{
  const ConfigFile file("- mapviz_plugins/grid\n- mapviz_plugins/path\n");

  EXPECT_FALSE(LoadConfigFile(file.Path(), kLogger));
}
