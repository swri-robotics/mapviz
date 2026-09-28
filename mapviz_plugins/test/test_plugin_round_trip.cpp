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
#include <QFont>
#include <QOpenGLWidget>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <functional>
#include <memory>
#include <ostream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <mapviz_plugins/attitude_indicator_plugin.hpp>
#include <mapviz_plugins/coordinate_picker_plugin.hpp>
#include <mapviz_plugins/disparity_plugin.hpp>
#include <mapviz_plugins/draw_polygon_plugin.hpp>
#include <mapviz_plugins/float_plugin.hpp>
#include <mapviz_plugins/gps_plugin.hpp>
#include <mapviz_plugins/grid_plugin.hpp>
#include <mapviz_plugins/image_plugin.hpp>
#include <mapviz_plugins/laserscan_plugin.hpp>
#include <mapviz_plugins/marker_plugin.hpp>
#include <mapviz_plugins/measuring_plugin.hpp>
#include <mapviz_plugins/navsat_plugin.hpp>
#include <mapviz_plugins/occupancy_grid_plugin.hpp>
#include <mapviz_plugins/odometry_plugin.hpp>
#include <mapviz_plugins/path_plugin.hpp>
#include <mapviz_plugins/plan_route_plugin.hpp>
#include <mapviz_plugins/point_click_publisher_plugin.hpp>
#include <mapviz_plugins/pointcloud2_plugin.hpp>
#include <mapviz_plugins/pose_plugin.hpp>
#include <mapviz_plugins/robot_image_plugin.hpp>
#include <mapviz_plugins/robot_model_plugin.hpp>
#include <mapviz_plugins/route_plugin.hpp>
#include <mapviz_plugins/string_plugin.hpp>
#include <mapviz_plugins/textured_marker_plugin.hpp>
#include <mapviz_plugins/tf_frame_plugin.hpp>

namespace
{
rclcpp::Node::SharedPtr g_node;

/// Stands in for the map canvas.  It is never shown, so no OpenGL context is
/// needed; plugins only ask it to repaint.
std::unique_ptr<QOpenGLWidget> g_canvas;

/// Some plugins use the canvas while loading their config, to repaint or to
/// convert between pixels and percent.  Mapviz provides it through
/// Initialize(), which also needs an OpenGL context, so hand it over directly.
template<typename PluginT>
class WithCanvas : public PluginT
{
public:
  // Qualified, because some plugins declare a canvas_ of their own.
  WithCanvas() {this->mapviz::MapvizPlugin::canvas_ = g_canvas.get();}
};

/// Non-default QoS settings for every key LoadQosConfig() reads.
std::string Qos(const std::string & prefix = "")
{
  return prefix + "qos_depth: 7\n" +
         prefix + "qos_history: 2\n" +
         prefix + "qos_reliability: 2\n" +
         prefix + "qos_durability: 1\n";
}

/// A font in the format the running Qt version writes, which differs between
/// Qt 5 and Qt 6.
std::string Font()
{
  return "'" + QFont("Sans Serif", 26).toString().toStdString() + "'";
}

/// Concatenates the pieces of a config.  Not a fold expression: uncrustify
/// strips the parentheses it needs from a return statement.
std::string Join() {return "";}

template<typename First, typename ... Rest>
std::string Join(const First & first, const Rest & ... rest)
{
  return std::string(first) + Join(rest ...);
}

struct RoundTripCase
{
  std::string name;
  std::function<std::unique_ptr<mapviz::MapvizPlugin>()> create;
  /// Every key a plugin saves, set to something other than its default.
  std::function<std::string()> config;
};

template<typename PluginT>
RoundTripCase Case(const std::string & name, std::function<std::string()> config)
{
  return {name, [] {return std::make_unique<WithCanvas<PluginT>>();}, std::move(config)};
}

void PrintTo(const RoundTripCase & round_trip_case, std::ostream * os)
{
  *os << round_trip_case.name;
}

/// Compares numbers numerically, so 2 and 2.0 match, and anything else as text.
::testing::AssertionResult SameValue(const YAML::Node & expected, const YAML::Node & actual)
{
  if (!actual) {
    return ::testing::AssertionFailure() << "not saved";
  }
  const std::string expected_text = expected.as<std::string>();
  const std::string actual_text = actual.as<std::string>();
  double expected_number, actual_number;
  if (YAML::convert<double>::decode(expected, expected_number) &&
    YAML::convert<double>::decode(actual, actual_number))
  {
    if (std::abs(expected_number - actual_number) < 1e-9) {
      return ::testing::AssertionSuccess();
    }
  } else if (expected_text == actual_text) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure() <<
         "saved as '" << actual_text << "', loaded as '" << expected_text << "'";
}
}  // namespace

class ConfigRoundTrip : public ::testing::TestWithParam<RoundTripCase> {};

TEST_P(ConfigRoundTrip, SavesWhatItLoaded)
{
  const RoundTripCase & round_trip_case = GetParam();
  const YAML::Node loaded = YAML::Load(round_trip_case.config());

  std::unique_ptr<mapviz::MapvizPlugin> plugin = round_trip_case.create();
  plugin->SetNode(*g_node);
  plugin->LoadConfigPlugin(loaded, "");

  YAML::Emitter emitter;
  emitter << YAML::BeginMap;
  plugin->SaveConfigPlugin(emitter, "");
  emitter << YAML::EndMap;
  const YAML::Node saved = YAML::Load(emitter.c_str());

  // A setting that doesn't come back is silently reset the next time mapviz
  // starts (see #790).
  for (const auto & entry : loaded) {
    const std::string key = entry.first.as<std::string>();
    EXPECT_TRUE(SameValue(entry.second, saved[key])) << "key: " << key;
  }
}

INSTANTIATE_TEST_SUITE_P(
  Plugins, ConfigRoundTrip,
  ::testing::Values(
    Case<mapviz_plugins::AttitudeIndicatorPlugin>(
      "attitude_indicator", [] {
        return "topic: /imu\nx: 30\ny: 40\nwidth: 150\nheight: 160\n" + Qos();
      }),
    Case<mapviz_plugins::CoordinatePickerPlugin>(
      "coordinate_picker", [] {
        return std::string("frame: utm\ncopy: false\n");
      }),
    Case<mapviz_plugins::DisparityPlugin>(
      "disparity", [] {
        return Join(
          "topic: /disparity\n",
          "anchor: bottom right\n",
          "units: percent\n",
          "offset_x: 12\n",
          "offset_y: 34\n",
          "width: 55\n",
          "height: 66\n",
          Qos());
      }),
    Case<mapviz_plugins::DrawPolygonPlugin>(
      "draw_polygon", [] {
        return std::string("frame: utm\npolygon_topic: /polygon\ncolor: '#00ff00'\n");
      }),
    Case<mapviz_plugins::FloatPlugin>(
      "float", [] {
        return Join(
          "topic: /speed\n",
          "font: " + Font() + "\n",
          "color: '#123456'\n",
          "anchor: bottom center\n",
          "units: percent\n",
          "offset_x: 11\n",
          "offset_y: 22\n",
          "postfix_text: m/s\n",
          Qos());
      }),
    Case<mapviz_plugins::GpsPlugin>(
      "gps", [] {
        return Join(
          "topic: /gps\n",
          "color: '#123456'\n",
          "draw_style: arrows\n",
          "position_tolerance: 2.5\n",
          "buffer_size: 42\n",
          "show_laps: true\n",
          "static_arrow_sizes: true\n",
          "arrow_size: 33\n",
          Qos());
      }),
    Case<mapviz_plugins::GridPlugin>(
      "grid", [] {
        return std::string(
          "color: '#123456'\nalpha: 0.25\nframe: utm\nx: 1.5\ny: -2.5\n"
          "size: 7.5\nrows: 12\ncolumns: 13\n");
      }),
    Case<mapviz_plugins::ImagePlugin>(
      "image", [] {
        // keep_ratio is off: when on, height follows width and the image's
        // aspect ratio instead of the saved value.
        return Join(
          "topic: /camera/image\n",
          "anchor: center right\n",
          "units: percent\n",
          "offset_x: 12\n",
          "offset_y: 34\n",
          "width: 55\n",
          "height: 66\n",
          "keep_ratio: false\n",
          "rotation: 90\n",
          "image_transport: raw\n",
          Qos());
      }),
    Case<mapviz_plugins::LaserScanPlugin>(
      "laserscan", [] {
        return Join(
          "topic: /scan\n",
          "size: 5\n",
          "buffer_size: 17\n",
          "alpha: 0.25\n",
          "color_transformer: Range\n",
          "min_color: '#010203'\n",
          "max_color: '#040506'\n",
          "value_min: 1.5\n",
          "value_max: 25.5\n",
          "use_rainbow: false\n",
          Qos());
      }),
    Case<mapviz_plugins::MarkerPlugin>(
      "marker", [] {
        return "topic: /markers\nuse_pixel_scale: true\n" + Qos();
      }),
    Case<mapviz_plugins::MeasuringPlugin>(
      "measuring", [] {
        return std::string(
          "main_color: '#123456'\nbkgnd_color: '#654321'\nshow_bkgnd_color: false\n"
          "show_measurements: false\nfont_size: 17\nalpha: 0.25\n");
      }),
    Case<mapviz_plugins::NavSatPlugin>(
      "navsat", [] {
        return Join(
          "topic: /fix\n",
          "color: '#123456'\n",
          "draw_style: points\n",
          "position_tolerance: 2.5\n",
          "buffer_size: 42\n",
          Qos());
      }),
    Case<mapviz_plugins::OccupancyGridPlugin>(
      "occupancy_grid", [] {
        // The alpha spin box shows one decimal place.
        return "topic: /map\nupdate: true\nalpha: 0.4\nscheme: costmap\n" + Qos();
      }),
    Case<mapviz_plugins::OdometryPlugin>(
      "odometry", [] {
        return Join(
          "topic: /odom\n",
          "color: '#123456'\n",
          "draw_style: arrows\n",
          "position_tolerance: 2.5\n",
          "buffer_size: 42\n",
          "show_laps: true\n",
          "show_covariance: true\n",
          "show_all_covariances: true\n",
          "show_timestamps: 5\n",
          "static_arrow_sizes: true\n",
          "arrow_size: 33\n",
          Qos());
      }),
    Case<mapviz_plugins::PathPlugin>(
      "path", [] {
        return "topic: /plan\ncolor: '#123456'\n" + Qos();
      }),
    Case<mapviz_plugins::PlanRoutePlugin>(
      "plan_route", [] {
        return std::string(
          "route_topic: /planned_route\nservice: /plan_route\ncolor: '#123456'\n"
          "start_from_vehicle: true\n");
      }),
    Case<mapviz_plugins::PointClickPublisherPlugin>(
      "point_click_publisher", [] {
        return std::string("topic: /clicked\noutput_frame: utm\n");
      }),
    Case<mapviz_plugins::PointCloud2Plugin>(
      "pointcloud2", [] {
        // #804, #901: the auto min/max flag and the range it computed.
        return Join(
          "topic: /points\n",
          "size: 5\n",
          "buffer_size: 17\n",
          "alpha: 0.25\n",
          "color_transformer: Z Axis\n",
          "min_color: '#010203'\n",
          "max_color: '#040506'\n",
          "value_min: 1.5\n",
          "value_max: 25.5\n",
          "use_rainbow: false\n",
          "unpack_rgb: true\n",
          "use_automaxmin: true\n",
          Qos());
      }),
    Case<mapviz_plugins::PosePlugin>(
      "pose", [] {
        return Join(
          "topic: /pose\n",
          "color: '#123456'\n",
          "draw_style: arrows\n",
          "position_tolerance: 2.5\n",
          "buffer_size: 42\n",
          "show_laps: true\n",
          "static_arrow_sizes: true\n",
          "arrow_size: 33\n",
          Qos());
      }),
    Case<mapviz_plugins::RobotImagePlugin>(
      "robot_image", [] {
        return std::string(
          "frame: base_link\nimage: ''\nwidth: 3.5\nheight: 2.5\n"
          "offset_x: 0.5\noffset_y: -0.5\nratio: custom\n");
      }),
    Case<mapviz_plugins::RobotModelPlugin>(
      "robot_model", [] {
        return std::string("source_type: 0\ntopic: /robot_description\nalpha: 0.25\n");
      }),
    Case<mapviz_plugins::RoutePlugin>(
      "route", [] {
        return Join(
          "topic: /route\n",
          "postopic: /route_position\n",
          "color: '#123456'\n",
          "poscolor: '#654321'\n",
          "draw_style: points\n",
          Qos("route"),
          Qos("position"));
      }),
    Case<mapviz_plugins::StringPlugin>(
      "string", [] {
        // #790: every setting was reset to its default on restart.
        return Join(
          "topic: /status\n",
          "font: " + Font() + "\n",
          "color: '#55aaff'\n",
          "anchor: bottom center\n",
          "units: percent\n",
          "offset_x: 11\n",
          "offset_y: 22\n",
          Qos());
      }),
    Case<mapviz_plugins::TexturedMarkerPlugin>(
      "textured_marker", [] {
        return "topic: /textured_markers\n" + Qos();
      }),
    Case<mapviz_plugins::TfFramePlugin>(
      "tf_frame", [] {
        return std::string(
          "frame: base_link\ncolor: '#123456'\ndraw_style: arrows\n"
          "position_tolerance: 2.5\nbuffer_size: 42\nstatic_arrow_sizes: true\n"
          "arrow_size: 33\n");
      })),
  [](const ::testing::TestParamInfo<RoundTripCase> & info) {return info.param.name;});

int main(int argc, char ** argv)
{
  // The plugins build QWidget based config panels in their constructors, so a
  // QApplication has to exist first.  The test runs headless; see the ENV set
  // on this target in CMakeLists.txt.
  QApplication app(argc, argv);
  rclcpp::init(argc, argv);
  g_node = std::make_shared<rclcpp::Node>("test_plugin_round_trip");
  g_canvas = std::make_unique<QOpenGLWidget>();

  testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();

  g_canvas.reset();
  g_node.reset();
  rclcpp::shutdown();
  return result;
}
