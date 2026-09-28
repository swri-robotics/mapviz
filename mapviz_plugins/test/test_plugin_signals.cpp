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
#include <QOpenGLWidget>
#include <QtGlobal>
#include <yaml-cpp/yaml.h>

#include <functional>
#include <memory>
#include <ostream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <mapviz_plugins/attitude_indicator_plugin.hpp>
#include <mapviz_plugins/coordinate_picker_plugin.hpp>
#include <mapviz_plugins/disparity_plugin.hpp>
#include <mapviz_plugins/draw_marker_plugin.hpp>
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
#include <mapviz_plugins/speedometer_plugin.hpp>
#include <mapviz_plugins/string_plugin.hpp>
#include <mapviz_plugins/textured_marker_plugin.hpp>
#include <mapviz_plugins/tf_frame_plugin.hpp>

namespace
{
rclcpp::Node::SharedPtr g_node;

/// Stands in for the map canvas.  It is never shown, so no OpenGL context is
/// needed; plugins only ask it to repaint.
std::unique_ptr<QOpenGLWidget> g_canvas;

/// Some plugins use the canvas while loading their config.  Mapviz provides
/// it through Initialize(), which also needs an OpenGL context, so hand it
/// over directly.
template<typename PluginT>
class WithCanvas : public PluginT
{
public:
  // Qualified, because some plugins declare a canvas_ of their own.
  WithCanvas() {this->mapviz::MapvizPlugin::canvas_ = g_canvas.get();}
};

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

struct SignalCase
{
  std::string name;
  std::function<std::unique_ptr<mapviz::MapvizPlugin>()> create;
};

template<typename PluginT>
SignalCase Case(const std::string & name)
{
  return {name, [] {return std::make_unique<WithCanvas<PluginT>>();}};
}

void PrintTo(const SignalCase & signal_case, std::ostream * os)
{
  *os << signal_case.name;
}
}  // namespace

class SignalConnections : public ::testing::TestWithParam<SignalCase>
{
protected:
  void SetUp() override
  {
    g_connect_warnings.clear();
    previous_handler_ = qInstallMessageHandler(CollectConnectWarnings);
  }

  void TearDown() override {qInstallMessageHandler(previous_handler_);}

  QtMessageHandler previous_handler_ = nullptr;
};

TEST_P(SignalConnections, ConnectsEverySignal)
{
  // A signal that doesn't exist, such as QComboBox::activated(QString), which
  // Qt 6 removed, leaves its control doing nothing when it is used.
  std::unique_ptr<mapviz::MapvizPlugin> plugin = GetParam().create();
  plugin->SetNode(*g_node);
  plugin->LoadConfigPlugin(YAML::Node(YAML::NodeType::Map), "");

  for (const std::string & warning : g_connect_warnings) {
    ADD_FAILURE() << warning;
  }
}

INSTANTIATE_TEST_SUITE_P(
  Plugins, SignalConnections,
  ::testing::Values(
    Case<mapviz_plugins::AttitudeIndicatorPlugin>("attitude_indicator"),
    Case<mapviz_plugins::CoordinatePickerPlugin>("coordinate_picker"),
    Case<mapviz_plugins::DisparityPlugin>("disparity"),
    Case<mapviz_plugins::DrawMarkerPlugin>("draw_marker"),
    Case<mapviz_plugins::DrawPolygonPlugin>("draw_polygon"),
    Case<mapviz_plugins::FloatPlugin>("float"),
    Case<mapviz_plugins::GpsPlugin>("gps"),
    Case<mapviz_plugins::GridPlugin>("grid"),
    Case<mapviz_plugins::ImagePlugin>("image"),
    Case<mapviz_plugins::LaserScanPlugin>("laserscan"),
    Case<mapviz_plugins::MarkerPlugin>("marker"),
    Case<mapviz_plugins::MeasuringPlugin>("measuring"),
    Case<mapviz_plugins::NavSatPlugin>("navsat"),
    Case<mapviz_plugins::OccupancyGridPlugin>("occupancy_grid"),
    Case<mapviz_plugins::OdometryPlugin>("odometry"),
    Case<mapviz_plugins::PathPlugin>("path"),
    Case<mapviz_plugins::PlanRoutePlugin>("plan_route"),
    Case<mapviz_plugins::PointClickPublisherPlugin>("point_click_publisher"),
    Case<mapviz_plugins::PointCloud2Plugin>("pointcloud2"),
    Case<mapviz_plugins::PosePlugin>("pose"),
    Case<mapviz_plugins::RobotImagePlugin>("robot_image"),
    Case<mapviz_plugins::RobotModelPlugin>("robot_model"),
    Case<mapviz_plugins::RoutePlugin>("route"),
    Case<mapviz_plugins::SpeedometerPlugin>("speedometer"),
    Case<mapviz_plugins::StringPlugin>("string"),
    Case<mapviz_plugins::TexturedMarkerPlugin>("textured_marker"),
    Case<mapviz_plugins::TfFramePlugin>("tf_frame")),
  [](const ::testing::TestParamInfo<SignalCase> & info) {return info.param.name;});

int main(int argc, char ** argv)
{
  // The plugins build QWidget based config panels in their constructors, so a
  // QApplication has to exist first.  The test runs headless; see the ENV set
  // on this target in CMakeLists.txt.
  QApplication app(argc, argv);
  rclcpp::init(argc, argv);
  g_node = std::make_shared<rclcpp::Node>("test_plugin_signals");
  g_canvas = std::make_unique<QOpenGLWidget>();

  testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();

  g_canvas.reset();
  g_node.reset();
  rclcpp::shutdown();
  return result;
}
