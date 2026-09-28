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
#include <QCoreApplication>

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <marti_nav_msgs/msg/route.hpp>
#include <marti_nav_msgs/srv/plan_route.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mapviz_plugins/plan_route_plugin.hpp>

using marti_nav_msgs::srv::PlanRoute;

namespace
{
rclcpp::Node::SharedPtr g_node;

/// Stands in for a route planner: records each request and answers with a
/// fixed two-point route, or with a failure.
class MockPlanner
{
public:
  MockPlanner(const std::string & service, bool succeed)
  : succeed_(succeed)
  {
    service_ = g_node->create_service<PlanRoute>(
      service,
      [this](
        const std::shared_ptr<PlanRoute::Request> request,
        std::shared_ptr<PlanRoute::Response> response)
      {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          requests_.push_back(*request);
        }
        response->success = succeed_;
        if (!succeed_) {
          response->message = "No route between the waypoints.";
          return;
        }
        response->route.header.frame_id = "wgs84";
        for (const char * id : {"start", "end"}) {
          marti_nav_msgs::msg::RoutePoint point;
          point.id = id;
          point.pose.orientation.w = 1.0;
          response->route.route_points.push_back(point);
        }
      });
  }

  std::vector<PlanRoute::Request> Requests()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return requests_;
  }

private:
  const bool succeed_;
  rclcpp::Service<PlanRoute>::SharedPtr service_;
  std::mutex mutex_;
  std::vector<PlanRoute::Request> requests_;
};

std::vector<geometry_msgs::msg::Pose> Waypoints(size_t count)
{
  std::vector<geometry_msgs::msg::Pose> waypoints(count);
  for (size_t i = 0; i < count; i++) {
    waypoints[i].position.x = -98.6 + 0.001 * static_cast<double>(i);
    waypoints[i].position.y = 29.45;
    waypoints[i].orientation.w = 1.0;
  }
  return waypoints;
}

/// Service responses reach the plugin through the ROS spin thread and a
/// queued Qt signal, so keep the Qt event loop turning while waiting.
bool WaitFor(const std::function<bool()> & condition)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!condition()) {
    if (std::chrono::steady_clock::now() > deadline) {
      return false;
    }
    QCoreApplication::processEvents();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return true;
}
}  // namespace

namespace mapviz_plugins
{
class PlanRoutePluginTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Mapviz spins its node on a background thread; do the same so the mock
    // planner and the plugin's service client both get serviced.
    executor_.add_node(g_node);
    spin_thread_ = std::thread([this]() {executor_.spin();});
    plugin_.SetNode(*g_node);
  }

  void TearDown() override
  {
    executor_.cancel();
    spin_thread_.join();
    executor_.remove_node(g_node);
  }

  void SetService(const std::string & service) {plugin_.ui_.service->setText(service.c_str());}
  void SetTopic(const std::string & topic) {plugin_.ui_.topic->setText(topic.c_str());}
  void SetStartFromVehicle(bool start) {plugin_.ui_.start_from_vehicle->setChecked(start);}

  /// Places waypoints as if they had been clicked, then plans.
  void Plan(const std::vector<geometry_msgs::msg::Pose> & waypoints)
  {
    plugin_.waypoints_ = waypoints;
    plugin_.PlanRoute();
  }

  void Publish() {plugin_.PublishRoute();}

  bool HasPreview() const {return static_cast<bool>(plugin_.route_preview_);}

  std::string Status() const {return plugin_.ui_.status->text().toStdString();}

  /// Plans with @p waypoints and waits for the planner's route.
  ::testing::AssertionResult PlanAndWait(const std::vector<geometry_msgs::msg::Pose> & waypoints)
  {
    Plan(waypoints);
    if (!WaitFor([this]() {return HasPreview();})) {
      return ::testing::AssertionFailure() << "no route; status: " << Status();
    }
    return ::testing::AssertionSuccess();
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
  PlanRoutePlugin plugin_;
};
}  // namespace mapviz_plugins

using mapviz_plugins::PlanRoutePluginTest;

TEST_F(PlanRoutePluginTest, PlansThroughTheService)
{
  MockPlanner planner("/test_plan_route/plans", true);
  SetService("/test_plan_route/plans");

  ASSERT_TRUE(PlanAndWait(Waypoints(3)));

  const std::vector<PlanRoute::Request> requests = planner.Requests();
  ASSERT_EQ(1u, requests.size());
  EXPECT_EQ(3u, requests[0].waypoints.size());
  EXPECT_FALSE(requests[0].plan_from_vehicle);
  EXPECT_EQ("OK", Status());
}

TEST_F(PlanRoutePluginTest, PlansFromTheVehicleWithOneWaypoint)
{
  MockPlanner planner("/test_plan_route/from_vehicle", true);
  SetService("/test_plan_route/from_vehicle");
  SetStartFromVehicle(true);

  // The vehicle's position is the other end of the route.
  ASSERT_TRUE(PlanAndWait(Waypoints(1)));

  const std::vector<PlanRoute::Request> requests = planner.Requests();
  ASSERT_EQ(1u, requests.size());
  EXPECT_EQ(1u, requests[0].waypoints.size());
  EXPECT_TRUE(requests[0].plan_from_vehicle);
}

TEST_F(PlanRoutePluginTest, DoesNotPlanWithOneWaypoint)
{
  MockPlanner planner("/test_plan_route/one_waypoint", true);
  SetService("/test_plan_route/one_waypoint");

  Plan(Waypoints(1));

  EXPECT_FALSE(WaitFor([&planner]() {return !planner.Requests().empty();}));
  EXPECT_FALSE(HasPreview());
}

TEST_F(PlanRoutePluginTest, PublishesThePlannedRoute)
{
  MockPlanner planner("/test_plan_route/publishes", true);
  SetService("/test_plan_route/publishes");
  ASSERT_TRUE(PlanAndWait(Waypoints(2)));

  std::mutex mutex;
  std::vector<marti_nav_msgs::msg::Route> received;
  auto subscription = g_node->create_subscription<marti_nav_msgs::msg::Route>(
    "/test_plan_route/route", 10,
    [&](marti_nav_msgs::msg::Route::ConstSharedPtr route) {
      std::lock_guard<std::mutex> lock(mutex);
      received.push_back(*route);
    });

  SetTopic("/test_plan_route/route");
  Publish();

  ASSERT_TRUE(
    WaitFor(
      [&]() {
        std::lock_guard<std::mutex> lock(mutex);
        return !received.empty();
      }));
  std::lock_guard<std::mutex> lock(mutex);
  ASSERT_EQ(2u, received[0].route_points.size());
  EXPECT_EQ("start", received[0].route_points[0].id);
  EXPECT_EQ("end", received[0].route_points[1].id);
}

TEST_F(PlanRoutePluginTest, RefusesToPublishWithoutATopic)
{
  MockPlanner planner("/test_plan_route/no_topic", true);
  SetService("/test_plan_route/no_topic");
  ASSERT_TRUE(PlanAndWait(Waypoints(2)));

  // An empty topic used to skip creating the publisher and then publish
  // through a null pointer (#918).
  SetTopic("");
  EXPECT_NO_THROW(Publish());
  EXPECT_EQ("Route topic may not be empty.", Status());

  SetTopic("   ");
  EXPECT_NO_THROW(Publish());
  EXPECT_EQ("Route topic may not be empty.", Status());
}

TEST_F(PlanRoutePluginTest, ReportsAnUnavailableService)
{
  SetService("/test_plan_route/nobody_home");

  Plan(Waypoints(2));

  EXPECT_EQ("Service is unavailable.", Status());
  EXPECT_FALSE(HasPreview());
}

TEST_F(PlanRoutePluginTest, ReportsAFailedPlan)
{
  MockPlanner planner("/test_plan_route/fails", false);
  SetService("/test_plan_route/fails");

  Plan(Waypoints(2));

  ASSERT_TRUE(WaitFor([this]() {return Status() == "No route between the waypoints.";}))
    << "status: " << Status();
  EXPECT_FALSE(HasPreview());

  // Nothing to publish, so publishing does nothing.
  SetTopic("/test_plan_route/failed_route");
  EXPECT_NO_THROW(Publish());
  EXPECT_EQ(0u, g_node->count_publishers("/test_plan_route/failed_route"));
}

TEST_F(PlanRoutePluginTest, ReportsARejectedServiceName)
{
  SetService("/test_plan_route/not a service");

  EXPECT_NO_THROW(Plan(Waypoints(2)));

  EXPECT_EQ(0u, Status().rfind("Invalid service name", 0)) << "status: " << Status();
  EXPECT_FALSE(HasPreview());
}

TEST_F(PlanRoutePluginTest, ReportsARejectedRouteTopic)
{
  MockPlanner planner("/test_plan_route/rejected_topic", true);
  SetService("/test_plan_route/rejected_topic");
  ASSERT_TRUE(PlanAndWait(Waypoints(2)));

  SetTopic("/test_plan_route/not a topic");
  EXPECT_NO_THROW(Publish());

  EXPECT_EQ(0u, Status().rfind("Invalid topic name", 0)) << "status: " << Status();
}

int main(int argc, char ** argv)
{
  // The plugin builds a QWidget based config panel in its constructor, so a
  // QApplication has to exist first.  The test runs headless; see the ENV set
  // on this target in CMakeLists.txt.
  QApplication app(argc, argv);
  rclcpp::init(argc, argv);
  g_node = std::make_shared<rclcpp::Node>("test_plan_route");

  testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();

  g_node.reset();
  rclcpp::shutdown();
  return result;
}
