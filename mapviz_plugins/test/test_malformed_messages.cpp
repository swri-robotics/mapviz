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

#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include "swri_transform_util/transform_manager.h"
#include <tf2_ros/buffer.hpp>
#include <mapviz_plugins/occupancy_grid_plugin.hpp>
#include <mapviz_plugins/pointcloud2_plugin.hpp>

using map_msgs::msg::OccupancyGridUpdate;
using nav_msgs::msg::OccupancyGrid;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

namespace
{
rclcpp::Node::SharedPtr g_node;

OccupancyGrid::ConstSharedPtr Grid(uint32_t width, uint32_t height, size_t data_size)
{
  auto grid = std::make_shared<OccupancyGrid>();
  grid->header.frame_id = "map";
  grid->info.width = width;
  grid->info.height = height;
  grid->info.resolution = 1.0;
  grid->data.resize(data_size, 0);
  return grid;
}

OccupancyGrid::ConstSharedPtr Grid(uint32_t width, uint32_t height)
{
  return Grid(width, height, static_cast<size_t>(width) * height);
}

OccupancyGridUpdate::ConstSharedPtr Update(
  int32_t x, int32_t y, uint32_t width, uint32_t height, size_t data_size)
{
  auto update = std::make_shared<OccupancyGridUpdate>();
  update->x = x;
  update->y = y;
  update->width = width;
  update->height = height;
  update->data.resize(data_size, 100);
  return update;
}

OccupancyGridUpdate::ConstSharedPtr Update(int32_t x, int32_t y, uint32_t width, uint32_t height)
{
  return Update(x, y, width, height, static_cast<size_t>(width) * height);
}

PointField Field(const std::string & name, uint32_t offset)
{
  PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = PointField::FLOAT32;
  field.count = 1;
  return field;
}

/// A cloud of @p count points, each x, y, z and intensity as floats.
std::shared_ptr<PointCloud2> Cloud(size_t count)
{
  auto cloud = std::make_shared<PointCloud2>();
  cloud->header.frame_id = "map";
  cloud->height = 1;
  cloud->width = static_cast<uint32_t>(count);
  cloud->fields = {Field("x", 0), Field("y", 4), Field("z", 8), Field("intensity", 12)};
  cloud->point_step = 16;
  cloud->row_step = cloud->point_step * cloud->width;
  cloud->data.resize(cloud->row_step);
  for (size_t i = 0; i < count; i++) {
    const float values[] = {
      static_cast<float>(i), 2.0f * static_cast<float>(i), 3.0f, 0.5f};
    std::memcpy(&cloud->data[i * cloud->point_step], values, sizeof(values));
  }
  return cloud;
}
}  // namespace

namespace mapviz_plugins
{
class OccupancyGridPluginTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    plugin_.SetNode(*g_node);
    // Receiving a grid looks up its transform, which Mapviz normally makes
    // possible through Initialize().  That also needs an OpenGL context, so
    // hand over a transform manager directly.
    plugin_.tf_manager_ = std::make_shared<swri_transform_util::TransformManager>(
      g_node, std::make_shared<tf2_ros::Buffer>(g_node->get_clock()));
  }

  void Receive(const OccupancyGrid::ConstSharedPtr & grid) {plugin_.handleGrid(grid);}
  void Receive(const OccupancyGridUpdate::ConstSharedPtr & update)
  {
    plugin_.handleGridUpdate(update);
  }

  bool HasGrid() const {return static_cast<bool>(plugin_.grid_);}

  /// The value of a cell as it will be drawn.
  int Cell(size_t col, size_t row) const
  {
    return plugin_.raw_buffer_[col + row * plugin_.texture_size_];
  }

  OccupancyGridPlugin plugin_;
};

class PointCloud2PluginTest : public ::testing::Test
{
protected:
  static PointCloud2Plugin::Scan Decode(const PointCloud2::ConstSharedPtr & cloud)
  {
    return PointCloud2Plugin::DecodeScan(cloud);
  }

  /// handleScan() drops a scan without features.
  static bool Dropped(const PointCloud2Plugin::Scan & scan) {return scan.new_features.empty();}
};
}  // namespace mapviz_plugins

using mapviz_plugins::OccupancyGridPluginTest;
using mapviz_plugins::PointCloud2PluginTest;

TEST_F(OccupancyGridPluginTest, DrawsAGrid)
{
  auto grid = std::make_shared<OccupancyGrid>(*Grid(4, 3));
  grid->data[1 + 2 * 4] = 100;
  Receive(grid);

  ASSERT_TRUE(HasGrid());
  EXPECT_EQ(100, Cell(1, 2));
  EXPECT_EQ(0, Cell(2, 1));
}

TEST_F(OccupancyGridPluginTest, RejectsAGridWithTooLittleData)
{
  // Reading the missing cells runs off the end of the message's data.
  Receive(Grid(4000, 4000, 0));

  EXPECT_FALSE(HasGrid());
}

TEST_F(OccupancyGridPluginTest, KeepsTheLastGoodGridWhenAGridIsRejected)
{
  Receive(Grid(4, 4));
  Receive(Grid(4000, 4000, 0));

  ASSERT_TRUE(HasGrid());
  EXPECT_EQ(0, Cell(3, 3));
}

TEST_F(OccupancyGridPluginTest, AppliesAnUpdate)
{
  Receive(Grid(4, 4));
  Receive(Update(1, 1, 2, 2));

  EXPECT_EQ(0, Cell(0, 0));
  EXPECT_EQ(100, Cell(1, 1));
  EXPECT_EQ(100, Cell(2, 2));
  EXPECT_EQ(0, Cell(3, 3));
}

TEST_F(OccupancyGridPluginTest, IgnoresAnUpdateBeforeAnyGrid)
{
  EXPECT_NO_THROW(Receive(Update(0, 0, 2, 2)));
  EXPECT_FALSE(HasGrid());
}

TEST_F(OccupancyGridPluginTest, RejectsAnUpdateThatRunsOffTheGrid)
{
  Receive(Grid(4, 4));

  // Columns past the edge of the grid used to wrap onto the next row.
  Receive(Update(2, 0, 4, 1));

  EXPECT_EQ(0, Cell(2, 0));
  EXPECT_EQ(0, Cell(0, 1));
  EXPECT_EQ(0, Cell(1, 1));
}

TEST_F(OccupancyGridPluginTest, RejectsAnUpdateOutsideTheGrid)
{
  Receive(Grid(4, 4));

  // Both used to write far outside the texture.
  Receive(Update(1000000, 0, 1, 1));
  Receive(Update(-1, -1, 1, 1));

  EXPECT_EQ(0, Cell(0, 0));
}

TEST_F(OccupancyGridPluginTest, RejectsAnUpdateWithTooLittleData)
{
  Receive(Grid(4, 4));

  // Reading the missing cells runs off the end of the update's data.
  Receive(Update(0, 0, 4, 4, 0));

  EXPECT_EQ(0, Cell(0, 0));
}

TEST_F(PointCloud2PluginTest, DecodesACloud)
{
  const mapviz_plugins::PointCloud2Plugin::Scan scan = Decode(Cloud(3));

  ASSERT_FALSE(Dropped(scan));
  ASSERT_EQ(3u, scan.points.size());
  EXPECT_FLOAT_EQ(2.0f, scan.points[2].point.x());
  EXPECT_FLOAT_EQ(4.0f, scan.points[2].point.y());
  EXPECT_FLOAT_EQ(3.0f, scan.points[2].point.z());
  EXPECT_EQ(4u, scan.new_features.size());
}

TEST_F(PointCloud2PluginTest, IgnoresATrailingPartialPoint)
{
  auto cloud = Cloud(2);
  cloud->data.resize(cloud->data.size() + 5);

  const mapviz_plugins::PointCloud2Plugin::Scan scan = Decode(cloud);

  ASSERT_FALSE(Dropped(scan));
  EXPECT_EQ(2u, scan.points.size());
}

TEST_F(PointCloud2PluginTest, DropsACloudWithoutCoordinates)
{
  auto cloud = Cloud(2);
  cloud->fields.erase(cloud->fields.begin());

  EXPECT_TRUE(Dropped(Decode(cloud)));
}

TEST_F(PointCloud2PluginTest, DropsACloudWithAZeroPointStep)
{
  // Used to divide by zero while counting the points.
  auto cloud = Cloud(2);
  cloud->point_step = 0;

  EXPECT_TRUE(Dropped(Decode(cloud)));
}

TEST_F(PointCloud2PluginTest, DropsACloudWithACoordinatePastThePoint)
{
  // Used to read far past the end of the cloud's data.
  auto cloud = Cloud(2);
  cloud->fields[2].offset = 1000000;

  EXPECT_TRUE(Dropped(Decode(cloud)));
}

TEST_F(PointCloud2PluginTest, DropsACloudWithAFieldThatRunsOffThePoint)
{
  // The last point's intensity used to be read from past the end of the
  // cloud's data.
  auto cloud = Cloud(2);
  cloud->fields[3].offset = 14;

  EXPECT_TRUE(Dropped(Decode(cloud)));
}

int main(int argc, char ** argv)
{
  // The plugins build QWidget based config panels in their constructors, so a
  // QApplication has to exist first.  The test runs headless; see the ENV set
  // on this target in CMakeLists.txt.
  QApplication app(argc, argv);
  rclcpp::init(argc, argv);
  g_node = std::make_shared<rclcpp::Node>("test_malformed_messages");

  testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();

  g_node.reset();
  rclcpp::shutdown();
  return result;
}
