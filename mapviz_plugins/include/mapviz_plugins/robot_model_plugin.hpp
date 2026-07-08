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

#ifndef MAPVIZ_PLUGINS__ROBOT_MODEL_PLUGIN_HPP_
#define MAPVIZ_PLUGINS__ROBOT_MODEL_PLUGIN_HPP_

#include <mapviz/mapviz_plugin.hpp>

#include <QColor>
#include <QObject>
#include <QOpenGLContext>
#include <QOpenGLFunctions>
#include <QOffscreenSurface>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "ui_robot_model_config.h"

namespace mapviz_plugins {

class RobotModelPlugin : public mapviz::MapvizPlugin {
  Q_OBJECT

 public:
  // Holds a diffuse texture image plus its lazy-uploaded GL texture ID.
  // Shared via shared_ptr so the GL ID persists across Transform() rebuilds.
  struct TextureData {
    std::vector<uint8_t> rgb_data;  // RGB8, row-major, top-row first
    int width{0};
    int height{0};
    mutable uint32_t gl_id{0};  // 0 = not yet uploaded
  };

  struct LinkGeometry {
    struct MeshFace {
      std::array<tf2::Vector3, 3> verts;       // root_link frame
      std::array<float, 3> brightness;         // per-vertex Lambert [0,1]
      std::array<std::array<float, 2>, 3> uvs; // per-vertex UV (u, v); v=0 is image top
      bool use_texture{false};
      QColor color;  // flat color when !use_texture; tint when use_texture
    };

    // Common:
    QColor color;    // URDF material color (tints texture or fills solid geom)
    std::string name;

    // Mesh-only:
    std::vector<MeshFace> mesh_faces;
    std::shared_ptr<TextureData> texture_data;  // null if no texture
  };

  RobotModelPlugin();
  ~RobotModelPlugin() override = default;

  bool Initialize(QOpenGLWidget* canvas) override;
  void Shutdown() override;
  void Draw(double x, double y, double scale) override;
  void Transform() override;
  void LoadConfig(const YAML::Node& node, const std::string& path) override;
  void SaveConfig(YAML::Emitter& emitter, const std::string& path) override;
  QWidget* GetConfigWidget(QWidget* parent) override;

 protected:
  void PrintError(const std::string& message) override;
  void PrintInfo(const std::string& message) override;
  void PrintWarning(const std::string& message) override;

 protected Q_SLOTS:
  void SelectTopic();
  void TopicEdited();
  void DrawIcon();
  void SourceChanged(int index);
  void BrowseFile();
  void FileEdited();

 private:
  struct BakeResult {
    GLuint texture{0};
    double bbox_min_x{0}, bbox_max_x{0};
    double bbox_min_y{0}, bbox_max_y{0};
    double baked_scale{0};
    bool was_offscreen{false};
    bool was_at_cap{false};
    bool ready{false};
  };

  void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg);
  void parseUrdf(const std::string& xml);
  void rebakeRaster(const std::vector<LinkGeometry>& geoms, double scale,
                    int canvas_max_dim, bool off_screen);

  Ui::robot_model_config ui_;
  QWidget* config_widget_;

  std::string topic_;
  std::string file_path_;
  std::string root_link_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_sub_;

  mutable std::mutex geometry_mutex_;
  std::vector<LinkGeometry> geometries_;  // root_link frame, built at parse time
  bool has_description_{false};

  // Display state (main thread only):
  GLuint display_texture_{0};
  double display_bbox_min_x_{0}, display_bbox_max_x_{0};
  double display_bbox_min_y_{0}, display_bbox_max_y_{0};
  double baked_scale_{0};
  bool baked_offscreen_{false};
  bool baked_at_cap_{false};
  std::array<std::pair<double, double>, 4> drawn_quad_corners_;
  bool drawn_quad_valid_{false};

  // Background bake coordination:
  std::mutex bake_result_mutex_;
  BakeResult pending_bake_;
  std::atomic<bool> bake_in_progress_{false};
  std::atomic<bool> urdf_dirty_{true};
  std::thread bake_thread_;

  QOpenGLContext* main_gl_context_{nullptr};
  QOffscreenSurface* offscreen_surface_{nullptr};
};

}  // namespace mapviz_plugins

#endif  // MAPVIZ_PLUGINS__ROBOT_MODEL_PLUGIN_HPP_
