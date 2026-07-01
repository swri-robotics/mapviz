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

#include <mapviz_plugins/robot_model_plugin.hpp>
#include <mapviz_plugins/topic_select.hpp>

#include <QColor>
#include <QFileDialog>
#include <QPainter>
#include <QPen>
#include <QPixmap>


#include <GL/gl.h>
#include <QOpenGLContext>
#include <QOpenGLFunctions>
#include <QOffscreenSurface>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>

#include <urdf/model.h>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <string>
#include <utility>
#include <vector>

PLUGINLIB_EXPORT_CLASS(mapviz_plugins::RobotModelPlugin,
                       mapviz::MapvizPlugin)

namespace mapviz_plugins {

namespace {

std::string resolveUri(const std::string& uri)
{
  if (uri.substr(0, 10) == "package://") {
    const std::string no_scheme = uri.substr(10);
    const size_t slash = no_scheme.find('/');
    if (slash == std::string::npos) {
      return {};
    }
    const std::string pkg = no_scheme.substr(0, slash);
    const std::string rel = no_scheme.substr(slash + 1);
    try {
      return ament_index_cpp::get_package_share_directory(pkg) + "/" + rel;
    } catch (...) {
      return {};
    }
  }
  if (uri.substr(0, 7) == "file://") {
    return uri.substr(7);
  }
  return uri;
}

struct FaceData {
  std::array<tf2::Vector3, 3> verts;
  std::array<float, 3> brightness;
  std::array<std::array<float, 2>, 3> uvs;
  bool use_texture{false};
  QColor color;
};

struct LoadedMesh {
  std::vector<FaceData> faces;
  QColor base_color;
  cv::Mat texture;  // BGR, from first textured submesh; empty if none
};

LoadedMesh loadMesh(const std::string& filename,
                    const urdf::Vector3& mesh_scale,
                    const tf2::Transform& transform)
{
  const std::string path = resolveUri(filename);
  if (path.empty()) return {};

  Assimp::Importer importer;
  // Strip any per-face normals that came from the file so GenSmoothNormals
  // always regenerates them after JoinIdenticalVertices connects the topology.
  // Without this, GenSmoothNormals is a no-op (skipped when normals exist),
  // leaving flat per-triangle shading visible on curved surfaces in 2D view.
  importer.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS, aiComponent_NORMALS);
  importer.SetPropertyFloat(AI_CONFIG_PP_GSN_MAX_SMOOTHING_ANGLE, 60.0f);
  const aiScene* scene = importer.ReadFile(
      path, aiProcess_RemoveComponent | aiProcess_Triangulate |
                aiProcess_JoinIdenticalVertices | aiProcess_GenSmoothNormals |
                aiProcess_SortByPType | aiProcess_GenUVCoords |
                aiProcess_FlipUVs);
  if (!scene || scene->mNumMeshes == 0) return {};

  struct SubmeshInfo {
    unsigned int vert_base;
    std::vector<std::array<unsigned int, 3>> tris;
    QColor color;
    cv::Mat texture;                       // loaded diffuse texture (empty if none)
    std::vector<std::array<float, 2>> uvs; // per-vertex UV, indexed by (tris[k] - vert_base)
  };

  std::vector<tf2::Vector3> all_verts;
  std::vector<tf2::Vector3> all_normals;
  std::vector<SubmeshInfo> submeshes;

  for (unsigned m = 0; m < scene->mNumMeshes; ++m) {
    const aiMesh* mesh = scene->mMeshes[m];
    SubmeshInfo si;
    si.vert_base = static_cast<unsigned int>(all_verts.size());

    if (mesh->mMaterialIndex < scene->mNumMaterials) {
      const aiMaterial* mat = scene->mMaterials[mesh->mMaterialIndex];
      // Load diffuse texture and scalar color independently
      aiString texPath;
      if (mat->GetTexture(aiTextureType_DIFFUSE, 0, &texPath) == AI_SUCCESS &&
          texPath.length > 0 && mesh->mTextureCoords[0] != nullptr) {
        const std::string dir = path.substr(0, path.find_last_of("/\\") + 1);
        cv::Mat img = cv::imread(dir + texPath.data);
        if (!img.empty()) {
          if (img.channels() == 4) {
            cv::cvtColor(img, img, cv::COLOR_BGRA2BGR);
          }
          si.texture = std::move(img);
          si.uvs.resize(mesh->mNumVertices);
          for (unsigned v = 0; v < mesh->mNumVertices; ++v) {
            si.uvs[v] = {mesh->mTextureCoords[0][v].x,
                         mesh->mTextureCoords[0][v].y};
          }
        }
      }
      // Scalar diffuse; DAE exporters often leave alpha = 0 even for opaque
      // materials, so ignore the alpha channel.
      aiColor4D c(0.f, 0.f, 0.f, 0.f);
      if (AI_SUCCESS == mat->Get(AI_MATKEY_COLOR_DIFFUSE, c) &&
          (c.r > 0.f || c.g > 0.f || c.b > 0.f)) {
        si.color = QColor::fromRgbF(c.r, c.g, c.b);
      } else if (AI_SUCCESS == mat->Get(AI_MATKEY_BASE_COLOR, c) &&
                 (c.r > 0.f || c.g > 0.f || c.b > 0.f)) {
        si.color = QColor::fromRgbF(c.r, c.g, c.b);
      }
    }

    for (unsigned v = 0; v < mesh->mNumVertices; ++v) {
      const tf2::Vector3 vw = transform * tf2::Vector3(
          mesh->mVertices[v].x * mesh_scale.x,
          mesh->mVertices[v].y * mesh_scale.y,
          mesh->mVertices[v].z * mesh_scale.z);
      all_verts.push_back(vw);

      // Transform normal via inverse-transpose to handle non-uniform scale:
      // divide each component by its scale factor, rotate, then re-normalize.
      tf2::Vector3 nw(0.0, 0.0, 1.0);
      if (mesh->HasNormals()) {
        const tf2::Vector3 n_scaled(
            mesh->mNormals[v].x / mesh_scale.x,
            mesh->mNormals[v].y / mesh_scale.y,
            mesh->mNormals[v].z / mesh_scale.z);
        const tf2::Vector3 n_rot = transform.getBasis() * n_scaled;
        const double len = n_rot.length();
        nw = (len > 1e-10) ? n_rot / len : tf2::Vector3(0.0, 0.0, 1.0);
      }
      all_normals.push_back(nw);
    }
    for (unsigned f = 0; f < mesh->mNumFaces; ++f) {
      const aiFace& face = mesh->mFaces[f];
      if (face.mNumIndices == 3) {
        si.tris.push_back({si.vert_base + face.mIndices[0],
                           si.vert_base + face.mIndices[1],
                           si.vert_base + face.mIndices[2]});
      }
    }
    submeshes.push_back(std::move(si));
  }

  if (all_verts.empty()) return {};

  // Lambert diffuse from a top-down light: ambient=0.35, diffuse=0.65.
  auto vbright = [](const tf2::Vector3& n) -> float {
    return 0.35f + 0.65f * static_cast<float>(std::max(0.0, n.z()));
  };

  LoadedMesh result;

  for (const auto& si : submeshes) {
    const bool use_tex = !si.texture.empty() && !si.uvs.empty();
    if (use_tex && result.texture.empty()) {
      result.texture = si.texture;  // keep first textured submesh's image
    }
    for (const auto& tri : si.tris) {
      const tf2::Vector3& v0 = all_verts[tri[0]];
      const tf2::Vector3& v1 = all_verts[tri[1]];
      const tf2::Vector3& v2 = all_verts[tri[2]];
      // Cull downward-facing triangles for the top-down view.
      const double nz = (v1.x() - v0.x()) * (v2.y() - v0.y()) -
                        (v1.y() - v0.y()) * (v2.x() - v0.x());
      if (nz <= 0.0) continue;
      FaceData fd;
      fd.verts = {v0, v1, v2};
      fd.use_texture = use_tex;
      fd.color = si.color;
      for (int k = 0; k < 3; ++k) {
        fd.brightness[k] = vbright(all_normals[tri[k]]);
        if (use_tex) {
          fd.uvs[k] = si.uvs[tri[k] - si.vert_base];
        }
      }
      result.faces.push_back(fd);
    }
  }

  for (const auto& si : submeshes) {
    if (si.color.isValid() && si.texture.empty()) { result.base_color = si.color; break; }
  }

  return result;
}

tf2::Transform urdfPoseToTf2(const urdf::Pose& p) {
  return tf2::Transform(
      tf2::Quaternion(p.rotation.x, p.rotation.y, p.rotation.z, p.rotation.w),
      tf2::Vector3(p.position.x, p.position.y, p.position.z));
}

void traverseLink(const urdf::Model& model, const std::string& link_name,
                  const tf2::Transform& root_T_link,
                  std::vector<RobotModelPlugin::LinkGeometry>& out) {
  const auto link = model.getLink(link_name);
  if (!link) {
    return;
  }

  const size_t num_vis = link->visual_array.size();
  size_t vis_idx = 0;
  for (const auto& vis : link->visual_array) {
    if (!vis || !vis->geometry) {
      ++vis_idx;
      continue;
    }

    const tf2::Transform root_T_vis = root_T_link * urdfPoseToTf2(vis->origin);

    RobotModelPlugin::LinkGeometry geom;
    geom.name = (num_vis > 1)
        ? link_name + " [" + std::to_string(vis_idx) + "]"
        : link_name;
    ++vis_idx;
    if (vis->material && vis->material->color.a > 0.0f) {
      const auto& c = vis->material->color;
      geom.color = QColor::fromRgbF(
          static_cast<float>(c.r), static_cast<float>(c.g),
          static_cast<float>(c.b), static_cast<float>(c.a));
    }

    // Push a world-space triangle as a mesh face only if its projected normal faces up.
    auto pushIfUp = [](RobotModelPlugin::LinkGeometry& g,
                       const tf2::Vector3& v0, const tf2::Vector3& v1,
                       const tf2::Vector3& v2) {
      const double nz = (v1.x()-v0.x())*(v2.y()-v0.y())
                      - (v1.y()-v0.y())*(v2.x()-v0.x());
      if (nz <= 0.0) return;
      const tf2::Vector3 n = (v1 - v0).cross(v2 - v0);
      const double len = n.length();
      const float brt = (len > 1e-10)
          ? 0.35f + 0.65f * static_cast<float>(std::max(0.0, n.z() / len))
          : 0.35f;
      RobotModelPlugin::LinkGeometry::MeshFace mf;
      mf.verts       = {v0, v1, v2};
      mf.brightness  = {brt, brt, brt};
      mf.use_texture = false;
      mf.color       = g.color;
      g.mesh_faces.push_back(mf);
    };

    switch (vis->geometry->type) {
      case urdf::Geometry::BOX: {
        const auto* box = static_cast<const urdf::Box*>(vis->geometry.get());
        const double hx = box->dim.x / 2.0;
        const double hy = box->dim.y / 2.0;
        const double hz = box->dim.z / 2.0;
        // 6 faces as quads (CCW winding from outside), each split into 2 triangles.
        const std::array<std::array<tf2::Vector3, 4>, 6> faces = {{
            {{ tf2::Vector3( hx, hy, hz), tf2::Vector3(-hx, hy, hz), tf2::Vector3(-hx,-hy, hz), tf2::Vector3( hx,-hy, hz) }},  // +Z
            {{ tf2::Vector3( hx, hy,-hz), tf2::Vector3( hx,-hy,-hz), tf2::Vector3(-hx,-hy,-hz), tf2::Vector3(-hx, hy,-hz) }},  // -Z
            {{ tf2::Vector3( hx, hy, hz), tf2::Vector3( hx,-hy, hz), tf2::Vector3( hx,-hy,-hz), tf2::Vector3( hx, hy,-hz) }},  // +X
            {{ tf2::Vector3(-hx,-hy, hz), tf2::Vector3(-hx, hy, hz), tf2::Vector3(-hx, hy,-hz), tf2::Vector3(-hx,-hy,-hz) }},  // -X
            {{ tf2::Vector3(-hx, hy, hz), tf2::Vector3( hx, hy, hz), tf2::Vector3( hx, hy,-hz), tf2::Vector3(-hx, hy,-hz) }},  // +Y
            {{ tf2::Vector3( hx,-hy, hz), tf2::Vector3(-hx,-hy, hz), tf2::Vector3(-hx,-hy,-hz), tf2::Vector3( hx,-hy,-hz) }},  // -Y
        }};
        for (const auto& f : faces) {
          const tf2::Vector3 a = root_T_vis * f[0];
          const tf2::Vector3 b = root_T_vis * f[1];
          const tf2::Vector3 c = root_T_vis * f[2];
          const tf2::Vector3 d = root_T_vis * f[3];
          pushIfUp(geom, a, b, c);
          pushIfUp(geom, a, c, d);
        }
        if (!geom.mesh_faces.empty()) out.push_back(geom);
        break;
      }
      case urdf::Geometry::CYLINDER: {
        const auto* cyl = static_cast<const urdf::Cylinder*>(vis->geometry.get());
        const double r   = cyl->radius;
        const double hzc = cyl->length / 2.0;
        constexpr int kCylSides = 24;
        const tf2::Vector3 top_c = root_T_vis * tf2::Vector3(0.0, 0.0,  hzc);
        const tf2::Vector3 bot_c = root_T_vis * tf2::Vector3(0.0, 0.0, -hzc);
        for (int i = 0; i < kCylSides; ++i) {
          const double a0 = 2.0 * M_PI *  i      / kCylSides;
          const double a1 = 2.0 * M_PI * (i + 1) / kCylSides;
          const tf2::Vector3 t0 = root_T_vis * tf2::Vector3(r*std::cos(a0), r*std::sin(a0),  hzc);
          const tf2::Vector3 t1 = root_T_vis * tf2::Vector3(r*std::cos(a1), r*std::sin(a1),  hzc);
          const tf2::Vector3 b0 = root_T_vis * tf2::Vector3(r*std::cos(a0), r*std::sin(a0), -hzc);
          const tf2::Vector3 b1 = root_T_vis * tf2::Vector3(r*std::cos(a1), r*std::sin(a1), -hzc);
          pushIfUp(geom, top_c, t0, t1);  // top cap
          pushIfUp(geom, bot_c, b1, b0);  // bottom cap (reversed winding)
          pushIfUp(geom, t1, t0, b0);     // side quad tri 1 (CCW from outside)
          pushIfUp(geom, t1, b0, b1);     // side quad tri 2
        }
        if (!geom.mesh_faces.empty()) out.push_back(geom);
        break;
      }
      case urdf::Geometry::SPHERE: {
        const auto* sph = static_cast<const urdf::Sphere*>(vis->geometry.get());
        const double r = sph->radius;
        // Icosphere: subdivide an icosahedron 3× for uniform triangle coverage.
        const double phi = (1.0 + std::sqrt(5.0)) / 2.0;
        const double sn = 1.0 / std::sqrt(1.0 + phi * phi);
        const double sp = phi * sn;
        using V3 = std::array<double, 3>;
        const V3 ico[12] = {
            {-sn, sp, 0.0}, {sn, sp, 0.0},   {-sn, -sp, 0.0}, {sn, -sp, 0.0},
            {0.0, -sn, sp}, {0.0, sn, sp},   {0.0, -sn, -sp}, {0.0, sn, -sp},
            {sp, 0.0, -sn}, {sp, 0.0, sn},   {-sp, 0.0, -sn}, {-sp, 0.0, sn},
        };
        using Tri = std::array<V3, 3>;
        std::vector<Tri> ico_tris = {
            {ico[0], ico[11], ico[5]},  {ico[0], ico[5],  ico[1]},
            {ico[0], ico[1],  ico[7]},  {ico[0], ico[7],  ico[10]},
            {ico[0], ico[10], ico[11]}, {ico[1], ico[5],  ico[9]},
            {ico[5], ico[11], ico[4]},  {ico[11], ico[10], ico[2]},
            {ico[10], ico[7], ico[6]},  {ico[7], ico[1],  ico[8]},
            {ico[3], ico[9],  ico[4]},  {ico[3], ico[4],  ico[2]},
            {ico[3], ico[2],  ico[6]},  {ico[3], ico[6],  ico[8]},
            {ico[3], ico[8],  ico[9]},  {ico[4], ico[9],  ico[5]},
            {ico[2], ico[4],  ico[11]}, {ico[6], ico[2],  ico[10]},
            {ico[8], ico[6],  ico[7]},  {ico[9], ico[8],  ico[1]},
        };
        auto sph_mid = [](const V3& a, const V3& b) -> V3 {
          const double x = a[0] + b[0], y = a[1] + b[1], z = a[2] + b[2];
          const double m = std::sqrt(x * x + y * y + z * z);
          return {x / m, y / m, z / m};
        };
        for (int s = 0; s < 3; ++s) {
          std::vector<Tri> sub;
          sub.reserve(ico_tris.size() * 4);
          for (const auto& t : ico_tris) {
            const V3 m01 = sph_mid(t[0], t[1]);
            const V3 m12 = sph_mid(t[1], t[2]);
            const V3 m20 = sph_mid(t[2], t[0]);
            sub.push_back({t[0], m01, m20});
            sub.push_back({t[1], m12, m01});
            sub.push_back({t[2], m20, m12});
            sub.push_back({m01, m12, m20});
          }
          ico_tris = std::move(sub);
        }
        for (const auto& t : ico_tris) {
          pushIfUp(geom,
              root_T_vis * tf2::Vector3(r * t[0][0], r * t[0][1], r * t[0][2]),
              root_T_vis * tf2::Vector3(r * t[1][0], r * t[1][1], r * t[1][2]),
              root_T_vis * tf2::Vector3(r * t[2][0], r * t[2][1], r * t[2][2]));
        }
        if (!geom.mesh_faces.empty()) out.push_back(geom);
        break;
      }
      case urdf::Geometry::MESH: {
        const auto* mesh_geom =
            static_cast<const urdf::Mesh*>(vis->geometry.get());
        auto loaded = loadMesh(mesh_geom->filename, mesh_geom->scale, root_T_vis);
        if (!loaded.faces.empty()) {
          // Build TextureData (RGB8 for GL upload) from the loaded BGR image.
          if (!loaded.texture.empty()) {
            auto td = std::make_shared<RobotModelPlugin::TextureData>();
            td->width  = loaded.texture.cols;
            td->height = loaded.texture.rows;
            cv::Mat rgb;
            cv::cvtColor(loaded.texture, rgb, cv::COLOR_BGR2RGB);
            td->rgb_data.assign(rgb.data, rgb.data + rgb.total() * 3);
            geom.texture_data = std::move(td);
          }
          for (auto& fd : loaded.faces) {
            RobotModelPlugin::LinkGeometry::MeshFace mf;
            mf.verts = fd.verts;
            mf.brightness = fd.brightness;
            mf.use_texture = fd.use_texture;
            mf.uvs = fd.uvs;
            mf.color = geom.color.isValid() ? geom.color : fd.color;
            geom.mesh_faces.push_back(mf);
          }
          if (!geom.color.isValid() && loaded.base_color.isValid()) {
            geom.color = loaded.base_color;
          }
          out.push_back(geom);
        }
        break;
      }
      default:
        RCLCPP_WARN(rclcpp::get_logger("robot_model_plugin"),
                    "Link '%s': unsupported geometry type %d, will not be rendered.",
                    link_name.c_str(), static_cast<int>(vis->geometry->type));
        break;
    }
  }

  for (const auto& joint : link->child_joints) {
    if (!joint) {
      continue;
    }
    traverseLink(
        model, joint->child_link_name,
        root_T_link * urdfPoseToTf2(joint->parent_to_joint_origin_transform),
        out);
  }
}

}  // namespace

RobotModelPlugin::RobotModelPlugin()
    : MapvizPlugin(), ui_(), config_widget_(new QWidget()) {
  ui_.setupUi(config_widget_);

  QPalette p(config_widget_->palette());
  p.setColor(QPalette::Window, Qt::white);
  config_widget_->setPalette(p);

  QPalette ps(ui_.status->palette());
  ps.setColor(QPalette::Text, Qt::red);
  ui_.status->setPalette(ps);

  connect(ui_.source_combo, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &RobotModelPlugin::SourceChanged);
  connect(ui_.selecttopic, SIGNAL(clicked()), this, SLOT(SelectTopic()));
  connect(ui_.topic, SIGNAL(editingFinished()), this, SLOT(TopicEdited()));
  connect(ui_.browsefile, SIGNAL(clicked()), this, SLOT(BrowseFile()));
  connect(ui_.filepath, SIGNAL(editingFinished()), this, SLOT(FileEdited()));
}

bool RobotModelPlugin::Initialize(QOpenGLWidget* canvas) {
  canvas_ = canvas;
  initialized_ = true;
  main_gl_context_ = canvas->context();
  offscreen_surface_ = new QOffscreenSurface();
  offscreen_surface_->setFormat(main_gl_context_->format());
  offscreen_surface_->create();
  DrawIcon();
  return true;
}

void RobotModelPlugin::Shutdown() {
  if (bake_thread_.joinable()) bake_thread_.join();
  if (display_texture_) {
    glDeleteTextures(1, &display_texture_);
    display_texture_ = 0;
  }
  {
    std::lock_guard<std::mutex> lk(bake_result_mutex_);
    if (pending_bake_.texture) {
      glDeleteTextures(1, &pending_bake_.texture);
      pending_bake_.texture = 0;
    }
  }
  delete offscreen_surface_;
  offscreen_surface_ = nullptr;
}

QWidget* RobotModelPlugin::GetConfigWidget(QWidget* parent) {
  config_widget_->setParent(parent);
  return config_widget_;
}

void RobotModelPlugin::DrawIcon() {
  if (!icon_) {
    return;
  }
  QPixmap icon(16, 16);
  icon.fill(Qt::transparent);
  QPainter painter(&icon);
  painter.setRenderHint(QPainter::Antialiasing, true);

  const QColor color(30, 144, 255);
  painter.setBrush(color);
  painter.setPen(QPen(color.darker(150), 1));

  // top-down vehicle: rectangle body + triangular nose pointing up
  painter.drawRect(4, 6, 8, 7);
  QPolygon nose;
  nose << QPoint(8, 2) << QPoint(4, 7) << QPoint(12, 7);
  painter.drawPolygon(nose);

  icon_->SetPixmap(icon);
}

void RobotModelPlugin::SelectTopic() {
  auto [topic, qos] = SelectTopicDialog::selectTopic(
      node_, "std_msgs/msg/String", rmw_qos_profile_default);
  (void)qos;
  if (!topic.empty()) {
    ui_.topic->setText(QString::fromStdString(topic));
    TopicEdited();
  }
}

void RobotModelPlugin::SourceChanged(int index) {
  ui_.source_stack->setCurrentIndex(index);
  description_sub_.reset();
  {
    std::lock_guard<std::mutex> lock(geometry_mutex_);
    geometries_.clear();
    drawn_quad_valid_ = false;
    urdf_dirty_ = true;
    has_description_ = false;
  }
  topic_ = "";
  file_path_ = "";
  PrintWarning(index == 0 ? "No topic." : "No file selected.");
}

void RobotModelPlugin::BrowseFile() {
  const QString filename = QFileDialog::getOpenFileName(
      config_widget_, "Open URDF File",
      QString::fromStdString(file_path_),
      "URDF Files (*.urdf *.xml);;All Files (*)");
  if (!filename.isEmpty()) {
    ui_.filepath->setText(filename);
    FileEdited();
  }
}

void RobotModelPlugin::FileEdited() {
  const std::string path = ui_.filepath->text().trimmed().toStdString();
  if (path == file_path_) {
    return;
  }
  file_path_ = path;
  if (file_path_.empty()) {
    PrintWarning("No file selected.");
    return;
  }
  if (std::filesystem::path(file_path_).extension() == ".xacro") {
    const std::string out =
        std::filesystem::path(file_path_).replace_extension("").string();
    PrintError("Cannot load .xacro files directly");
    RCLCPP_ERROR(rclcpp::get_logger("robot_model_plugin"),
                 "Run: xacro %s > %s", file_path_.c_str(), out.c_str());
    return;
  }
  std::ifstream ifs(file_path_);
  if (!ifs) {
    PrintError("Cannot open file: " + file_path_);
    return;
  }
  const std::string xml(std::istreambuf_iterator<char>(ifs),
                        std::istreambuf_iterator<char>{});
  parseUrdf(xml);
}

void RobotModelPlugin::TopicEdited() {
  if (ui_.source_combo->currentIndex() != 0) {
    return;
  }
  const std::string topic = ui_.topic->text().trimmed().toStdString();
  if (topic == topic_) {
    return;
  }
  topic_ = topic;

  {
    std::lock_guard<std::mutex> lock(geometry_mutex_);
    geometries_.clear();
    drawn_quad_valid_ = false;
    urdf_dirty_ = true;
    has_description_ = false;
  }

  description_sub_.reset();

  if (topic_.empty() || !node_) {
    PrintWarning("No topic.");
    return;
  }

  // robot_description is latched: use transient_local so we receive the
  // last-published value immediately on subscribe.
  auto qos = rclcpp::QoS(1).transient_local().reliable();
  description_sub_ = node_->create_subscription<std_msgs::msg::String>(
      topic_, qos,
      std::bind(&RobotModelPlugin::robotDescriptionCallback, this,
                std::placeholders::_1));
  PrintWarning("Waiting for description on " + topic_);
}

void RobotModelPlugin::robotDescriptionCallback(
    const std_msgs::msg::String::SharedPtr msg) {
  parseUrdf(msg->data);
}

void RobotModelPlugin::parseUrdf(const std::string& xml) {
  urdf::Model model;
  if (!model.initString(xml)) {
    PrintError("Failed to parse URDF.");
    return;
  }

  // Always use the URDF's declared root link as the TF source frame.
  if (!model.root_link_) {
    PrintError("URDF has no root link.");
    return;
  }
  const std::string root = model.root_link_->name;

  std::vector<LinkGeometry> new_geometries;
  traverseLink(model, root, tf2::Transform::getIdentity(), new_geometries);

  {
    std::lock_guard<std::mutex> lock(geometry_mutex_);
    geometries_ = std::move(new_geometries);
    drawn_quad_valid_ = false;
    urdf_dirty_ = true;
    has_description_ = !geometries_.empty();
    source_frame_ = root;
    root_link_ = root;
  }

  if (has_description_) {
    PrintInfo("OK — " + std::to_string(geometries_.size()) +
              " shapes from \"" + root + "\"");
  } else {
    PrintWarning("Parsed URDF but no box/cylinder/sphere geometry found.");
  }
}

void RobotModelPlugin::Transform() {
  std::lock_guard<std::mutex> lock(geometry_mutex_);

  if (!has_description_ || display_texture_ == 0) {
    drawn_quad_valid_ = false;
    return;
  }

  swri_transform_util::Transform transform;
  const rclcpp::Time stamp(0, 0, RCL_ROS_TIME);  // latest available TF
  if (!GetTransform(source_frame_, stamp, transform)) {
    drawn_quad_valid_ = false;
    PrintError("No TF for \"" + source_frame_ + "\"");
    return;
  }

  // Transform the four bounding box corners from root_link frame to map frame.
  const tf2::Vector3 corners_local[4] = {
    {display_bbox_min_x_, display_bbox_min_y_, 0.0},
    {display_bbox_max_x_, display_bbox_min_y_, 0.0},
    {display_bbox_max_x_, display_bbox_max_y_, 0.0},
    {display_bbox_min_x_, display_bbox_max_y_, 0.0},
  };
  for (int i = 0; i < 4; ++i) {
    const tf2::Vector3 p = transform * corners_local[i];
    drawn_quad_corners_[i] = {p.x(), p.y()};
  }
  drawn_quad_valid_ = true;
}

void RobotModelPlugin::Draw(double x, double y, double scale) {

  GLuint tex_id = 0;
  std::array<std::pair<double, double>, 4> quad;
  double alpha = 0.0;

  {
    std::lock_guard<std::mutex> lock(geometry_mutex_);
    if (!has_description_) return;

    // Adopt pending bake if one is ready.
    {
      std::lock_guard<std::mutex> lk(bake_result_mutex_);
      if (pending_bake_.ready) {
        if (display_texture_) glDeleteTextures(1, &display_texture_);
        display_texture_ = pending_bake_.texture;
        display_bbox_min_x_ = pending_bake_.bbox_min_x;
        display_bbox_max_x_ = pending_bake_.bbox_max_x;
        display_bbox_min_y_ = pending_bake_.bbox_min_y;
        display_bbox_max_y_ = pending_bake_.bbox_max_y;
        baked_scale_ = pending_bake_.baked_scale;
        baked_offscreen_ = pending_bake_.was_offscreen;
        baked_at_cap_ = pending_bake_.was_at_cap;
        pending_bake_ = BakeResult{};
      }
    }

    // Trigger a background bake if the URDF changed or zoom shifted significantly.
    if (!bake_in_progress_) {
      // When already at the texture cap, zooming in further won't produce a
      // larger texture, suppress those rebakes. Still rebake on zoom-out (ratio
      // > 1.5) since that means we're wasting memory on an oversized texture.
      const bool zoom_changed = baked_scale_ > 0.0 &&
          (scale / baked_scale_ > 1.5 ||
           (!baked_at_cap_ && scale / baked_scale_ < 1.0 / 1.5));

      // Check if the quad is entirely off-screen.
      bool off_screen = false;
      if (drawn_quad_valid_) {
        const double vp_half_w = canvas_->width()  / 2.0 * scale;
        const double vp_half_h = canvas_->height() / 2.0 * scale;
        double q_min_x = drawn_quad_corners_[0].first,  q_max_x = q_min_x;
        double q_min_y = drawn_quad_corners_[0].second, q_max_y = q_min_y;
        for (int i = 1; i < 4; ++i) {
          q_min_x = std::min(q_min_x, drawn_quad_corners_[i].first);
          q_max_x = std::max(q_max_x, drawn_quad_corners_[i].first);
          q_min_y = std::min(q_min_y, drawn_quad_corners_[i].second);
          q_max_y = std::max(q_max_y, drawn_quad_corners_[i].second);
        }
        off_screen = (q_max_x < x - vp_half_w || q_min_x > x + vp_half_w ||
                      q_max_y < y - vp_half_h || q_min_y > y + vp_half_h);
      }

      // Fire on both off→on and on→off transitions:
      // - on to off: bake a thumbnail to free memory
      // - off to on: bake full-res (zoom_changed may not fire if only panning)
      const bool screen_state_changed = off_screen != baked_offscreen_;
      if (urdf_dirty_.exchange(false) || (zoom_changed && !off_screen) ||
          screen_state_changed) {
        if (bake_thread_.joinable()) bake_thread_.join();
        bake_in_progress_ = true;
        // Off-screen: bake a thumbnail so the texture stays resident,
        // but uses minimal memory. Scrolling back in triggers a full-res rebake.
        constexpr int kOffScreenMaxDim = 170;
        const int canvas_max_dim = off_screen
            ? kOffScreenMaxDim
            : std::max(canvas_->width(), canvas_->height());
        auto geoms_copy = geometries_;
        bake_thread_ = std::thread(
            [this, geoms = std::move(geoms_copy), scale, canvas_max_dim, off_screen]() {
              rebakeRaster(geoms, scale, canvas_max_dim, off_screen);
            });
      }
    }

    if (display_texture_ == 0 || !drawn_quad_valid_) return;

    tex_id = display_texture_;
    quad = drawn_quad_corners_;
    alpha = ui_.alpha->value();
  }

  glPushAttrib(GL_ENABLE_BIT | GL_CURRENT_BIT | GL_TEXTURE_BIT);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, tex_id);
  glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
  glColor4f(1.0f, 1.0f, 1.0f, static_cast<float>(alpha));

  // UV (0,0) = bottom-left of FBO = corners_local[0] = (bbox_min_x, bbox_min_y)
  glBegin(GL_TRIANGLE_FAN);
  glTexCoord2f(0.0f, 0.0f); glVertex2d(quad[0].first, quad[0].second);
  glTexCoord2f(1.0f, 0.0f); glVertex2d(quad[1].first, quad[1].second);
  glTexCoord2f(1.0f, 1.0f); glVertex2d(quad[2].first, quad[2].second);
  glTexCoord2f(0.0f, 1.0f); glVertex2d(quad[3].first, quad[3].second);
  glEnd();

  glBindTexture(GL_TEXTURE_2D, 0);
  glPopAttrib();
  PrintInfo("OK");
}


void RobotModelPlugin::rebakeRaster(
    const std::vector<LinkGeometry>& geoms, double scale, int canvas_max_dim,
    bool off_screen) {
  // Create a background GL context that shares textures with the main context.
  QOpenGLContext bg_ctx;
  bg_ctx.setShareContext(main_gl_context_);
  bg_ctx.setFormat(main_gl_context_->format());
  if (!bg_ctx.create() || !bg_ctx.makeCurrent(offscreen_surface_)) {
    RCLCPP_ERROR(rclcpp::get_logger("robot_model_plugin"),
                 "Background GL context setup failed");
    bake_in_progress_ = false;
    return;
  }
  QOpenGLFunctions* gf = bg_ctx.functions();

  // Compute XY bounding box of all faces in root_link frame.
  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();
  for (const auto& geom : geoms) {
    for (const auto& mf : geom.mesh_faces) {
      for (const auto& v : mf.verts) {
        min_x = std::min(min_x, v.x());
        max_x = std::max(max_x, v.x());
        min_y = std::min(min_y, v.y());
        max_y = std::max(max_y, v.y());
      }
    }
  }
  if (min_x >= max_x || min_y >= max_y) {
    bg_ctx.doneCurrent();
    bake_in_progress_ = false;
    return;
  }

  // Small margin so geometry doesn't clip at the texture edge.
  const double mx = (max_x - min_x) * 0.02;
  const double my = (max_y - min_y) * 0.02;
  min_x -= mx; max_x += mx;
  min_y -= my; max_y += my;

  // Texture size: scale to current zoom, capped at 1.5× canvas max dimension.
  const double bb_w = max_x - min_x;
  const double bb_h = max_y - min_y;
  const double bb_long = std::max(bb_w, bb_h);
  const int max_tex = static_cast<int>(1.5 * canvas_max_dim);
  const int tex_size = std::max(1, std::min(
      static_cast<int>(bb_long / scale * 1.5), max_tex));
  int tex_w, tex_h;
  if (bb_w >= bb_h) {
    tex_w = tex_size;
    tex_h = std::max(1, static_cast<int>(tex_size * bb_h / bb_w));
  } else {
    tex_h = tex_size;
    tex_w = std::max(1, static_cast<int>(tex_size * bb_w / bb_h));
  }

  const double mb = static_cast<double>(tex_w) * tex_h * 4 / (1024.0 * 1024.0);
  const std::string model_name = geoms.empty() ? "?" : geoms.front().name;
  RCLCPP_DEBUG(rclcpp::get_logger("robot_model_plugin"),
               "FBO baking [%s]: %d x %d px (%.2f MB) at scale %.4f m/px",
               model_name.c_str(), tex_w, tex_h, mb, scale);

  // Upload any pending DAE textures into the shared context.
  for (const auto& geom : geoms) {
    if (geom.texture_data && geom.texture_data->gl_id == 0) {
      GLuint tid = 0;
      glGenTextures(1, &tid);
      glBindTexture(GL_TEXTURE_2D, tid);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_TRUE);
      glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
                   geom.texture_data->width, geom.texture_data->height,
                   0, GL_RGB, GL_UNSIGNED_BYTE, geom.texture_data->rgb_data.data());
      glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
      glBindTexture(GL_TEXTURE_2D, 0);
      geom.texture_data->gl_id = static_cast<uint32_t>(tid);
    }
  }

  // Create the output texture (shared — visible in the main context after glFinish).
  GLuint out_tex = 0;
  glGenTextures(1, &out_tex);
  glBindTexture(GL_TEXTURE_2D, out_tex);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, tex_w, tex_h, 0,
               GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glBindTexture(GL_TEXTURE_2D, 0);

  // Create a local FBO (not shared; only this context uses it).
  GLuint fbo = 0;
  gf->glGenFramebuffers(1, &fbo);
  gf->glBindFramebuffer(GL_FRAMEBUFFER, fbo);
  gf->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                             GL_TEXTURE_2D, out_tex, 0);
  if (gf->glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
    RCLCPP_ERROR(rclcpp::get_logger("robot_model_plugin"), "BG FBO incomplete");
    gf->glBindFramebuffer(GL_FRAMEBUFFER, 0);
    gf->glDeleteFramebuffers(1, &fbo);
    glDeleteTextures(1, &out_tex);
    bg_ctx.doneCurrent();
    bake_in_progress_ = false;
    return;
  }

  glViewport(0, 0, tex_w, tex_h);
  glMatrixMode(GL_PROJECTION); glLoadIdentity();
  glOrtho(min_x, max_x, min_y, max_y, -1.0, 1.0);
  glMatrixMode(GL_MODELVIEW); glLoadIdentity();
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glClear(GL_COLOR_BUFFER_BIT);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // Sort all faces back-to-front by root_link Z (painter's algorithm).
  struct AnyFace {
    const LinkGeometry::MeshFace* mf;
    QColor fill;
    GLuint tex_id;
    QColor tint;
    double avg_z;
  };
  std::vector<AnyFace> all_faces;
  for (const auto& geom : geoms) {
    const QColor fill = geom.color.isValid() ? geom.color : QColor(160, 160, 160);
    const QColor tint = geom.color.isValid() ? geom.color : QColor(255, 255, 255);
    const GLuint tid = (geom.texture_data && geom.texture_data->gl_id != 0)
                           ? geom.texture_data->gl_id : 0u;
    for (const auto& mf : geom.mesh_faces) {
      const double avg_z = (mf.verts[0].z() + mf.verts[1].z() + mf.verts[2].z()) / 3.0;
      all_faces.push_back({&mf, fill, mf.use_texture ? tid : 0u, tint, avg_z});
    }
  }
  std::sort(all_faces.begin(), all_faces.end(),
      [](const AnyFace& a, const AnyFace& b) { return a.avg_z < b.avg_z; });

  // Render sorted faces; batch consecutive solid-color faces for efficiency.
  bool in_solid_batch = false;
  GLuint current_tex = 0;
  for (const auto& af : all_faces) {
    if (af.tex_id != 0) {
      if (in_solid_batch) { glEnd(); in_solid_batch = false; }
      if (af.tex_id != current_tex) {
        current_tex = af.tex_id;
        glEnable(GL_TEXTURE_2D);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
        glBindTexture(GL_TEXTURE_2D, current_tex);
      }
      glBegin(GL_TRIANGLES);
      for (int i = 0; i < 3; ++i) {
        const float b = af.mf->brightness[i];
        glColor4f(af.tint.redF() * b, af.tint.greenF() * b, af.tint.blueF() * b, 1.0f);
        glTexCoord2f(af.mf->uvs[i][0], af.mf->uvs[i][1]);
        glVertex2d(af.mf->verts[i].x(), af.mf->verts[i].y());
      }
      glEnd();
    } else {
      if (current_tex != 0) {
        if (in_solid_batch) { glEnd(); in_solid_batch = false; }
        glDisable(GL_TEXTURE_2D);
        current_tex = 0;
      }
      if (!in_solid_batch) { glBegin(GL_TRIANGLES); in_solid_batch = true; }
      const QColor fc = af.mf->color.isValid() ? af.mf->color : af.fill;
      for (int i = 0; i < 3; ++i) {
        const float b = af.mf->brightness[i];
        glColor4f(fc.redF() * b, fc.greenF() * b, fc.blueF() * b, 1.0f);
        glVertex2d(af.mf->verts[i].x(), af.mf->verts[i].y());
      }
    }
  }
  if (in_solid_batch) glEnd();
  if (current_tex != 0) glDisable(GL_TEXTURE_2D);

  // Ensure all GPU work is visible to the main context before handing off.
  glFinish();
  gf->glBindFramebuffer(GL_FRAMEBUFFER, 0);
  gf->glDeleteFramebuffers(1, &fbo);
  bg_ctx.doneCurrent();

  // Hand the completed texture off to the main thread.
  BakeResult result;
  result.texture = out_tex;
  result.bbox_min_x = min_x; result.bbox_max_x = max_x;
  result.bbox_min_y = min_y; result.bbox_max_y = max_y;
  result.baked_scale = scale;
  result.was_offscreen = off_screen;
  result.was_at_cap = (tex_size == max_tex);
  result.ready = true;
  {
    std::lock_guard<std::mutex> lk(bake_result_mutex_);
    pending_bake_ = result;
  }
  bake_in_progress_ = false;
}

void RobotModelPlugin::PrintError(const std::string& message) {
  PrintErrorHelper(ui_.status, message);
}

void RobotModelPlugin::PrintInfo(const std::string& message) {
  PrintInfoHelper(ui_.status, message);
}

void RobotModelPlugin::PrintWarning(const std::string& message) {
  PrintWarningHelper(ui_.status, message);
}

void RobotModelPlugin::LoadConfig(const YAML::Node& node,
                                        const std::string& /*path*/) {
  if (node["alpha"]) {
    ui_.alpha->setValue(node["alpha"].as<double>());
  }
  const int source = node["source_type"] ? node["source_type"].as<int>() : 0;
  ui_.source_combo->setCurrentIndex(source);  // fires SourceChanged
  if (source == 0) {
    if (node["topic"]) {
      ui_.topic->setText(
          QString::fromStdString(node["topic"].as<std::string>()));
    }
    TopicEdited();
  } else {
    if (node["file_path"]) {
      ui_.filepath->setText(
          QString::fromStdString(node["file_path"].as<std::string>()));
    }
    FileEdited();
  }
}

void RobotModelPlugin::SaveConfig(YAML::Emitter& emitter,
                                        const std::string& /*path*/) {
  emitter << YAML::Key << "alpha" << YAML::Value << ui_.alpha->value();
  emitter << YAML::Key << "source_type" << YAML::Value
          << ui_.source_combo->currentIndex();
  emitter << YAML::Key << "topic" << YAML::Value
          << ui_.topic->text().toStdString();
  emitter << YAML::Key << "file_path" << YAML::Value
          << ui_.filepath->text().toStdString();
}

}  // namespace mapviz_plugins
