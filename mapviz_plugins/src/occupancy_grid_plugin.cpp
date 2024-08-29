// *****************************************************************************
//
// Copyright (c) 2018, Eurecat
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Eurecat nor the
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

#include <GL/glew.h>

#include <mapviz_plugins/occupancy_grid_plugin.h>
#include <mapviz_plugins/topic_select.h>

// QT libraries
#include <QGLWidget>
#include <QPalette>

// Declare plugin
#include <pluginlib/class_list_macros.hpp>

// C++ standard libraries
#include <cstdio>
#include <string>
#include <vector>

PLUGINLIB_EXPORT_CLASS(mapviz_plugins::OccupancyGridPlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{
  const int CHANNELS = 4;

  typedef std::array<uchar, 256*4> Palette;

  Palette makeMapPalette()
  {
    Palette palette;
    uchar* palette_ptr = palette.data();
    // Standard gray map palette values
    for( int i = 0; i <= 100; i++ )
    {
      uchar v = 255 - (255 * i) / 100;
      *palette_ptr++ = v;   // red
      *palette_ptr++ = v;   // green
      *palette_ptr++ = v;   // blue
      *palette_ptr++ = 255;   // alpha
    }
    // illegal positive values in green
    for( int i = 101; i <= 127; i++ )
    {
      *palette_ptr++ = 0;   // red
      *palette_ptr++ = 255;   // green
      *palette_ptr++ = 0;   // blue
      *palette_ptr++ = 255;   // alpha
    }
    // illegal negative (char) values in shades of red/yellow
    for( int i = 128; i <= 254; i++ )
    {
      *palette_ptr++ = 255;   // red
      *palette_ptr++ = (255*(i-128))/(254-128);   // green
      *palette_ptr++ = 0;   // blue
      *palette_ptr++ = 255;   // alpha
    }
    // legal -1 value is tasteful blueish greenish grayish color
    *palette_ptr++ = 0x70;  // red
    *palette_ptr++ = 0x89;  // green
    *palette_ptr++ = 0x86;  // blue
    *palette_ptr++ = 160;   // alpha

    return palette;
  }

  Palette makeCostmapPalette()
  {
    Palette palette;
    uchar* palette_ptr = palette.data();

    // zero values have alpha=0
    *palette_ptr++ = 0;   // red
    *palette_ptr++ = 0;   // green
    *palette_ptr++ = 0;   // blue
    *palette_ptr++ = 0;   // alpha

    // Blue to red spectrum for most normal cost values
    for( int i = 1; i <= 98; i++ )
    {
      uchar v = (255 * i) / 100;
      *palette_ptr++ = v;   // red
      *palette_ptr++ = 0;   // green
      *palette_ptr++ = 255 - v;   // blue
      *palette_ptr++ = 255;   // alpha
    }
    // inscribed obstacle values (99) in cyan
    *palette_ptr++ = 0;   // red
    *palette_ptr++ = 255;   // green
    *palette_ptr++ = 255;   // blue
    *palette_ptr++ = 255;   // alpha
    // lethal obstacle values (100) in purple
    *palette_ptr++ = 255;   // red
    *palette_ptr++ = 0;   // green
    *palette_ptr++ = 255;   // blue
    *palette_ptr++ = 255;   // alpha
    // illegal positive values in green
    for( int i = 101; i <= 127; i++ )
    {
      *palette_ptr++ = 0;   // red
      *palette_ptr++ = 255;   // green
      *palette_ptr++ = 0;   // blue
      *palette_ptr++ = 255;   // alpha
    }
    // illegal negative (char) values in shades of red/yellow
    for( int i = 128; i <= 254; i++ )
    {
      *palette_ptr++ = 255;   // red
      *palette_ptr++ = (255*(i-128))/(254-128);   // green
      *palette_ptr++ = 0;   // blue
      *palette_ptr++ = 255;   // alpha
    }
    // legal -1 value is tasteful blueish greenish grayish color
    *palette_ptr++ = 0x70;  // red
    *palette_ptr++ = 0x89;  // green
    *palette_ptr++ = 0x86;  // blue
    *palette_ptr++ = 160;   // alpha

    return palette;
  }

  OccupancyGridPlugin::OccupancyGridPlugin() :
    MapvizPlugin(),
    ui_(),
    config_widget_(new QWidget()),
    transformed_(false),
    texture_id_(0),
    texture_x_(0.0),
    texture_y_(0.0),
    texture_size_(0),
    map_palette_(makeMapPalette()),
    costmap_palette_( makeCostmapPalette()),
    topic_(""),
    qos_(rmw_qos_profile_default)
  {
    ui_.setupUi(config_widget_);

    // Set background white
    QPalette p(config_widget_->palette());
    p.setColor(QPalette::Window, Qt::white);
    config_widget_->setPalette(p);

    // Set status text red
    QPalette p3(ui_.status->palette());
    p3.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p3);

    QObject::connect(ui_.select_grid, SIGNAL(clicked()), this, SLOT(SelectTopicGrid()));

    QObject::connect(
      ui_.topic_grid,
      SIGNAL(textEdited(const QString&)),
      this,
      SLOT(TopicGridEdited()));

    QObject::connect(
      this,
      SIGNAL(TargetFrameChanged(std::string)),
      this,
      SLOT(FrameChanged(std::string)));

    QObject::connect(
      ui_.checkbox_update,
      SIGNAL(toggled(bool)),
      this,
      SLOT(upgradeCheckBoxToggled(bool)));

    QObject::connect(
      ui_.color_scheme,
      SIGNAL(currentTextChanged(const QString &)),
      this,
      SLOT(colorSchemeUpdated(const QString &)));
  }

  void OccupancyGridPlugin::DrawIcon()
  {
    if (icon_)
    {
      QPixmap icon(16, 16);
      icon.fill(Qt::transparent);

      QPainter painter(&icon);
      painter.setRenderHint(QPainter::Antialiasing, true);

      QPen pen(Qt::black);

      pen.setWidth(2);
      pen.setCapStyle(Qt::SquareCap);
      painter.setPen(pen);

      painter.drawLine(2, 2, 14, 2);
      painter.drawLine(2, 2, 2, 14);
      painter.drawLine(14, 2, 14, 14);
      painter.drawLine(2, 14, 14, 14);
      painter.drawLine(8, 2, 8, 14);
      painter.drawLine(2, 8, 14, 8);

      icon_->SetPixmap(icon);
    }
  }

  void OccupancyGridPlugin::FrameChanged(std::string)
  {
    transformed_ = false;
  }

  void OccupancyGridPlugin::SelectTopicGrid()
  {
    auto [topic, qos] = SelectTopicDialog::selectTopic(
      node_,
      "nav_msgs/msg/OccupancyGrid",
      qos_);
    if (!topic.empty())
    {
      connectCallback(topic, qos);
    }
  }

  void OccupancyGridPlugin::TopicGridEdited()
  {
    const std::string topic = ui_.topic_grid->text().trimmed().toStdString();
    connectCallback(topic, qos_);
  }

  void OccupancyGridPlugin::connectCallback(const std::string& topic, const rmw_qos_profile_t& qos)
  {
    ui_.topic_grid->setText(QString::fromStdString(topic));
    if ((topic_ != topic) || !qosEqual(qos, qos_))
    {
      initialized_ = false;
      grid_.reset();
      raw_buffer_.clear();

      grid_sub_.reset();
      update_sub_.reset();
      topic_ = topic;
      qos_ = qos;

      if (!topic.empty())
      {
        grid_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
          topic,
          rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos),
          std::bind(&OccupancyGridPlugin::Callback, this, std::placeholders::_1));
        if(ui_.checkbox_update->isChecked())
        {
          update_sub_ = node_->create_subscription<map_msgs::msg::OccupancyGridUpdate>(
            topic,
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos),
            std::bind(&OccupancyGridPlugin::CallbackUpdate, this, std::placeholders::_1));
        }
        RCLCPP_INFO(node_->get_logger(), "Subscribing to %s", topic.c_str());
      }
    }
  }

  void OccupancyGridPlugin::upgradeCheckBoxToggled(bool)
  {
    const std::string topic = ui_.topic_grid->text().trimmed().toStdString();
    update_sub_.reset();

    if (ui_.checkbox_update)
    {
      update_sub_ = node_->create_subscription<map_msgs::msg::OccupancyGridUpdate>(
        topic,
        rclcpp::QoS(10),
        std::bind(&OccupancyGridPlugin::CallbackUpdate, this, std::placeholders::_1));
    }
  }

  void OccupancyGridPlugin::colorSchemeUpdated(const QString &)
  {
    if( grid_ && !raw_buffer_.empty())
    {
      const size_t width  = grid_->info.width;
      const size_t height = grid_->info.height;
      const Palette& palette =
        (ui_.color_scheme->currentText() == "map") ?  map_palette_ : costmap_palette_;

      for (size_t row = 0; row < height;  row++)
      {
        for (size_t col = 0; col < width; col++)
        {
          size_t index = (col + row * texture_size_);
          uchar color = raw_buffer_[index];
          memcpy( &color_buffer_[index*CHANNELS], &palette[color*CHANNELS], CHANNELS);
        }
      }
      updateTexture();
    }
  }

  void OccupancyGridPlugin::PrintError(const std::string& message)
  {
    PrintErrorHelper(ui_.status, message);
  }

  void OccupancyGridPlugin::PrintInfo(const std::string& message)
  {
    PrintInfoHelper(ui_.status, message);
  }

  void OccupancyGridPlugin::PrintWarning(const std::string& message)
  {
    PrintWarningHelper(ui_.status, message);
  }

  QWidget* OccupancyGridPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool OccupancyGridPlugin::Initialize(QGLWidget* canvas)
  {
    canvas_ = canvas;
    DrawIcon();
    return true;
  }

  void OccupancyGridPlugin::updateTexture()
  {
    if (texture_id_ != -1)
    {
      glDeleteTextures(1, &texture_id_);
    }

    // Get a new texture id.
    glGenTextures(1, &texture_id_);

    // Bind the texture with the correct size and null memory.
    glBindTexture(GL_TEXTURE_2D, texture_id_);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );

    glTexImage2D(
          GL_TEXTURE_2D,
          0,
          GL_RGBA,
          texture_size_,
          texture_size_,
          0,
          GL_RGBA,
          GL_UNSIGNED_BYTE,
          color_buffer_.data());

    glBindTexture(GL_TEXTURE_2D, 0);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
  }


  void OccupancyGridPlugin::Callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    grid_ = msg;
    const int width  = grid_->info.width;
    const int height = grid_->info.height;
    initialized_ = true;
    source_frame_ = grid_->header.frame_id;
    transformed_ = GetTransform( source_frame_, grid_->header.stamp, transform_);
    if ( !transformed_ )
    {
      PrintError("No transform between " + source_frame_ + " and " + target_frame_);
    }

    int32_t max_dimension = std::max(height, width);

    texture_size_ = 2;
    while (texture_size_ < max_dimension){
      texture_size_ = texture_size_ << 1;
    }

    const Palette& palette =
      (ui_.color_scheme->currentText() == "map") ?  map_palette_ : costmap_palette_;

    raw_buffer_.resize(texture_size_*texture_size_, 0);
    color_buffer_.resize(texture_size_*texture_size_*CHANNELS, 0);

    for (size_t row = 0; row < height; row++)
    {
      for (size_t col = 0; col < width; col++)
      {
        size_t index_src = (col + row*width);
        size_t index_dst = (col + row*texture_size_);
        uchar color = static_cast<uchar>( grid_->data[ index_src ] );
        raw_buffer_[index_dst] = color;
        memcpy( &color_buffer_[index_dst*CHANNELS], &palette[color*CHANNELS], CHANNELS);
      }
    }

    texture_x_ = static_cast<float>(width) / static_cast<float>(texture_size_);
    texture_y_ = static_cast<float>(height) / static_cast<float>(texture_size_);

    updateTexture();
    PrintInfo("Map received");
  }

  void OccupancyGridPlugin::CallbackUpdate(const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg)
  {
    PrintInfo("Update Received");

    if( initialized_ )
    {
      const Palette& palette =
        (ui_.color_scheme->currentText() == "map") ?  map_palette_ : costmap_palette_;

      for (size_t row = 0; row < msg->height; row++)
      {
        for (size_t col = 0; col < msg->width; col++)
        {
          size_t index_src = (col + row * msg->width);
          size_t index_dst = ( (col + msg->x) + (row + msg->y)*texture_size_);
          uchar color = static_cast<uchar>( msg->data[ index_src ] );
          raw_buffer_[index_dst] = color;
          memcpy( &color_buffer_[index_dst*CHANNELS], &palette[color*CHANNELS], CHANNELS);
        }
      }
      updateTexture();
    }
  }

  void OccupancyGridPlugin::Draw(double x, double y, double scale)
  {
    glPushMatrix();

    if( grid_ && transformed_)
    {
      double resolution = grid_->info.resolution;
      glTranslatef( transform_.GetOrigin().getX(),
                    transform_.GetOrigin().getY(),
                    0.0);

      const double RAD_TO_DEG = 180.0 / M_PI;

      tf2Scalar yaw, pitch, roll;
      tf2::Matrix3x3 mat( transform_.GetOrientation() );
      mat.getEulerYPR(yaw, pitch, roll);

      glRotatef(pitch * RAD_TO_DEG, 0, 1, 0);
      glRotatef(roll  * RAD_TO_DEG, 1, 0, 0);
      glRotatef(yaw   * RAD_TO_DEG, 0, 0, 1);

      glTranslatef( grid_->info.origin.position.x,
                    grid_->info.origin.position.y,
                    0.0);

      glScalef( resolution, resolution, 1.0);

      float width  = static_cast<float>(grid_->info.width);
      float height = static_cast<float>(grid_->info.height);

      glEnable(GL_TEXTURE_2D);
      glBindTexture(GL_TEXTURE_2D, texture_id_);
      glBegin(GL_TRIANGLES);

      glColor4f(1.0f, 1.0f, 1.0f, ui_.alpha->value() );

      glTexCoord2d(0, 0);
      glVertex2d(0, 0);
      glTexCoord2d(texture_x_, 0);
      glVertex2d(width, 0);
      glTexCoord2d(texture_x_, texture_y_);
      glVertex2d(width, height);

      glTexCoord2d(0, 0);
      glVertex2d(0, 0);
      glTexCoord2d(texture_x_, texture_y_);
      glVertex2d(width, height);
      glTexCoord2d(0, texture_y_);
      glVertex2d(0, height);

      glEnd();

      glBindTexture(GL_TEXTURE_2D, 0);
      glDisable(GL_TEXTURE_2D);
    }
    glPopMatrix();
  }

  void OccupancyGridPlugin::Transform()
  {
    if( !initialized_ ) return;
    swri_transform_util::Transform transform;
    if ( grid_ )
    {
      if( GetTransform( source_frame_, rclcpp::Time(0), transform) )
      {
        transformed_ = true;
        transform_ = transform;
      }
    }
    if ( !transformed_ )
    {
      PrintError("No transform between " + source_frame_ + " and " + target_frame_);
    }
  }

  void OccupancyGridPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    LoadQosConfig(node, qos_);
    if (node["topic"])
    {
      std::string topic = node["topic"].as<std::string>();
      ui_.topic_grid->setText(QString::fromStdString(topic));
    }

    if (node["update"])
    {
      bool checked = node["update"].as<bool>();
      ui_.checkbox_update->setChecked( checked );
    }

    if (node["alpha"])
    {
      double alpha = node["alpha"].as<double>();
      ui_.alpha->setValue(alpha);
    }

    if (node["scheme"])
    {
      std::string scheme = node["scheme"].as<std::string>();
      int index = ui_.color_scheme->findText(QString::fromStdString(scheme), Qt::MatchExactly);
      if (index >= 0)
      {
        ui_.color_scheme->setCurrentIndex(index);
      }
      colorSchemeUpdated(QString::fromStdString(scheme));
    }

    TopicGridEdited();
  }

  void OccupancyGridPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    emitter << YAML::Key << "alpha"  << YAML::Value << ui_.alpha->value();
    emitter << YAML::Key << "topic"  << YAML::Value << ui_.topic_grid->text().toStdString();
    emitter << YAML::Key << "update" << YAML::Value << ui_.checkbox_update->isChecked();
    emitter << YAML::Key
      << "scheme"
      << YAML::Value
      << ui_.color_scheme->currentText().toStdString();
    SaveQosConfig(emitter, qos_);
  }
}   // namespace mapviz_plugins

