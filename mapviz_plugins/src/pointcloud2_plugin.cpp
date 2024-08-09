// *****************************************************************************
//
// Copyright (c) 2015, Southwest Research Institute® (SwRI®)
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

#include <GL/glew.h>
#include <mapviz_plugins/pointcloud2_plugin.h>
#include <mapviz_plugins/topic_select.h>

// Boost libraries
#include <boost/algorithm/string.hpp>

// QT libraries
#include <QDialog>
#include <QGLWidget>

// ROS libraries
#include <rclcpp/rclcpp.hpp>
#include <swri_transform_util/transform.h>

// Declare plugin
#include <pluginlib/class_list_macros.hpp>

// C++ standard libraries
#include <cstdio>
#include <map>
#include <string>
#include <utility>
#include <vector>

PLUGINLIB_EXPORT_CLASS(mapviz_plugins::PointCloud2Plugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{
  PointCloud2Plugin::PointCloud2Plugin() :
    MapvizPlugin(),
    ui_(),
    config_widget_(new QWidget()),
    topic_(""),
    alpha_(1.0),
    max_value_(100.0),
    min_value_(0.0),
    point_size_(3),
    buffer_size_(1),
    new_topic_(true),
    has_message_(false),
    num_of_feats_(0),
    need_new_list_(true),
    need_minmax_(false),
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

    // Initialize color selector colors
    ui_.min_color->setColor(Qt::white);
    ui_.max_color->setColor(Qt::black);
    // Set color transformer choices
    ui_.color_transformer->addItem(QString("Flat Color"), QVariant(0));

    QObject::connect(ui_.selecttopic,
                     SIGNAL(clicked()),
                     this,
                     SLOT(SelectTopic()));
    QObject::connect(ui_.buttonResetBuffer,
                     SIGNAL(clicked()),
                     this,
                     SLOT(ClearPointClouds()));
    QObject::connect(ui_.topic,
                     SIGNAL(editingFinished()),
                     this,
                     SLOT(TopicEdited()));
    QObject::connect(ui_.alpha,
                     SIGNAL(valueChanged(double)),
                     this,
                     SLOT(AlphaEdited(double)));
    QObject::connect(ui_.color_transformer,
                     SIGNAL(currentIndexChanged(int)),
                     this,
                     SLOT(ColorTransformerChanged(int)));
    QObject::connect(ui_.max_color,
                     SIGNAL(colorEdited(const QColor &)),
                     this,
                     SLOT(UpdateColors()));
    QObject::connect(ui_.min_color,
                     SIGNAL(colorEdited(const QColor &)),
                     this,
                     SLOT(UpdateColors()));
    QObject::connect(ui_.minValue,
                     SIGNAL(valueChanged(double)),
                     this,
                     SLOT(MinValueChanged(double)));
    QObject::connect(ui_.maxValue,
                     SIGNAL(valueChanged(double)),
                     this,
                     SLOT(MaxValueChanged(double)));
    QObject::connect(ui_.bufferSize,
                     SIGNAL(valueChanged(int)),
                     this,
                     SLOT(BufferSizeChanged(int)));
    QObject::connect(ui_.pointSize,
                     SIGNAL(valueChanged(int)),
                     this,
                     SLOT(PointSizeChanged(int)));
    QObject::connect(ui_.use_rainbow,
                     SIGNAL(stateChanged(int)),
                     this,
                     SLOT(UseRainbowChanged(int)));
    QObject::connect(ui_.unpack_rgb,
                     SIGNAL(stateChanged(int)),
                     this,
                     SLOT(UseRainbowChanged(int)));
    QObject::connect(ui_.use_automaxmin,
                     SIGNAL(stateChanged(int)),
                     this,
                     SLOT(UseAutomaxminChanged(int)));
    QObject::connect(ui_.max_color,
                     SIGNAL(colorEdited(const QColor &)),
                     this,
                     SLOT(DrawIcon()));
    QObject::connect(ui_.min_color,
                     SIGNAL(colorEdited( const QColor &)),
                     this,
                     SLOT(DrawIcon()));
    QObject::connect(this,
                     SIGNAL(TargetFrameChanged(const std::string&)),
                     this,
                     SLOT(ResetTransformedPointClouds()));
    QObject::connect(this,
                     SIGNAL(VisibleChanged(bool)),
                     this,
                     SLOT(SetSubscription(bool)));
  }

  void PointCloud2Plugin::ClearHistory()
  {
    RCLCPP_DEBUG(node_->get_logger(), "PointCloud2Plugin::ClearHistory()");
    scans_.clear();
  }

  void PointCloud2Plugin::DrawIcon()
  {
    if (icon_)
    {
      QPixmap icon(16, 16);
      icon.fill(Qt::transparent);

      QPainter painter(&icon);
      painter.setRenderHint(QPainter::Antialiasing, true);

      QPen pen;
      pen.setWidth(4);
      pen.setCapStyle(Qt::RoundCap);

      pen.setColor(ui_.min_color->color());
      painter.setPen(pen);
      painter.drawPoint(2, 13);

      pen.setColor(ui_.min_color->color());
      painter.setPen(pen);
      painter.drawPoint(4, 6);

      pen.setColor(ui_.max_color->color());
      painter.setPen(pen);
      painter.drawPoint(12, 9);

      pen.setColor(ui_.max_color->color());
      painter.setPen(pen);
      painter.drawPoint(13, 2);

      icon_->SetPixmap(icon);
    }
  }

  void PointCloud2Plugin::ResetTransformedPointClouds()
  {
    QMutexLocker locker(&scan_mutex_);
    for (Scan& scan : scans_)
    {
      scan.transformed = false;
      scan.gl_color.clear();
      scan.gl_point.clear();
    }
  }

  void PointCloud2Plugin::ClearPointClouds()
  {
      QMutexLocker locker(&scan_mutex_);
      scans_.clear();
  }

  void PointCloud2Plugin::SetSubscription(bool subscribe)
  {
    pc2_sub_.reset();

    if (subscribe && !topic_.empty())
    {
      pc2_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic_,
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_)),
        std::bind(&PointCloud2Plugin::PointCloud2Callback, this, std::placeholders::_1)
      );
      new_topic_ = true;
      need_new_list_ = true;
      max_.clear();
      min_.clear();
    }
  }

  QColor PointCloud2Plugin::CalculateColor(const StampedPoint& point)
  {
    float val;
    unsigned int color_transformer =
      static_cast<unsigned int>(ui_.color_transformer->currentIndex());
    unsigned int transformer_index = color_transformer -1;
    if (num_of_feats_ > 0 && color_transformer > 0)
    {
      val = point.features[transformer_index];
      if (need_minmax_)
      {
        if (val > max_[transformer_index])
        {
          max_[transformer_index] = val;
        }

        if (val < min_[transformer_index])
        {
          min_[transformer_index] = val;
        }
      }
    } else {
      // No intensity or  (color_transformer == COLOR_FLAT)
      return ui_.min_color->color();
    }

    if (ui_.unpack_rgb->isChecked())
    {
        uint8_t* pixelColor = reinterpret_cast<uint8_t*>(&val);
        return QColor(pixelColor[2], pixelColor[1], pixelColor[0], 255);
    }

    if (max_value_ > min_value_)
    {
      val = (val - min_value_) / (max_value_ - min_value_);
    }
    val = std::max(0.0f, std::min(val, 1.0f));

    if (ui_.use_automaxmin->isChecked())
    {
      max_value_ = max_[transformer_index];
      min_value_ = min_[transformer_index];
    }

    if (ui_.use_rainbow->isChecked())
    {  // Hue Interpolation
      int hue = static_cast<int>(val * 255.0);
      return QColor::fromHsl(hue, 255, 127, 255);
    } else {
      const QColor min_color = ui_.min_color->color();
      const QColor max_color = ui_.max_color->color();
      // RGB Interpolation
      int red, green, blue;
      red = static_cast<int>(val * max_color.red() + ((1.0 - val) * min_color.red()));
      green = static_cast<int>(val * max_color.green() + ((1.0 - val) * min_color.green()));
      blue = static_cast<int>(val * max_color.blue() + ((1.0 - val) * min_color.blue()));
      return QColor(red, green, blue, 255);
    }
  }

  inline int32_t findChannelIndex(
    const sensor_msgs::msg::PointCloud2::SharedPtr cloud,
    const std::string& channel)
  {
    for (int32_t i = 0; static_cast<size_t>(i) < cloud->fields.size(); ++i)
    {
      if (cloud->fields[i].name == channel)
      {
        return i;
      }
    }

    return -1;
  }

  void PointCloud2Plugin::UpdateColors()
  {
    {
      QMutexLocker locker(&scan_mutex_);
      for (Scan& scan : scans_)
      {
        scan.gl_color.clear();
        scan.gl_color.reserve(scan.points.size()*4);
        for (const StampedPoint& point : scan.points)
        {
          const QColor color = CalculateColor(point);
          scan.gl_color.push_back( color.red());
          scan.gl_color.push_back( color.green());
          scan.gl_color.push_back( color.blue());
          scan.gl_color.push_back( static_cast<uint8_t>(alpha_ * 255.0 ) );
        }
      }
    }
    canvas_->update();
  }

  void PointCloud2Plugin::SelectTopic()
  {
    auto [topic, qos] = SelectTopicDialog::selectTopic(
      node_,
      "sensor_msgs/msg/PointCloud2",
      qos_);
    if (!topic.empty())
    {
      connectCallback(topic, qos);
    }
  }


  void PointCloud2Plugin::TopicEdited()
  {
    std::string topic = ui_.topic->text().trimmed().toStdString();
    connectCallback(topic, qos_);
  }

  void PointCloud2Plugin::connectCallback(const std::string& topic, const rmw_qos_profile_t& qos)
  {
    ui_.topic->setText(QString::fromStdString(topic));
    if ((topic != topic_) || !qosEqual(qos, qos_))
    {
      initialized_ = false;
      {
        QMutexLocker locker(&scan_mutex_);
        scans_.clear();
      }
      has_message_ = false;
      PrintWarning("No messages received.");

      topic_ = topic;
      qos_ = qos;
      SetSubscription(this->Visible());
    }

  }

  void PointCloud2Plugin::MinValueChanged(double value)
  {
    min_value_ = value;
    UpdateColors();
  }

  void PointCloud2Plugin::MaxValueChanged(double value)
  {
    max_value_ = value;
    UpdateColors();
  }

  void PointCloud2Plugin::BufferSizeChanged(int value)
  {
    buffer_size_ = (size_t)value;

    if (buffer_size_ > 0)
    {
      QMutexLocker locker(&scan_mutex_);
      while (scans_.size() > buffer_size_)
      {
        scans_.pop_front();
      }
    }

    canvas_->update();
  }

  void PointCloud2Plugin::PointSizeChanged(int value)
  {
    point_size_ = (size_t)value;

    canvas_->update();
  }

  void PointCloud2Plugin::PointCloud2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!has_message_)
    {
      initialized_ = true;
      has_message_ = true;
    }

    // Note that unlike some plugins, this one does not store nor rely on the
    // source_frame_ member variable.  This one can potentially store many
    // messages with different source frames, so we need to store and transform
    // them individually.

    Scan scan;
    {
        // recycle already allocated memory, reusing an old scan
      QMutexLocker locker(&scan_mutex_);
      if (buffer_size_ > 0 )
      {
          if( scans_.size() >= buffer_size_)
          {
              scan = std::move( scans_.front() );
          } else {
             glGenBuffers(1, &scan.color_vbo);
             glGenBuffers(1, &scan.point_vbo);
          }
          while (scans_.size() >= buffer_size_)
          {
            scans_.pop_front();
          }
      }
    }

    scan.stamp = msg->header.stamp;
    scan.color = QColor::fromRgbF(1.0f, 0.0f, 0.0f, 1.0f);
    scan.source_frame = msg->header.frame_id;
    scan.transformed = true;

    swri_transform_util::Transform transform;
    if (!GetTransform(scan.source_frame, msg->header.stamp, transform))
    {
      scan.transformed = false;
      PrintError("No transform between " + scan.source_frame + " and " + target_frame_);
      return;
    }

    int32_t xi = findChannelIndex(msg, "x");
    int32_t yi = findChannelIndex(msg, "y");
    int32_t zi = findChannelIndex(msg, "z");

    if (xi == -1 || yi == -1 || zi == -1)
    {
      return;
    }

    if (new_topic_)
    {
      for (auto & field : msg->fields)
      {
        FieldInfo input;
        std::string name = field.name;

        uint32_t offset_value = field.offset;
        uint8_t datatype_value = field.datatype;
        input.offset = offset_value;
        input.datatype = datatype_value;
        scan.new_features.insert(std::pair<std::string, FieldInfo>(name, input));
      }

      new_topic_ = false;
      num_of_feats_ = scan.new_features.size();

      max_.resize(num_of_feats_);
      min_.resize(num_of_feats_);

      int label = 1;
      if (need_new_list_)
      {
        int new_feature_index = ui_.color_transformer->currentIndex();
        std::map<std::string, FieldInfo>::const_iterator it;
        for (it = scan.new_features.begin(); it != scan.new_features.end(); ++it)
        {
          ui_.color_transformer->removeItem(static_cast<int>(num_of_feats_));
          num_of_feats_--;
        }

        for (it = scan.new_features.begin(); it != scan.new_features.end(); ++it)
        {
          std::string const field = it->first;
          if (field == saved_color_transformer_)
          {
            // The very first time we see a new set of features, that means the
            // plugin was just created; if we have a saved value, set the current
            // index to that and clear the saved value.
            new_feature_index = label;
            saved_color_transformer_ = "";
          }

          ui_.color_transformer->addItem(QString::fromStdString(field), QVariant(label));
          num_of_feats_++;
          label++;
        }
        ui_.color_transformer->setCurrentIndex(new_feature_index);
        need_new_list_ = false;
      }
    }

    if (!msg->data.empty())
    {
      const uint8_t* ptr = &msg->data.front();
      const uint32_t point_step = msg->point_step;
      const uint32_t xoff = msg->fields[xi].offset;
      const uint32_t yoff = msg->fields[yi].offset;
      const uint32_t zoff = msg->fields[zi].offset;
      const size_t num_points = msg->data.size() / point_step;
      const size_t num_features = scan.new_features.size();
      scan.points.resize(num_points);

      std::vector<FieldInfo> field_infos;
      field_infos.reserve(num_features);
      for (auto & new_feature : scan.new_features)
      {
        field_infos.push_back(new_feature.second);
      }

      scan.gl_point.clear();
      scan.gl_point.reserve(num_points*2);
      scan.gl_color.clear();
      scan.gl_color.reserve(num_points*4);

      for (size_t i = 0; i < num_points; i++, ptr += point_step)
      {
        float x = *reinterpret_cast<const float*>(ptr + xoff);
        float y = *reinterpret_cast<const float*>(ptr + yoff);
        float z = *reinterpret_cast<const float*>(ptr + zoff);

        StampedPoint& point = scan.points[i];
        point.point = tf2::Vector3(x, y, z);

        point.features.resize(num_features);

        for (int count=0; count < field_infos.size(); count++)
        {
          point.features[count] = PointFeature(ptr, field_infos[count]);
        }
        if (scan.transformed)
        {
          const tf2::Vector3 transformed_point = transform * point.point;
          scan.gl_point.push_back( transformed_point.getX() );
          scan.gl_point.push_back( transformed_point.getY() );
        }
        const QColor color = CalculateColor(point);
        scan.gl_color.push_back( color.red());
        scan.gl_color.push_back( color.green());
        scan.gl_color.push_back( color.blue());
        scan.gl_color.push_back( static_cast<uint8_t>(alpha_ * 255.0 ) );
      }
    }

    {
      QMutexLocker locker(&scan_mutex_);
      scans_.push_back( std::move(scan) );
    }
    new_topic_ = true;
    canvas_->update();
  }

  float PointCloud2Plugin::PointFeature(const uint8_t* data, const FieldInfo& feature_info)
  {
    switch (feature_info.datatype)
    {
      case 1:
        return *reinterpret_cast<const int8_t*>(data + feature_info.offset);
      case 2:
        return *(data + feature_info.offset);
      case 3:
        return *reinterpret_cast<const int16_t*>(data + feature_info.offset);
      case 4:
        return *reinterpret_cast<const uint16_t*>(data + feature_info.offset);
      case 5:
        return *reinterpret_cast<const int32_t*>(data + feature_info.offset);
      case 6:
        return *reinterpret_cast<const uint32_t*>(data + feature_info.offset);
      case 7:
        return *reinterpret_cast<const float*>(data + feature_info.offset);
      case 8:
        return static_cast<float>(*reinterpret_cast<const double*>(data + feature_info.offset));
      default:
        RCLCPP_WARN(node_->get_logger(), "Unknown data type in point: %d", feature_info.datatype);
        return 0.0;
    }
  }

  void PointCloud2Plugin::PrintError(const std::string& message)
  {
    PrintErrorHelper(ui_.status, message);
  }

  void PointCloud2Plugin::PrintInfo(const std::string& message)
  {
    PrintInfoHelper(ui_.status, message);
  }

  void PointCloud2Plugin::PrintWarning(const std::string& message)
  {
    PrintWarningHelper(ui_.status, message);
  }

  QWidget* PointCloud2Plugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool PointCloud2Plugin::Initialize(QGLWidget* canvas)
  {
    canvas_ = canvas;

    DrawIcon();

    return true;
  }

  void PointCloud2Plugin::Draw(double x, double y, double scale)
  {
    glPointSize(point_size_);

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);

    {
      QMutexLocker locker(&scan_mutex_);

      for (Scan& scan : scans_)
      {
        if (scan.transformed && !scan.gl_color.empty())
        {
          glBindBuffer(GL_ARRAY_BUFFER, scan.point_vbo);  // coordinates
          glBufferData(
            GL_ARRAY_BUFFER,
            scan.gl_point.size() * sizeof(float),
            scan.gl_point.data(),
            GL_STATIC_DRAW);
          glVertexPointer( 2, GL_FLOAT, 0, nullptr);

          glBindBuffer(GL_ARRAY_BUFFER, scan.color_vbo);  // color
          glBufferData(
            GL_ARRAY_BUFFER,
            scan.gl_color.size() * sizeof(uint8_t),
            scan.gl_color.data(),
            GL_STATIC_DRAW);
          glColorPointer( 4, GL_UNSIGNED_BYTE, 0, nullptr);

          glDrawArrays(GL_POINTS, 0, scan.gl_point.size() / 2 );
        }
      }
    }
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    PrintInfo("OK");
  }

  void PointCloud2Plugin::UseRainbowChanged(int check_state)
  {
    UpdateMinMaxWidgets();
    UpdateColors();
  }

  void PointCloud2Plugin::UseAutomaxminChanged(int check_state)
  {
    need_minmax_ = check_state == Qt::Checked;
    if( !need_minmax_ )
    {
      min_value_ = ui_.minValue->value();
      max_value_ = ui_.maxValue->value();
    }

    UpdateMinMaxWidgets();
    UpdateColors();
  }

  void PointCloud2Plugin::Transform()
  {
    {
      QMutexLocker locker(&scan_mutex_);

      bool was_using_latest_transforms = use_latest_transforms_;
      use_latest_transforms_ = false;
      for (Scan& scan : scans_)
      {
        if (!scan.transformed)
        {
          swri_transform_util::Transform transform;
          if (GetTransform(scan.source_frame, scan.stamp, transform))
          {
            scan.gl_point.clear();
            scan.gl_point.reserve(scan.points.size()*2);

            scan.transformed = true;
            for (StampedPoint& point : scan.points)
            {
              const tf2::Vector3 transformed_point = transform * point.point;
              scan.gl_point.push_back( transformed_point.getX() );
              scan.gl_point.push_back( transformed_point.getY() );
            }
          } else {
            RCLCPP_WARN(node_->get_logger(), "Unable to get transform.");
            scan.transformed = false;
          }
        }
      }
      use_latest_transforms_ = was_using_latest_transforms;
    }
    // Z color is based on transformed color, so it is dependent on the
    // transform
    if (ui_.color_transformer->currentIndex() == COLOR_Z)
    {
      UpdateColors();
    }
  }

  void PointCloud2Plugin::LoadConfig(const YAML::Node& node,
                                     const std::string& path)
  {
    LoadQosConfig(node, qos_);

    if (node["topic"])
    {
      std::string topic = node["topic"].as<std::string>();
      ui_.topic->setText(boost::trim_copy(topic).c_str());
      TopicEdited();
    }

    if (node["size"])
    {
      point_size_ = node["size"].as<size_t>();
      ui_.pointSize->setValue(static_cast<int>(point_size_));
    }

    if (node["buffer_size"])
    {
      buffer_size_ = node["buffer_size"].as<size_t>();
      ui_.bufferSize->setValue(static_cast<int>(buffer_size_));
    }

    if (node["color_transformer"])
    {
      saved_color_transformer_ = node["color_transformer"].as<std::string>();
    }

    if (node["min_color"])
    {
      std::string min_color_str = node["min_color"].as<std::string>();
      ui_.min_color->setColor(QColor(min_color_str.c_str()));
    }

    if (node["max_color"])
    {
      std::string max_color_str = node["max_color"].as<std::string>();
      ui_.max_color->setColor(QColor(max_color_str.c_str()));
    }

    if (node["value_min"])
    {
      min_value_ = node["value_min"].as<double>();
      ui_.minValue->setValue(min_value_);
    }

    if (node["value_max"])
    {
      max_value_ = node["value_max"].as<double>();
      ui_.maxValue->setValue(max_value_);
    }

    if (node["alpha"])
    {
      alpha_ = node["alpha"].as<double>();
      ui_.alpha->setValue(alpha_);
    }

    if (node["use_rainbow"])
    {
      bool use_rainbow = node["use_rainbow"].as<bool>();
      ui_.use_rainbow->setChecked(use_rainbow);
    }

    if (node["unpack_rgb"])
    {
      bool unpack_rgb = node["unpack_rgb"].as<bool>();
      ui_.unpack_rgb->setChecked(unpack_rgb);
    }

    // UseRainbowChanged must be called *before* ColorTransformerChanged
    UseRainbowChanged(ui_.use_rainbow->checkState());

    if (node["use_automaxmin"])
    {
      bool use_automaxmin = node["use_automaxmin"].as<bool>();
      ui_.use_automaxmin->setChecked(use_automaxmin);
    }
    // UseRainbowChanged must be called *before* ColorTransformerChanged
    UseAutomaxminChanged(ui_.use_automaxmin->checkState());
    // ColorTransformerChanged will also update colors of all points
    ColorTransformerChanged(ui_.color_transformer->currentIndex());
  }

  void PointCloud2Plugin::ColorTransformerChanged(int index)
  {
    RCLCPP_DEBUG(node_->get_logger(), "Color transformer changed to %d", index);
    UpdateMinMaxWidgets();
    UpdateColors();
  }

  void PointCloud2Plugin::UpdateMinMaxWidgets()
  {
    bool color_is_flat = ui_.color_transformer->currentIndex() == COLOR_FLAT;

    if (color_is_flat)
    {
      ui_.maxColorLabel->hide();
      ui_.max_color->hide();
      ui_.minColorLabel->hide();
      ui_.min_max_color_widget->show();
      ui_.min_max_value_widget->hide();
      ui_.use_automaxmin->hide();
      ui_.use_rainbow->hide();
    } else {
      ui_.maxColorLabel->show();
      ui_.max_color->show();
      ui_.minColorLabel->show();
      ui_.min_max_color_widget->setVisible(!ui_.use_rainbow->isChecked());
      ui_.min_max_value_widget->setVisible(!ui_.use_automaxmin->isChecked());
      ui_.use_automaxmin->show();
      ui_.use_rainbow->show();
    }

    config_widget_->updateGeometry();
    config_widget_->adjustSize();

    Q_EMIT SizeChanged();
  }

  /**
   * Coerces alpha to [0.0, 1.0] and stores it in alpha_
   */
  void PointCloud2Plugin::AlphaEdited(double value)
  {
    alpha_ = std::max(0.0f, std::min(static_cast<float>(value), 1.0f));
  }

  void PointCloud2Plugin::SaveConfig(YAML::Emitter& emitter,
                                     const std::string& path)
  {
    emitter << YAML::Key << "topic" <<
      YAML::Value << boost::trim_copy(ui_.topic->text().toStdString());
    emitter << YAML::Key << "size" <<
      YAML::Value << ui_.pointSize->value();
    emitter << YAML::Key << "buffer_size" <<
      YAML::Value << ui_.bufferSize->value();
    emitter << YAML::Key << "alpha" <<
      YAML::Value << alpha_;
    emitter << YAML::Key << "color_transformer" <<
      YAML::Value << ui_.color_transformer->currentText().toStdString();
    emitter << YAML::Key << "min_color" <<
      YAML::Value << ui_.min_color->color().name().toStdString();
    emitter << YAML::Key << "max_color" <<
      YAML::Value << ui_.max_color->color().name().toStdString();
    emitter << YAML::Key << "value_min" <<
      YAML::Value << ui_.minValue->value();
    emitter << YAML::Key << "value_max" <<
      YAML::Value << ui_.maxValue->value();
    emitter << YAML::Key << "use_rainbow" <<
      YAML::Value << ui_.use_rainbow->isChecked();
    emitter << YAML::Key << "use_automaxmin" <<
      YAML::Value << ui_.use_automaxmin->isChecked();
    emitter << YAML::Key << "unpack_rgb" <<
      YAML::Value << ui_.unpack_rgb->isChecked();

    SaveQosConfig(emitter, qos_);
  }
}   // namespace mapviz_plugins
