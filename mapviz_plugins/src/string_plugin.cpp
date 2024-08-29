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
// ARE DISCLAIMED. IN NO EVENT SHALL Southwest Research Institute® BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
// OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.
//
// *****************************************************************************

#include <mapviz_plugins/string_plugin.h>
#include <mapviz_plugins/topic_select.h>

#include <QFontDialog>
#include <QFontDatabase>

#include <pluginlib/class_list_macros.hpp>

#include <string>
PLUGINLIB_EXPORT_CLASS(mapviz_plugins::StringPlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{
  const char* StringPlugin::COLOR_KEY = "color";
  const char* StringPlugin::FONT_KEY = "font";
  const char* StringPlugin::TOPIC_KEY = "topic";
  const char* StringPlugin::ANCHOR_KEY = "anchor";
  const char* StringPlugin::UNITS_KEY = "units";
  const char* StringPlugin::OFFSET_X_KEY = "offset_x";
  const char* StringPlugin::OFFSET_Y_KEY = "offset_y";
  static constexpr int DEFAULT_FONT_SIZE = 9;

  StringPlugin::StringPlugin() :
    MapvizPlugin(),
    ui_(),
    config_widget_(new QWidget()),
    anchor_(TOP_LEFT),
    units_(PIXELS),
    offset_x_(0),
    offset_y_(0),
    has_message_(false),
    has_painted_(false),
    color_(Qt::black),
    font_(),
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

    QObject::connect(ui_.selecttopic, SIGNAL(clicked()), this, SLOT(SelectTopic()));
    QObject::connect(ui_.topic, SIGNAL(editingFinished()), this, SLOT(TopicEdited()));
    QObject::connect(ui_.anchor, SIGNAL(activated(QString)), this, SLOT(SetAnchor(QString)));
    QObject::connect(ui_.units, SIGNAL(activated(QString)), this, SLOT(SetUnits(QString)));
    QObject::connect(ui_.offsetx, SIGNAL(valueChanged(int)), this, SLOT(SetOffsetX(int)));
    QObject::connect(ui_.offsety, SIGNAL(valueChanged(int)), this, SLOT(SetOffsetY(int)));
    QObject::connect(ui_.font_button, SIGNAL(clicked()), this, SLOT(SelectFont()));
    QObject::connect(ui_.color, SIGNAL(colorEdited(const QColor &)), this, SLOT(SelectColor()));

    // Change the default font size to our desired size and update the UI with it
    font_.setPointSize(DEFAULT_FONT_SIZE);
    ui_.font_button->setFont(font_);
    ui_.font_button->setText(font_.family());

    ui_.color->setColor(color_);
  }

  bool StringPlugin::Initialize(QGLWidget* canvas)
  {
    canvas_ = canvas;
    return true;
  }

  void StringPlugin::Draw(double, double, double)
  {
    // This plugin doesn't do any  OpenGL drawing.
  }

  void StringPlugin::Paint(QPainter* painter, double, double, double)
  {
    if (has_message_)
    {
      painter->save();
      painter->resetTransform();
      painter->setFont(font_);

      if (!has_painted_)
      {
        // After the first time we get a new message, we do not know how wide it's
        // going to be when rendered, so we can't accurately calculate the top left
        // coordinate if it's offset from the right or bottom borders.
        // The easiest workaround I've found for this is to draw it once using
        // a completely transparent pen, which will cause the QStaticText class to
        // know how wide it is; then we can recalculate the offsets and draw it
        // again with a visible pen.
        QPen invisPen(QBrush(Qt::transparent), 1);
        painter->setPen(invisPen);
        PaintText(painter);
        has_painted_ = true;
      }
      QPen pen(QBrush(color_), 1);
      painter->setPen(pen);
      PaintText(painter);

      painter->restore();
      PrintInfo("OK");
    } else {
      PrintWarning("No messages received.");
    }
  }

  void StringPlugin::PaintText(QPainter* painter)
  {
    // Calculate the correct offsets and dimensions
    int x_offset = offset_x_;
    int y_offset = offset_y_;
    if (units_ == PERCENT)
    {
      x_offset = static_cast<int>(static_cast<float>(offset_x_ * canvas_->width()) / 100.0);
      y_offset = static_cast<int>(static_cast<float>(offset_y_ * canvas_->height()) / 100.0);
    }

    int right = static_cast<int>(
                  static_cast<float>(canvas_->width()) - message_.size().width()) - x_offset;
    int bottom = static_cast<int>(
                  static_cast<float>(canvas_->height()) - message_.size().height()) - y_offset;
    int yCenter = static_cast<int>(
                  static_cast<float>(canvas_->height()) / 2.0 - message_.size().height()/2.0);
    int xCenter = static_cast<int>(
                  static_cast<float>(canvas_->width()) / 2.0 - message_.size().width()/2.0);

    QPoint ulPoint;

    switch (anchor_)
    {
      case TOP_LEFT:
        ulPoint.setX(x_offset);
        ulPoint.setY(y_offset);
        break;
      case TOP_CENTER:
        ulPoint.setX(xCenter);
        ulPoint.setY(y_offset);
        break;
      case TOP_RIGHT:
        ulPoint.setX(right);
        ulPoint.setY(y_offset);
        break;
      case CENTER_LEFT:
        ulPoint.setX(x_offset);
        ulPoint.setY(yCenter);
        break;
      case CENTER:
        ulPoint.setX(xCenter);
        ulPoint.setY(yCenter);
        break;
      case CENTER_RIGHT:
        ulPoint.setX(right);
        ulPoint.setY(yCenter);
        break;
      case BOTTOM_LEFT:
        ulPoint.setX(x_offset);
        ulPoint.setY(bottom);
        break;
      case BOTTOM_CENTER:
        ulPoint.setX(xCenter);
        ulPoint.setY(bottom);
        break;
      case BOTTOM_RIGHT:
        ulPoint.setX(right);
        ulPoint.setY(bottom);
        break;
    }
    painter->drawStaticText(ulPoint, message_);
  }

  void StringPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    LoadQosConfig(node, qos_);
    if (node[TOPIC_KEY])
    {
      ui_.topic->setText(QString(node[TOPIC_KEY].as<std::string>().c_str()));
      TopicEdited();
    }

    if (node[FONT_KEY])
    {
      std::string saved_font = node[FONT_KEY].as<std::string>();
      bool ok = font_.fromString(QString(saved_font.c_str()));

      // Revert to the default system font and size if we fail to load
      // the stored font setting
      if (!ok)
      {
        RCLCPP_ERROR(
          node_->get_logger(),
          "Unable to load saved font: %s, reverting to default font",
          saved_font.c_str());
        font_ = QFont();
        font_.setPointSize(DEFAULT_FONT_SIZE);
      }

      // Make a copy of the display font here so that we can change the size
      // to a consistent value, and update the UI with that information
      QFont button_font = font_;
      button_font.setPointSize(DEFAULT_FONT_SIZE); 
      ui_.font_button->setFont(button_font);
      ui_.font_button->setText(button_font.family());
    }

    if (node[COLOR_KEY])
    {
      color_ = QColor(node[COLOR_KEY].as<std::string>().c_str());
      ui_.color->setColor(QColor(color_.name().toStdString().c_str()));
    }

    if (node[ANCHOR_KEY])
    {
      std::string anchor = node[ANCHOR_KEY].as<std::string>();
      ui_.anchor->setCurrentIndex(ui_.anchor->findText(anchor.c_str()));
      SetAnchor(anchor.c_str());
    }

    if (node[UNITS_KEY])
    {
      std::string units = node[UNITS_KEY].as<std::string>();
      ui_.units->setCurrentIndex(ui_.units->findText(units.c_str()));
      SetUnits(units.c_str());
    }

    if (node[OFFSET_X_KEY])
    {
      offset_x_ = node[OFFSET_X_KEY].as<int>();
      ui_.offsetx->setValue(offset_x_);
    }

    if (node[OFFSET_Y_KEY])
    {
      offset_y_ = node[OFFSET_Y_KEY].as<int>();
      ui_.offsety->setValue(offset_y_);
    }
  }

  void StringPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    emitter << YAML::Key << FONT_KEY << YAML::Value << font_.toString().toStdString();
    emitter << YAML::Key << COLOR_KEY << YAML::Value << color_.name().toStdString();
    emitter << YAML::Key << TOPIC_KEY << YAML::Value << ui_.topic->text().toStdString();
    emitter << YAML::Key << ANCHOR_KEY << YAML::Value << AnchorToString(anchor_);
    emitter << YAML::Key << UNITS_KEY << YAML::Value << UnitsToString(units_);
    emitter << YAML::Key << OFFSET_X_KEY << YAML::Value << offset_x_;
    emitter << YAML::Key << OFFSET_Y_KEY << YAML::Value << offset_y_;
    SaveQosConfig(emitter, qos_);
  }

  QWidget* StringPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);
    return config_widget_;
  }

  void StringPlugin::PrintError(const std::string& message)
  {
    PrintErrorHelper(ui_.status, message);
  }

  void StringPlugin::PrintInfo(const std::string& message)
  {
    PrintInfoHelper(ui_.status, message);
  }

  void StringPlugin::PrintWarning(const std::string& message)
  {
    PrintWarningHelper(ui_.status, message);
  }

  void StringPlugin::SelectColor()
  {
    color_ = ui_.color->color();
  }

  void StringPlugin::SelectFont()
  {
    bool ok;
    QFont font = QFontDialog::getFont(&ok, font_, canvas_);
    // Update the UI if a font was chosen
    if (ok)
    {
      font_ = font;
      message_.prepare(QTransform(), font_);
      // Override the user's font size so the button size stays consistent
      QFont font = font_;
      font.setPointSize(DEFAULT_FONT_SIZE);
      ui_.font_button->setFont(font);
      ui_.font_button->setText(font.family());
    }
  }

  void StringPlugin::SelectTopic()
  {
    auto [topic, qos] = SelectTopicDialog::selectTopic(
      node_,
      "std_msgs/msg/String",
      qos_);

    if (!topic.empty())
    {
      connectCallback(topic, qos);
    }
  }

  void StringPlugin::TopicEdited()
  {
    std::string topic = ui_.topic->text().trimmed().toStdString();
    connectCallback(topic, qos_);
  }

  void StringPlugin::connectCallback(const std::string& topic, const rmw_qos_profile_t& qos)
  {
    ui_.topic->setText(QString::fromStdString(topic));
    if ((topic != topic_) || !qosEqual(qos, qos_))
    {
      initialized_ = false;
      has_message_ = false;
      PrintWarning("No messages received.");

      string_sub_.reset();

      topic_ = topic;
      qos_ = qos;
      if (!topic.empty())
      {
        try
        {
          string_sub_ = node_->create_subscription<std_msgs::msg::String>(topic_,
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos),
              [this](const std_msgs::msg::String::ConstSharedPtr str) {
            SetText(QString(str->data.c_str()));
          });
        }
        catch(...)
        {
          RCLCPP_ERROR(node_->get_logger(),
            "Exception thrown while subscribing to standard string: %s",
            topic_.c_str());
        }

        try
        {
          string_stamped_sub_ = node_->create_subscription<marti_common_msgs::msg::StringStamped>(topic_,
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos),
              [this](const marti_common_msgs::msg::StringStamped::ConstSharedPtr str) {
            SetText(QString(str->value.c_str()));
          });
        }
        catch(...)
        {
          RCLCPP_ERROR(node_->get_logger(),
            "Exception thrown while subscribing to Marti stamped string: %s",
            topic_.c_str());
        }
         
        RCLCPP_INFO(node_->get_logger(), "Subscribing to %s", topic_.c_str());
      }
    }
  }

  void StringPlugin::SetText(const QString& text)
  {
    message_.setText(text);
    message_.prepare(QTransform(), font_);

    has_message_ = true;
    has_painted_ = false;
    initialized_ = true;
  }

  void StringPlugin::SetAnchor(QString anchor)
  {
    if (anchor == "top left")
    {
      anchor_ = TOP_LEFT;
    } else if (anchor == "top center") {
      anchor_ = TOP_CENTER;
    } else if (anchor == "top right") {
      anchor_ = TOP_RIGHT;
    } else if (anchor == "center left") {
      anchor_ = CENTER_LEFT;
    } else if (anchor == "center") {
      anchor_ = CENTER;
    } else if (anchor == "center right") {
      anchor_ = CENTER_RIGHT;
    } else if (anchor == "bottom left") {
      anchor_ = BOTTOM_LEFT;
    } else if (anchor == "bottom center") {
      anchor_ = BOTTOM_CENTER;
    } else if (anchor == "bottom right") {
      anchor_ = BOTTOM_RIGHT;
    }
  }

  void StringPlugin::SetUnits(QString units)
  {
    if (units == "pixels")
    {
      units_ = PIXELS;
    } else if (units == "percent") {
      units_ = PERCENT;
    }
  }

  void StringPlugin::SetOffsetX(int offset)
  {
    offset_x_ = offset;
  }

  void StringPlugin::SetOffsetY(int offset)
  {
    offset_y_ = offset;
  }

  std::string StringPlugin::AnchorToString(StringPlugin::Anchor anchor)
  {
    std::string anchor_string = "top left";

    if (anchor == TOP_LEFT)
    {
      anchor_string = "top left";
    } else if (anchor == TOP_CENTER) {
      anchor_string = "top center";
    } else if (anchor == TOP_RIGHT) {
      anchor_string = "top right";
    } else if (anchor == CENTER_LEFT) {
      anchor_string = "center left";
    } else if (anchor == CENTER) {
      anchor_string = "center";
    } else if (anchor == CENTER_RIGHT) {
      anchor_string = "center right";
    } else if (anchor == BOTTOM_LEFT) {
      anchor_string = "bottom left";
    } else if (anchor == BOTTOM_CENTER) {
      anchor_string = "bottom center";
    } else if (anchor == BOTTOM_RIGHT) {
      anchor_string = "bottom right";
    }

    return anchor_string;
  }

  std::string StringPlugin::UnitsToString(StringPlugin::Units units)
  {
    std::string units_string = "pixels";

    if (units == PIXELS)
    {
      units_string = "pixels";
    } else if (units == PERCENT) {
      units_string = "percent";
    }

    return units_string;
  }
}   // namespace mapviz_plugins
