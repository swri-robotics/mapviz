// *****************************************************************************
//
// Copyright (c) 2018, Southwest Research Institute® (SwRI®)
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

#include <mapviz_plugins/measuring_plugin.h>
#include <mapviz/mapviz_plugin.h>

#include <QClipboard>
#include <QMouseEvent>
#include <QTextStream>

#if QT_VERSION >= 0x050000
#include <QGuiApplication>
#else
#include <QApplication>
#endif

// ROS Libraries
#include <ros/ros.h>

// Mapviz Libraries
#include <mapviz/select_frame_dialog.h>

//
#include <swri_transform_util/transform.h>

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mapviz_plugins::MeasuringPlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{

MeasuringPlugin::MeasuringPlugin()
  : config_widget_(new QWidget()),
  map_canvas_(NULL),
  last_position_(tf::Vector3(0.0,0.0,0.0))
{
  ui_.setupUi(config_widget_);

  QObject::connect(ui_.selectframe, SIGNAL(clicked()),
                   this, SLOT(SelectFrame()));
  QObject::connect(ui_.frame, SIGNAL(editingFinished()),
                   this, SLOT(FrameEdited()));

#if QT_VERSION >= 0x050000
  ui_.measurement->setText(tr("Click on the map; distance between clicks will appear here"));
#endif
}

MeasuringPlugin::~MeasuringPlugin()
{
  if (map_canvas_)
  {
    map_canvas_->removeEventFilter(this);
  }
}

QWidget* MeasuringPlugin::GetConfigWidget(QWidget* parent)
{
  config_widget_->setParent(parent);

  return config_widget_;
}

bool MeasuringPlugin::Initialize(QGLWidget* canvas)
{
  map_canvas_ = static_cast< mapviz::MapCanvas* >(canvas);
  map_canvas_->installEventFilter(this);

  initialized_ = true;
  PrintInfo("OK");

  return true;
}

bool MeasuringPlugin::eventFilter(QObject* object, QEvent* event)
{
  switch (event->type())
  {
    case QEvent::MouseButtonPress:
      return handleMousePress(static_cast< QMouseEvent* >(event));
    case QEvent::MouseButtonRelease:
      return handleMouseRelease(static_cast< QMouseEvent* >(event));
    case QEvent::MouseMove:
      return handleMouseMove(static_cast< QMouseEvent* >(event));
    default:
      return false;
  }
}

bool MeasuringPlugin::handleMousePress(QMouseEvent* event)
{
#if QT_VERSION >= 0x050000
  QPointF point = event->localPos();
#else
  QPointF point = event->posF();
#endif
  ROS_DEBUG("Map point: %f %f", point.x(), point.y());

  swri_transform_util::Transform transform;
  std::string frame = ui_.frame->text().toStdString();
  if (frame.empty())
  {
    frame = target_frame_;
  }

  // Frames get confusing. The `target_frame_` member is set by the "Fixed
  // Frame" combobox in the GUI. When we transform the map coordinate to the
  // fixed frame, we get it in the `target_frame_` frame.
  //
  // Then we translate from that frame into *our* target frame, `frame`.
  double distance = -1.0;
  if (tf_manager_.GetTransform(frame, target_frame_, transform))
  {
    ROS_DEBUG("Transforming from fixed frame '%s' to (plugin) target frame '%s'",
              target_frame_.c_str(),
              frame.c_str());
    QPointF transformed = map_canvas_->MapGlCoordToFixedFrame(point);
    ROS_DEBUG("Point in fixed frame: %f %f", transformed.x(), transformed.y());
    tf::Vector3 position(transformed.x(), transformed.y(), 0.0);
    position = transform * position;
    point.setX(position.x());
    point.setY(position.y());

    if (last_position_ != tf::Vector3(0.0,0.0,0.0))
    {
      distance = last_position_.distance(position);
    }

    last_position_ = position;

    PrintInfo("OK");
  }
  else
  {
    QString warning;
    QTextStream(&warning) << "No available transform from '" << QString::fromStdString(target_frame_) << "' to '" << QString::fromStdString(frame) << "'";
    PrintWarning(warning.toStdString());
    return false;
  }


  ROS_DEBUG("Transformed point in frame '%s': %f %f", frame.c_str(), point.x(), point.y());
  QString new_point;
  QTextStream stream(&new_point);
  stream.setRealNumberPrecision(4);

  if (distance >= 0.0)
  {
    stream << distance << " meters";
  }

  ui_.measurement->setText(new_point);

  // Let other plugins process this event too
  return false;
}

bool MeasuringPlugin::handleMouseRelease(QMouseEvent* event)
{
  // Let other plugins process this event too
  return false;
}

bool MeasuringPlugin::handleMouseMove(QMouseEvent* event)
{
  // Let other plugins process this event too
  return false;
}

void MeasuringPlugin::SelectFrame()
{
  std::string frame = mapviz::SelectFrameDialog::selectFrame(tf_);
  if (!frame.empty())
  {
    ui_.frame->setText(QString::fromStdString(frame));
    FrameEdited();
  }
}

void MeasuringPlugin::FrameEdited()
{
  ROS_INFO("Setting target frame to %s", ui_.frame->text().toStdString().c_str());
}

void MeasuringPlugin::Draw(double x, double y, double scale)
{
}

void MeasuringPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
{
  if (node["frame"])
  {
    std::string frame;
    node["frame"] >> frame;
    ui_.frame->setText(QString::fromStdString(frame));
  }
}

void MeasuringPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
{
  std::string frame = ui_.frame->text().toStdString();
  emitter << YAML::Key << "frame" << YAML::Value << frame;
}

void MeasuringPlugin::PrintError(const std::string& message)
{
  PrintErrorHelper(ui_.status, message, 1.0);
}

void MeasuringPlugin::PrintInfo(const std::string& message)
{
  PrintInfoHelper(ui_.status, message, 1.0);
}

void MeasuringPlugin::PrintWarning(const std::string& message)
{
  PrintWarningHelper(ui_.status, message, 1.0);
}

} // namespace mapviz_plugins
