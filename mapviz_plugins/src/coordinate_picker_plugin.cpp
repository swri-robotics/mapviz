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

#include <mapviz_plugins/coordinate_picker_plugin.h>
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
PLUGINLIB_EXPORT_CLASS(mapviz_plugins::CoordinatePickerPlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{

CoordinatePickerPlugin::CoordinatePickerPlugin()
  : config_widget_(new QWidget()),
  map_canvas_(NULL),
  copy_on_click_(false)
{
  ui_.setupUi(config_widget_);

  QObject::connect(ui_.selectframe, SIGNAL(clicked()),
                   this, SLOT(SelectFrame()));
  QObject::connect(ui_.frame, SIGNAL(editingFinished()),
                   this, SLOT(FrameEdited()));
  QObject::connect(ui_.copyCheckBox, SIGNAL(stateChanged(int)),
                   this, SLOT(ToggleCopyOnClick(int)));
  QObject::connect(ui_.clearListButton, SIGNAL(clicked()),
                   this, SLOT(ClearCoordList()));

#if QT_VERSION >= 0x050000
  ui_.coordTextEdit->setPlaceholderText(tr("Click on the map; coordinates appear here"));
#endif
}

CoordinatePickerPlugin::~CoordinatePickerPlugin()
{
  if (map_canvas_)
  {
    map_canvas_->removeEventFilter(this);
  }
}

QWidget* CoordinatePickerPlugin::GetConfigWidget(QWidget* parent)
{
  config_widget_->setParent(parent);

  return config_widget_;
}

bool CoordinatePickerPlugin::Initialize(QGLWidget* canvas)
{
  map_canvas_ = static_cast< mapviz::MapCanvas* >(canvas);
  map_canvas_->installEventFilter(this);

  initialized_ = true;
  PrintInfo("OK");

  return true;
}

bool CoordinatePickerPlugin::eventFilter(QObject* object, QEvent* event)
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

bool CoordinatePickerPlugin::handleMousePress(QMouseEvent* event)
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
  if (tf_manager_->GetTransform(frame, target_frame_, transform))
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
  stream << point.x() << ", " << point.y();

  if (copy_on_click_)
  {
#if QT_VERSION >= 0x050000
    QClipboard* clipboard = QGuiApplication::clipboard();
#else
    QClipboard* clipboard = QApplication::clipboard();
#endif
    clipboard->setText(new_point);
  }

  stream << " (" << QString::fromStdString(frame) << ")\n";

  ui_.coordTextEdit->setPlainText(ui_.coordTextEdit->toPlainText().prepend(new_point));

  // Let other plugins process this event too
  return false;
}

bool CoordinatePickerPlugin::handleMouseRelease(QMouseEvent* event)
{
  // Let other plugins process this event too
  return false;
}

bool CoordinatePickerPlugin::handleMouseMove(QMouseEvent* event)
{
  // Let other plugins process this event too
  return false;
}

void CoordinatePickerPlugin::SelectFrame()
{
  std::string frame = mapviz::SelectFrameDialog::selectFrame(tf_);
  if (!frame.empty())
  {
    ui_.frame->setText(QString::fromStdString(frame));
    FrameEdited();
  }
}

void CoordinatePickerPlugin::FrameEdited()
{
  ROS_INFO("Setting target frame to %s", ui_.frame->text().toStdString().c_str());
}

void CoordinatePickerPlugin::ToggleCopyOnClick(int state)
{
  switch(state)
  {
    case Qt::Checked:
      copy_on_click_ = true;
      break;
    case Qt::PartiallyChecked:
    case Qt::Unchecked:
    default:
      copy_on_click_ = false;
      break;
  }
}

void CoordinatePickerPlugin::ClearCoordList()
{
  ui_.coordTextEdit->setPlainText(QString());
}

void CoordinatePickerPlugin::Draw(double x, double y, double scale)
{
}

void CoordinatePickerPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
{
  if (node["frame"])
  {
    std::string frame;
    node["frame"] >> frame;
    ui_.frame->setText(QString::fromStdString(frame));
  }

  if (node["copy"])
  {
    bool copy;
    node["copy"] >> copy;
    if (copy)
    {
      ui_.copyCheckBox->setCheckState(Qt::Checked);
    }
    else
    {
      ui_.copyCheckBox->setCheckState(Qt::Unchecked);
    }
  }
}

void CoordinatePickerPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
{
  std::string frame = ui_.frame->text().toStdString();
  emitter << YAML::Key << "frame" << YAML::Value << frame;

  bool copy_on_click = ui_.copyCheckBox->isChecked();
  emitter << YAML::Key << "copy" << YAML::Value << copy_on_click;
}

void CoordinatePickerPlugin::PrintError(const std::string& message)
{
  PrintErrorHelper(ui_.status, message, 1.0);
}

void CoordinatePickerPlugin::PrintInfo(const std::string& message)
{
  PrintInfoHelper(ui_.status, message, 1.0);
}

void CoordinatePickerPlugin::PrintWarning(const std::string& message)
{
  PrintWarningHelper(ui_.status, message, 1.0);
}

} // namespace mapviz_plugins
