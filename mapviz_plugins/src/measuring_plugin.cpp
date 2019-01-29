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

// QT libraries
#include <QDateTime>
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

namespace stu = swri_transform_util;

namespace mapviz_plugins
{

MeasuringPlugin::MeasuringPlugin():
  config_widget_(new QWidget()),
  print_Flag(0),
  selected_point_(-1),
  is_mouse_down_(false),
  max_ms_(Q_INT64_C(500)),
  max_distance_(2.0),
  map_canvas_(NULL)
  //last_position_(tf::Vector3(0.0,0.0,0.0))
{
  ui_.setupUi(config_widget_);

  QObject::connect(ui_.selectframe, SIGNAL(clicked()),
                   this, SLOT(SelectFrame()));
  QObject::connect(ui_.frame, SIGNAL(editingFinished()),
                   this, SLOT(FrameEdited()));
  QObject::connect(ui_.clear, SIGNAL(clicked()), this,
                   SLOT(Clear()));

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

void MeasuringPlugin::Clear()
{
  vertices_.clear();
  transformed_vertices_.clear();
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
  selected_point_ = -1;
  int closest_point = 0;
  double closest_distance = std::numeric_limits<double>::max();
#if QT_VERSION >= 0x050000
  QPointF point = event->localPos();
#else
  QPointF point = event->posF();
#endif
  stu::Transform transform;
  ROS_DEBUG("Map point: %f %f", point.x(), point.y());

  std::string frame = ui_.frame->text().toStdString();
  if (tf_manager_->GetTransform(target_frame_, frame, transform))
  {
    for (size_t i = 0; i < vertices_.size(); i++)
    {
      tf::Vector3 vertex = vertices_[i];
      vertex = transform * vertex;
      QPointF transformed = map_canvas_->FixedFrameToMapGlCoord(QPointF(vertex.x(), vertex.y()));

      double distance = QLineF(transformed, point).length();

      if (distance < closest_distance)
      {
        closest_distance = distance;
        closest_point = static_cast<int>(i);
      }
    }
  }
  if (event->button() == Qt::LeftButton)
  {
    if (closest_distance < 15)
    {
      selected_point_ = closest_point;
      return true;
    }
    else
    {
      is_mouse_down_ = true;
#if QT_VERSION >= 0x050000
      mouse_down_pos_ = event->localPos();
#else
      mouse_down_pos_ = event->posF();
#endif
      mouse_down_time_ = QDateTime::currentMSecsSinceEpoch();
      return false;
    }
  }
  else if (event->button() == Qt::RightButton)
  {
    if (closest_distance < 15)
    {
      vertices_.erase(vertices_.begin() + closest_point);
      transformed_vertices_.resize(vertices_.size());
      return true;
    }
  }

  // Let other plugins process this event too
  return false;
}

bool MeasuringPlugin::handleMouseRelease(QMouseEvent* event)
{
    std::string frame = ui_.frame->text().toStdString();
    stu::Transform transform;
    ROS_INFO("Selected Point :%d", selected_point_);
    if (selected_point_ >= 0 && static_cast<size_t>(selected_point_) < vertices_.size())
    {
         double test = 1;
         ROS_INFO("Debugging Flag :%d", test);
#if QT_VERSION >= 0x050000
      QPointF point = event->localPos();
#else
      QPointF point = event->posF();
#endif
      if (tf_manager_->GetTransform(frame, target_frame_, transform))
      {
        QPointF transformed = map_canvas_->MapGlCoordToFixedFrame(point);
        tf::Vector3 position(transformed.x(), transformed.y(), 0.0);
        position = transform * position;
        vertices_[selected_point_].setX(position.x());
        vertices_[selected_point_].setY(position.y());
      }

      selected_point_ = -1;
      return true;
    }
    else if (is_mouse_down_)
    {
#if QT_VERSION >= 0x050000
      qreal distance = QLineF(mouse_down_pos_, event->localPos()).length();
#else
      qreal distance = QLineF(mouse_down_pos_, event->posF()).length();
#endif
      qint64 msecsDiff = QDateTime::currentMSecsSinceEpoch() - mouse_down_time_;

      // Only fire the event if the mouse has moved less than the maximum distance
      // and was held for shorter than the maximum time..  This prevents click
      // events from being fired if the user is dragging the mouse across the map
      // or just holding the cursor in place.
      if (msecsDiff < max_ms_ && distance <= max_distance_)
      {
#if QT_VERSION >= 0x050000
        QPointF point = event->localPos();
#else
        QPointF point = event->posF();
#endif
        if (frame.empty())
        {
          frame = target_frame_;
        }

        // Frames get confusing. The `target_frame_` member is set by the "Fixed
        // Frame" combobox in the GUI. When we transform the map coordinate to the
        // fixed frame, we get it in the `target_frame_` frame.
        //
        // Then we translate from that frame into *our* target frame, `frame`.
        double distance_instant = -1; //measurement between last two points
        double distance_sum = 0; //sum of distance from all points
        tf::Vector3 last_position_(0,0,0);
       /*if (tf_manager_->GetTransform(frame, target_frame_, transform))
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
        } */

        ROS_DEBUG("Transformed point in frame '%s': %f %f", frame.c_str(), point.x(), point.y());

        QPointF transformed = map_canvas_->MapGlCoordToFixedFrame(point);
        //ROS_INFO("mouse point at %f, %f -> %f, %f", point.x(), point.y(), transformed.x(), transformed.y());

        stu::Transform transform;
        tf::Vector3 position(transformed.x(), transformed.y(), 0.0);
        vertices_.push_back(position);
        transformed_vertices_.resize(vertices_.size());
        ROS_INFO("vertices_ size:%d", vertices_.size());
        ROS_INFO("Adding vertex at %lf, %lf %s", position.x(), position.y(), frame.c_str());


        if (tf_manager_->GetTransform(target_frame_, frame, transform))
        {
          for (size_t i = 0; i < vertices_.size(); i++)
          {
            tf::Vector3 vertex = vertices_[i];
            //vertex = transform * vertex;
           // QPointF transformed = map_canvas_->FixedFrameToMapGlCoord(QPointF(vertex.x(), vertex.y()));
            if (last_position_ != tf::Vector3(0,0,0))
            {
                distance_instant = last_position_.distance(vertex);
                ROS_INFO("distance %f", distance_instant);
                distance_sum = distance_sum + distance_instant;
                ROS_INFO("Total Distance %f", distance_sum);
            }
            last_position_ = vertex;
            ROS_INFO("Adding VERTEX at %lf, %lf %s", vertex.x(), vertex.y(), frame.c_str());
           }
        }

        QString new_point;
        QTextStream stream(&new_point);
        stream.setRealNumberPrecision(4);

        if (distance_instant > 0.0)
        {
          stream << distance_instant << " meters";
        }

        ui_.measurement->setText(new_point);

        QString new_point2;
        QTextStream stream2(&new_point2);
        stream2.setRealNumberPrecision(4);

        if (distance_sum > 0.0)
        {
          stream2 << distance_sum << " meters";
        }

        ui_.totaldistance->setText(new_point2);
  /*      if (tf_manager_->GetTransform(frame, target_frame_, transform))
        {
          position = transform * position;
          vertices_.push_back(position);
          transformed_vertices_.resize(vertices_.size());
          ROS_INFO("Adding vertex at %lf, %lf %s", position.x(), position.y(), frame.c_str());
        } */
      }
    }
    is_mouse_down_ = false;
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
    stu::Transform transform;
    std::string frame = ui_.frame->text().toStdString();
    if (!tf_manager_->GetTransform(target_frame_, frame, transform))
    {
      return;
    }

    // Transform
    for (size_t i = 0; i < vertices_.size(); i++)
    {
      transformed_vertices_[i] = transform * vertices_[i];
    }


    glLineWidth(1);
    const QColor color = ui_.color->color();
    glColor4d(color.redF(), color.greenF(), color.blueF(), 1.0);
    glBegin(GL_LINE_STRIP);

    for (const auto& vertex: transformed_vertices_)
    {
      glVertex2d(vertex.x(), vertex.y());
    }

    glEnd();

    glBegin(GL_LINES);

    glColor4d(color.redF(), color.greenF(), color.blueF(), 0.25);

    glEnd();

    // Draw vertices
    glPointSize(9);
    glBegin(GL_POINTS);

    for (const auto& vertex: transformed_vertices_)
    {
      glVertex2d(vertex.x(), vertex.y());
    }
    glEnd();



    PrintInfo("OK");
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
