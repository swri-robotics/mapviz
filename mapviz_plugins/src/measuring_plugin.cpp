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
#include <QMouseEvent>
#include <QTextStream>
#include <QPainter>

#if QT_VERSION >= 0x050000
#include <QGuiApplication>
#else
#include <QApplication>
#endif

// ROS Libraries
#include <ros/ros.h>

// Mapviz Libraries
#include <mapviz/select_frame_dialog.h>

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mapviz_plugins::MeasuringPlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{

MeasuringPlugin::MeasuringPlugin():
  config_widget_(new QWidget()),
  selected_point_(-1),
  toggle_measurements_(true),
  is_mouse_down_(false),
  max_ms_(Q_INT64_C(500)),
  max_distance_(2.0),
  map_canvas_(NULL)
{
  ui_.setupUi(config_widget_);
  ui_.color->setColor(Qt::black);

  QObject::connect(ui_.clear, SIGNAL(clicked()), this,
                   SLOT(Clear()));
  QObject::connect(ui_.toggle_measurements, SIGNAL(clicked()), this,
                   SLOT(ToggleMeasurements()));
  connect(ui_.color, SIGNAL(colorEdited(const QColor &)), this, SLOT(DrawIcon()));
  
#if QT_VERSION >= 0x050000
  ui_.measurement->setText(tr("Click on the map. Distance between clicks will appear here"));
  ui_.totaldistance->setText(tr("Click on the map. Total distance between clicks will appear here"));
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
  measurements_.clear();
  ui_.measurement->setText(tr("Click on the map. Distance between clicks will appear here"));
  ui_.totaldistance->setText(tr("Click on the map. Total distance between clicks will appear here"));
}

void MeasuringPlugin::ToggleMeasurements()
{
  toggle_measurements_ = !toggle_measurements_;
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
  ROS_DEBUG("Map point: %f %f", point.x(), point.y());
  for (size_t i = 0; i < vertices_.size(); i++)
  {
    tf::Vector3 vertex = vertices_[i];
    QPointF transformed = map_canvas_->FixedFrameToMapGlCoord(QPointF(vertex.x(), vertex.y()));

    double distance = QLineF(transformed, point).length();

    if (distance < closest_distance)
    {
      closest_distance = distance;
      closest_point = static_cast<int>(i);
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
      DistanceCalculation(); //function to calculate distance
      return true;
    }
  }

  // Let other plugins process this event too
  return false;
}

bool MeasuringPlugin::handleMouseRelease(QMouseEvent* event)
{
  if (selected_point_ >= 0 && static_cast<size_t>(selected_point_) < vertices_.size())
  {
#if QT_VERSION >= 0x050000
    QPointF point = event->localPos();
#else
    QPointF point = event->posF();
#endif
    QPointF transformed = map_canvas_->MapGlCoordToFixedFrame(point);
    tf::Vector3 position(transformed.x(), transformed.y(), 0.0);
    vertices_[selected_point_].setX(position.x());
    vertices_[selected_point_].setY(position.y());

    DistanceCalculation();

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

      QPointF transformed = map_canvas_->MapGlCoordToFixedFrame(point);
      tf::Vector3 position(transformed.x(), transformed.y(), 0.0);
      vertices_.push_back(position);
      DistanceCalculation(); //call to calculate distance
    }
  }
  is_mouse_down_ = false;
  // Let other plugins process this event too
  return false;
}

void MeasuringPlugin::DistanceCalculation()
{
  double distance_instant = -1; //measurement between last two points
  double distance_sum = 0; //sum of distance from all points
  tf::Vector3 last_position_(0,0,0);
  std::string frame = target_frame_;
  measurements_.clear();
  measurements_.push_back(0); // reserve head of vector for distance_sum
  for (size_t i = 0; i < vertices_.size(); i++)
  {
      tf::Vector3 vertex = vertices_[i];
      if (last_position_ != tf::Vector3(0,0,0))
      {
          distance_instant = last_position_.distance(vertex);
          distance_sum = distance_sum + distance_instant;
          measurements_[0] = distance_sum;
          measurements_.push_back(distance_instant);
      }
      last_position_ = vertex;
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
}

bool MeasuringPlugin::handleMouseMove(QMouseEvent* event)
{
  if (selected_point_ >= 0 && static_cast<size_t>(selected_point_) < vertices_.size())
  {
#if QT_VERSION >= 0x050000
    QPointF point = event->localPos();
#else
    QPointF point = event->posF();
#endif
    std::string frame = target_frame_;
    QPointF transformed = map_canvas_->MapGlCoordToFixedFrame(point);
    tf::Vector3 position(transformed.x(), transformed.y(), 0.0);
    vertices_[selected_point_].setY(position.y());
    vertices_[selected_point_].setX(position.x());
    return true;
  }// Let other plugins process this event too
  return false;
}

void MeasuringPlugin::Draw(double x, double y, double scale)
{
  glLineWidth(1);
  const QColor color = ui_.color->color();
  glColor4d(color.redF(), color.greenF(), color.blueF(), 1.0);
  glBegin(GL_LINE_STRIP);

  for (const auto& vertex: vertices_)
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

  for (const auto& vertex: vertices_)
  {
    glVertex2d(vertex.x(), vertex.y());
  }
  glEnd();

  PrintInfo("OK");
}

void MeasuringPlugin::Paint(QPainter* painter, double x, double y, double scale)
{
  if (!toggle_measurements_)
  {
    return;
  }

  QTransform tf = painter->worldTransform();
  QFont font("Helvetica", 10);
  painter->setFont(font);
  painter->save();
  painter->resetTransform();

  //set the draw color for the text to be the same as the rest
  QPen pen(QBrush(ui_.color->color()), 1);
  painter->setPen(pen);

  QPointF qpoint;
  std::vector<QPointF> qpoints;
  QString distance;

  //only draw if measurements are active
  if (!measurements_.empty() && !vertices_.empty())
  {
    qpoint = tf.map(QPointF(vertices_.back().x(),
                            vertices_.back().y()));
    qpoints.push_back(qpoint);

    //large rect for total distance tag
    QRectF qrect(qpoint, QSizeF(145.0, 20.0));
    distance.setNum(measurements_[0], 'g', 5);
    distance.append(" m");
    distance.prepend("Total distance: ");

    //medium rect for 0 distance tag
    if (measurements_.size()==1)
    {
      qrect.setTopLeft(qpoint);
      qrect.setSize(QSizeF(115.0, 20.0));
    }
    painter->drawText(qrect, distance);
    painter->drawRect(qrect);
  }

  for (int i=1; i<vertices_.size(); i++)
  {
    //draw measurement at midpoint
    tf::Vector3 v1 = vertices_[i];
    tf::Vector3 v2 = vertices_[i-1];
    qpoint = tf.map(QPointF((v1.x()+v2.x())/2,
                            (v1.y()+v2.y())/2));
    qpoints.push_back(qpoint);

    //prevent any tags from overlapping
    for (int j=0; j<i; j++)
    {
      double x_offset = qpoints[j].x() - qpoint.x();
      double y_offset = qpoints[j].y() - qpoint.y();
      if (((j == 0 && fabs(x_offset) < 145) || fabs(x_offset) < 80) 
            && fabs(y_offset) < 20)
      {
        if (y_offset > 0)
        {
          qpoint.setY(qpoint.y() - (20 - y_offset));
        }
        else
        {
          qpoint.setY(qpoint.y() + (20 + y_offset));
        }
      }
    }
    //small rect for measurement tag
    QRectF qrect(qpoint, QSizeF(80.0, 20.0));
    distance.setNum(measurements_[i], 'g', 5);
    distance.append(" m");
    painter->drawText(qrect, distance);
    painter->drawRect(qrect);
  }
  painter->restore();
}

void MeasuringPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
{
  if (node["color"])
  {            
    std::string color;
    node["color"] >> color;
    ui_.color->setColor(QColor(color.c_str()));
  }
}

void MeasuringPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
{
  emitter << YAML::Key << "color" << YAML::Value << ui_.color->color().name().toStdString();
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
