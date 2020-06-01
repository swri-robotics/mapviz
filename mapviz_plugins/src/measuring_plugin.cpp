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
  is_mouse_down_(false),
  max_ms_(Q_INT64_C(500)),
  max_distance_(2.0),
  map_canvas_(NULL)
{
  ui_.setupUi(config_widget_);
  ui_.main_color->setColor(Qt::black);
  ui_.bkgnd_color->setColor(Qt::white);

  QObject::connect(ui_.clear, SIGNAL(clicked()), this,
                   SLOT(Clear()));
  QObject::connect(ui_.show_measurements, SIGNAL(toggled(bool)), this,
                   SLOT(MeasurementsToggled(bool)));
  QObject::connect(ui_.show_bkgnd_color, SIGNAL(toggled(bool)), this,
                   SLOT(BkgndColorToggled(bool)));
  QObject::connect(ui_.font_size, SIGNAL(valueChanged(int)), this,
                   SLOT(FontSizeChanged(int)));
  QObject::connect(ui_.alpha, SIGNAL(valueChanged(double)), this,
                   SLOT(AlphaChanged(double)));
  connect(ui_.main_color, SIGNAL(colorEdited(const QColor &)), this, SLOT(DrawIcon()));
  connect(ui_.bkgnd_color, SIGNAL(colorEdited(const QColor &)), this, SLOT(DrawIcon()));
  
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
  if(!this->Visible())
  {
    ROS_DEBUG("Ignoring mouse event, since measuring plugin is hidden");
    return false;
  }

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
  for (size_t i = 0; i < vertices_.size(); i++)
  {
      tf::Vector3 vertex = vertices_[i];
      if (last_position_ != tf::Vector3(0,0,0))
      {
          distance_instant = last_position_.distance(vertex);
          distance_sum = distance_sum + distance_instant;
          measurements_.push_back(distance_instant);
      }
      last_position_ = vertex;
  }
  measurements_.push_back(distance_sum);

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
    DistanceCalculation(); //function to calculate distance
    return true;
  }// Let other plugins process this event too
  return false;
}

void MeasuringPlugin::Draw(double x, double y, double scale)
{
  glLineWidth(1);
  const QColor color = ui_.main_color->color();
  glColor4d(color.redF(), color.greenF(), color.blueF(), ui_.alpha->value()/2.0);
  glBegin(GL_LINE_STRIP);

  for (const auto& vertex: vertices_)
  {
    glVertex2d(vertex.x(), vertex.y());
  }

  glEnd();

  glBegin(GL_LINES);

  glColor4d(color.redF(), color.greenF(), color.blueF(), ui_.alpha->value()/2.0);

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
  bool show_measurements = ui_.show_measurements->isChecked();
  if (!show_measurements || vertices_.empty())
  {
    return;
  }

  QTransform tf = painter->worldTransform();
  QFont font("Helvetica", ui_.font_size->value());
  painter->setFont(font);
  painter->save();
  painter->resetTransform();

  //set the draw color for the text to be the same as the rest
  QColor color = ui_.main_color->color();
  double alpha = ui_.alpha->value()*2.0 < 1.0 ? ui_.alpha->value()*2.0 : 1.0;
  color.setAlphaF(alpha);
  QPen pen(QBrush(color), 1);
  painter->setPen(pen);

  const QRectF qrect = QRectF(0, 0, 0, 0);
  MeasurementBox mb;
  std::vector<MeasurementBox> tags;

  //(midpoint positioned) measurements
  for (int i=0; i<vertices_.size()-1; i++)
  {
    tf::Vector3 v1 = vertices_[i];
    tf::Vector3 v2 = vertices_[i+1];

    mb.string.setNum(measurements_[i], 'g', 5);
    mb.string.prepend(" ");
    mb.string.append(" m ");
    //drawText used here to get correct mb.rect size
    painter->drawText(qrect, 0, mb.string, &mb.rect);
    mb.rect.moveTopLeft(tf.map(QPointF((v1.x()+v2.x())/2, (v1.y()+v2.y())/2)));
    tags.push_back(mb);
  }
  //(endpoint positioned) total dist
  mb.string.setNum(measurements_.back(), 'g', 5);
  mb.string.prepend(" Total: ");
  mb.string.append(" m ");
  painter->drawText(qrect, 0, mb.string, &mb.rect);
  mb.rect.moveTopLeft(tf.map(QPointF(vertices_.back().x(), vertices_.back().y())));
  tags.push_back(mb);

  //prevent text overlapping
  for (int i=0; i<tags.size(); i++)
  {
    for (int j=0; j<tags.size(); j++)
    {
      if (i != j && tags[i].rect.intersects(tags[j].rect))
      {
        QRectF overlap = tags[i].rect.intersected(tags[j].rect);
        if (tags[i].rect.y() > tags[j].rect.y())
        {
          tags[i].rect.moveTop(tags[i].rect.y() + overlap.height());
        }
        else
        {
          tags[i].rect.moveTop(tags[i].rect.y() - overlap.height());
        }
      }
    }
  }

  //paint tags
  for (const auto& tag: tags)
  {
    if (ui_.show_bkgnd_color->isChecked())
    {
      color = ui_.bkgnd_color->color();
      color.setAlphaF(ui_.alpha->value());
      painter->fillRect(tag.rect, color);
      painter->drawRect(tag.rect);
    }
    painter->drawText(tag.rect, tag.string);
  }
  painter->restore();
}

void MeasuringPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
{
  if (node["main_color"])
  {            
    std::string color;
    node["main_color"] >> color;
    ui_.main_color->setColor(QColor(color.c_str()));
  }

  if (node["bkgnd_color"])
  {            
    std::string color;
    node["bkgnd_color"] >> color;
    ui_.bkgnd_color->setColor(QColor(color.c_str()));
  }

  if (node["show_bkgnd_color"])
  {
    bool show_bkgnd_color = false;
    node["show_bkgnd_color"] >> show_bkgnd_color;
    ui_.show_bkgnd_color->setChecked(show_bkgnd_color);
    BkgndColorToggled(show_bkgnd_color);
  }

  if (node["show_measurements"])
  {
    bool show_measurements = false;
    node["show_measurements"] >> show_measurements;
    ui_.show_measurements->setChecked(show_measurements);
    MeasurementsToggled(show_measurements);
  }

  if (node["font_size"])
  {
    int font_size;
    node["font_size"] >> font_size;
    ui_.font_size->setValue(font_size);
    FontSizeChanged(font_size);
  }

  if (node["alpha"])
  {
    double alpha;
    node["alpha"] >> alpha;
    ui_.alpha->setValue(alpha);
    AlphaChanged(alpha);
  }
}

void MeasuringPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
{
  emitter << YAML::Key << "main_color" << YAML::Value << ui_.main_color->color().name().toStdString();
  emitter << YAML::Key << "bkgnd_color" << YAML::Value << ui_.bkgnd_color->color().name().toStdString();
  emitter << YAML::Key << "show_bkgnd_color" << YAML::Value << ui_.show_bkgnd_color->isChecked();
  emitter << YAML::Key << "show_measurements" << YAML::Value << ui_.show_measurements->isChecked();
  emitter << YAML::Key << "font_size" << YAML::Value << ui_.font_size->value();
  emitter << YAML::Key << "alpha" << YAML::Value << ui_.alpha->value();
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
