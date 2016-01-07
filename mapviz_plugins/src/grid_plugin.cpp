// *****************************************************************************
//
// Copyright (c) 2014, Southwest Research Institute® (SwRI®)
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

#include <mapviz_plugins/grid_plugin.h>

// C++ standard libraries
#include <cstdio>
#include <vector>

// QT libraries
#include <QGLWidget>
#include <QPalette>

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(
    mapviz_plugins,
    grid,
    mapviz_plugins::GridPlugin,
    mapviz::MapvizPlugin)

namespace mapviz_plugins
{
  GridPlugin::GridPlugin() :
    config_widget_(new QWidget()),
    alpha_(1.0),
    top_left_(0, 0, 0),
    size_(1),
    rows_(1),
    columns_(1),
    exit_(false),
    transformed_(false)
  {
    ui_.setupUi(config_widget_);

    ui_.color->setColor(Qt::red);
    
    // Set background white
    QPalette p(config_widget_->palette());
    p.setColor(QPalette::Background, Qt::white);
    config_widget_->setPalette(p);

    // Set status text red
    QPalette p3(ui_.status->palette());
    p3.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p3);

    QObject::connect(ui_.alpha, SIGNAL(valueChanged(double)), this, SLOT(SetAlpha(double)));
    QObject::connect(ui_.x, SIGNAL(valueChanged(double)), this, SLOT(SetX(double)));
    QObject::connect(ui_.y, SIGNAL(valueChanged(double)), this, SLOT(SetY(double)));
    QObject::connect(ui_.size, SIGNAL(valueChanged(double)), this, SLOT(SetSize(double)));
    QObject::connect(ui_.rows, SIGNAL(valueChanged(int)), this, SLOT(SetRows(int)));
    QObject::connect(ui_.columns, SIGNAL(valueChanged(int)), this, SLOT(SetColumns(int)));
    QObject::connect(ui_.frame, SIGNAL(editTextChanged(QString)), this, SLOT(SetFrame(QString)));
    connect(ui_.color, SIGNAL(colorEdited(const QColor &)), this, SLOT(DrawIcon()));
  }

  GridPlugin::~GridPlugin()
  {
    Shutdown();
  }

  void GridPlugin::Shutdown()
  {
    exit_ = true;
    QObject::disconnect(&frame_timer_, 0, 0, 0);
    frame_timer_.stop();
  }

  void GridPlugin::DrawIcon()
  {
    if (icon_)
    {
      QPixmap icon(16, 16);
      icon.fill(Qt::transparent);
      
      QPainter painter(&icon);
      painter.setRenderHint(QPainter::Antialiasing, true);
      
      QPen pen(QColor(ui_.color->color()));
      
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

  void GridPlugin::SetAlpha(double alpha)
  {
    alpha_ = alpha;
    if (canvas_)
      canvas_->update();
  }

  void GridPlugin::SetX(double x)
  {
    top_left_.setX(x);

    RecalculateGrid();
  }

  void GridPlugin::SetY(double y)
  {
    top_left_.setY(y);

    RecalculateGrid();
  }

  void GridPlugin::SetSize(double size)
  {
    size_ = size;

    RecalculateGrid();
  }

  void GridPlugin::SetRows(int rows)
  {
    rows_ = rows;

    RecalculateGrid();
  }

  void GridPlugin::SetColumns(int columns)
  {
    columns_ = columns;

    RecalculateGrid();
  }

  void GridPlugin::SetFrame(QString frame)
  {
    source_frame_ = frame.toStdString();

    initialized_ = true;

    RecalculateGrid();

    if (canvas_)
    canvas_->update();
  }

  void GridPlugin::UpdateFrames()
  {
    if (exit_)
      return;

    std::vector<std::string> frames;
    tf_->getFrameStrings(frames);

    if (ui_.frame->count() >= 0 && 
        static_cast<size_t>(ui_.frame->count()) == frames.size())
    {
      bool changed = false;
      for (size_t i = 0; i < frames.size(); i++)
      {
        if (frames[i] != ui_.frame->itemText(i).toStdString())
        {
          changed = true;
        }
      }

      if (!changed)
        return;
    }

    ROS_INFO("Updating frames for grid plugin ...");

    std::string current = ui_.frame->currentText().toStdString();

    ui_.frame->clear();
    for (size_t i = 0; i < frames.size(); i++)
    {
      ui_.frame->addItem(frames[i].c_str());
    }

    if (current != "")
    {
      int index = ui_.frame->findText(current.c_str());
      if (index < 0)
      {
        ui_.frame->addItem(current.c_str());
      }

      index = ui_.frame->findText(current.c_str());
      ui_.frame->setCurrentIndex(index);
    }
  }

  void GridPlugin::PrintError(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_ERROR("Error: %s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  void GridPlugin::PrintInfo(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_INFO("%s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::green);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  void GridPlugin::PrintWarning(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_WARN("%s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::darkYellow);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  QWidget* GridPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool GridPlugin::Initialize(QGLWidget* canvas)
  {
    canvas_ = canvas;

    QObject::connect(&frame_timer_, SIGNAL(timeout()), this, SLOT(UpdateFrames()));

    frame_timer_.start(1000);
    
    DrawIcon();

    return true;
  }

  void GridPlugin::Draw(double x, double y, double scale)
  {
    if (transformed_)
    {
      QColor color = ui_.color->color();
      
      glLineWidth(3);
      glColor4f(color.redF(), color.greenF(), color.blueF(), alpha_);
      glBegin(GL_LINES);

        std::list<tf::Point>::iterator transformed_left_it = transformed_left_points_.begin();
        std::list<tf::Point>::iterator transformed_right_it = transformed_right_points_.begin();
        for (; transformed_left_it != transformed_left_points_.end(); ++transformed_left_it)
        {
          glVertex2f(transformed_left_it->getX(), transformed_left_it->getY());
          glVertex2f(transformed_right_it->getX(), transformed_right_it->getY());

          ++transformed_right_it;
        }

        std::list<tf::Point>::iterator transformed_top_it = transformed_top_points_.begin();
        std::list<tf::Point>::iterator transformed_bottom_it = transformed_bottom_points_.begin();
        for (; transformed_top_it != transformed_top_points_.end(); ++transformed_top_it)
        {
          glVertex2f(transformed_top_it->getX(), transformed_top_it->getY());
          glVertex2f(transformed_bottom_it->getX(), transformed_bottom_it->getY());

          ++transformed_bottom_it;
        }

      glEnd();

      PrintInfo("OK");
    }
  }

  void GridPlugin::RecalculateGrid()
  {
    transformed_ = false;

    left_points_.clear();
    right_points_.clear();
    top_points_.clear();
    bottom_points_.clear();

    transformed_left_points_.clear();
    transformed_right_points_.clear();
    transformed_top_points_.clear();
    transformed_bottom_points_.clear();

    // Set top and bottom
    for (int c = 0; c <= columns_; c++)
    {
      tf::Point top_point(top_left_.getX() + c * size_, top_left_.getY(), 0);
      top_points_.push_back(top_point);
      transformed_top_points_.push_back(transform_ * top_point);

      tf::Point bottom_point(top_left_.getX() + c * size_, top_left_.getY() + size_ * rows_, 0);
      bottom_points_.push_back(bottom_point);
      transformed_bottom_points_.push_back(transform_ * bottom_point);
    }

    // Set left and right
    for (int r = 0; r <= rows_; r++)
    {
      tf::Point left_point(top_left_.getX(), top_left_.getY() + r * size_, 0);
      left_points_.push_back(left_point);
      transformed_left_points_.push_back(transform_ * left_point);

      tf::Point right_point(top_left_.getX() + size_ * columns_, top_left_.getY() + r * size_, 0);
      right_points_.push_back(right_point);
      transformed_right_points_.push_back(transform_ * right_point);
    }

    if (canvas_)
    canvas_->update();
  }

  void GridPlugin::Transform()
  {
    transformed_ = false;

    if (GetTransform(ros::Time(), transform_))
    {
      Transform(left_points_, transformed_left_points_);
      Transform(right_points_, transformed_right_points_);
      Transform(top_points_, transformed_top_points_);
      Transform(bottom_points_, transformed_bottom_points_);

      transformed_ = true;
    }
  }

  void GridPlugin::Transform(std::list<tf::Point>& src, std::list<tf::Point>& dst)
  {
    std::list<tf::Point>::iterator points_it = src.begin();
    std::list<tf::Point>::iterator transformed_it = dst.begin();
    for (; points_it != src.end() && transformed_it != dst.end(); ++points_it)
    {
      (*transformed_it) = transform_ * (*points_it);

      ++transformed_it;
    }
  }

  void GridPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    std::string color;
    node["color"] >> color;
    ui_.color->setColor(QColor(color.c_str()));

    std::string frame;
    node["frame"] >> frame;
    ui_.frame->setEditText(frame.c_str());

    float x = 0;
    float y = 0;
    node["x"] >> x;
    node["y"] >> y;

    ui_.x->setValue(x);
    ui_.y->setValue(y);

    node["alpha"] >> alpha_;
    ui_.alpha->setValue(alpha_);

    node["size"] >> size_;
    ui_.size->setValue(size_);

    node["rows"] >> rows_;
    ui_.rows->setValue(rows_);

    node["columns"] >> columns_;
    ui_.columns->setValue(columns_);

    if (canvas_)
      canvas_->update();
  }

  void GridPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    emitter << YAML::Key << "color" << YAML::Value << ui_.color->color().name().toStdString();

    emitter << YAML::Key << "alpha" << YAML::Value << alpha_;

    std::string frame = ui_.frame->currentText().toStdString();
    emitter << YAML::Key << "frame" << YAML::Value << frame;

    emitter << YAML::Key << "x" << YAML::Value << top_left_.getX();
    emitter << YAML::Key << "y" << YAML::Value << top_left_.getY();
    emitter << YAML::Key << "size" << YAML::Value << size_;
    emitter << YAML::Key << "rows" << YAML::Value << rows_;
    emitter << YAML::Key << "columns" << YAML::Value << columns_;
  }
}

