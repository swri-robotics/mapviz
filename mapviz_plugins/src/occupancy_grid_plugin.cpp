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
#include <GL/glut.h>

// C++ standard libraries
#include <cstdio>
#include <vector>

// QT libraries
#include <QGLWidget>
#include <QPalette>

#include <mapviz/select_topic_dialog.h>

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mapviz_plugins::OccupancyGridPlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{
  OccupancyGridPlugin::OccupancyGridPlugin() :
    config_widget_(new QWidget()),
    transformed_(false),
    points_vbo_(0),
    colours_vbo_(0),
    vao_(0)
  {
    ui_.setupUi(config_widget_);

    // Set background white
    QPalette p(config_widget_->palette());
    p.setColor(QPalette::Background, Qt::white);
    config_widget_->setPalette(p);

    // Set status text red
    QPalette p3(ui_.status->palette());
    p3.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p3);

    QObject::connect(ui_.select_topic, SIGNAL(clicked()), this, SLOT(SelectTopic()));
    QObject::connect(ui_.topic, SIGNAL(textEdited(const QString&)), this, SLOT(TopicEdited()));
    QObject::connect(ui_.alpha, SIGNAL(valueChanged(double)), this, SLOT(AlphaEdited(double)));
    QObject::connect(this, SIGNAL(TargetFrameChanged(std::string)), this, SLOT(FrameChanged(std::string)));
  }

  OccupancyGridPlugin::~OccupancyGridPlugin()
  {
    Shutdown();
  }

  void OccupancyGridPlugin::Shutdown()
  {
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

  void OccupancyGridPlugin::SelectTopic()
  {
    ros::master::TopicInfo topic = mapviz::SelectTopicDialog::selectTopic("nav_msgs/OccupancyGrid");
    if (!topic.name.empty())
    {
      ui_.topic->setText(QString::fromStdString(topic.name));
      TopicEdited();
    }
  }

  void OccupancyGridPlugin::TopicEdited()
  {
    const std::string topic = ui_.topic->text().trimmed().toStdString();
    if (topic != topic_)
    {
      initialized_ = false;
      grid_.reset();
      PrintWarning("No messages received.");

      grid_sub_.shutdown();

      topic_ = topic;
      if (!topic.empty())
      {
        grid_sub_ = node_.subscribe(topic_, 1, &OccupancyGridPlugin::Callback, this);
        ROS_INFO("Subscribing to %s", topic_.c_str());
      }
    }
  }

  void OccupancyGridPlugin::AlphaEdited(double alpha)
  {
    if( grid_ )
    {
      for (int i=3; i< gl_colors_.size(); i += 4)
      {
        gl_colors_[i] = alpha;
      }

      glBindVertexArray(vao_);

      glGenBuffers(1, &colours_vbo_);
      glBindBuffer(GL_ARRAY_BUFFER, colours_vbo_);  // color
      glBufferData(GL_ARRAY_BUFFER, gl_colors_.size() * sizeof(GLfloat), gl_colors_.data(), GL_STATIC_DRAW);
      glColorPointer( 4, GL_FLOAT, 0, 0);

      glBindVertexArray(0);
      glBindBuffer(GL_ARRAY_BUFFER, 0);
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

  void OccupancyGridPlugin::Callback(const nav_msgs::OccupancyGridConstPtr& msg)
  {
    initialized_ = true;
    grid_ = msg;
    size_t num_quads = 0;
    for( size_t i=0; i<msg->data.size(); i++ )
    {
      if( msg->data[i] >= 0 ) num_quads++;
    }
    gl_points_.clear();
    gl_points_.reserve(  8*num_quads );
    gl_colors_.clear();
    gl_colors_.reserve( 16*num_quads );
    double alpha = ui_.alpha->value();

    for (size_t col = 0; col < msg->info.width; col++)
    {
      for (size_t row = 0; row < msg->info.height; row++)
      {
        const double color = msg->data[ row + (col*msg->info.height) ];

        if( color < 0) continue;

        gl_points_.push_back( row ); // X
        gl_points_.push_back( col ); // Y

        gl_points_.push_back( row ); // X
        gl_points_.push_back( col + 1 ); // Y

        gl_points_.push_back( row + 1 ); // X
        gl_points_.push_back( col + 1 ); // Y

        gl_points_.push_back( row + 1 ); // X
        gl_points_.push_back( col ); // Y

        for (int j=0; j<4; j++)
        {
          for (int c=0; c<3; c++)
          {
            gl_colors_.push_back( 1.0 - color * 0.01 );
          }
          gl_colors_.push_back(alpha);
        }
      }
    }

    source_frame_ = msg->header.frame_id;
    transformed_ = GetTransform( source_frame_, msg->header.stamp, transform_);
    if ( !transformed_ )
    {
      PrintError("No transform between " + source_frame_ + " and " + target_frame_);
    }

    glGenVertexArrays(1, &vao_); // Create new VAO
    glBindVertexArray(vao_);

    glGenBuffers(1, &points_vbo_);
    glBindBuffer(GL_ARRAY_BUFFER, points_vbo_);  // coordinates
    glBufferData(GL_ARRAY_BUFFER, gl_points_.size() * sizeof(GLfloat), gl_points_.data(), GL_STATIC_DRAW);
    glVertexPointer( 2, GL_FLOAT, 0, 0);

    glGenBuffers(1, &colours_vbo_);
    glBindBuffer(GL_ARRAY_BUFFER, colours_vbo_);  // color
    glBufferData(GL_ARRAY_BUFFER, gl_colors_.size() * sizeof(GLfloat), gl_colors_.data(), GL_STATIC_DRAW);
    glColorPointer( 4, GL_FLOAT, 0, 0);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
  }

  void OccupancyGridPlugin::Draw(double x, double y, double scale)
  {
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);

    glPushMatrix();

    if( grid_ && transformed_)
    {
      glTranslatef( transform_.GetOrigin().getX(),  transform_.GetOrigin().getY(), 0.0);

      tfScalar yaw, pitch, roll;
      tf::Matrix3x3 mat( transform_.GetOrientation() );
      mat.getEulerYPR(yaw, pitch, roll);

      glRotatef(pitch * 180.0 / M_PI, 0, 1, 0);
      glRotatef(roll  * 180.0 / M_PI, 1, 0, 0);
      glRotatef(yaw   * 180.0 / M_PI, 0, 0, 1);

      glScalef( grid_->info.resolution, grid_->info.resolution, 1.0);

      glBegin(GL_QUADS);
      glColor4f(0.6, 0.7, 0.7f, ui_.alpha->value()*0.5);

      glVertex2d(0, 0);
      glVertex2d(grid_->info.width, 0.0);
      glVertex2d(grid_->info.width, grid_->info.height);
      glVertex2d(0.0, grid_->info.height);

      glEnd();

      glBindVertexArray(vao_);
      glDrawArrays(GL_QUADS, 0, gl_points_.size() / 2 );

      PrintInfo("OK");
    }

    glPopMatrix();

    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);

  }

  void OccupancyGridPlugin::Transform()
  {
    if ( !transformed_ && grid_ )
    {
      transformed_ = GetTransform( source_frame_, grid_->header.stamp, transform_);
    }
    if ( !transformed_ )
    {
      PrintError("No transform between " + source_frame_ + " and " + target_frame_);
    }
  }

  void OccupancyGridPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    if (node["topic"])
    {
      std::string topic;
      node["topic"] >> topic;
      ui_.topic->setText(QString::fromStdString(topic));
    }

    if (node["alpha"])
    {
      double alpha;
      node["alpha"] >> alpha;
      ui_.alpha->setValue(alpha);
    }

    TopicEdited();
  }

  void OccupancyGridPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    emitter << YAML::Key << "alpha" << YAML::Value << ui_.alpha->value();

    std::string topic = ui_.topic->text().toStdString();
    emitter << YAML::Key << "topic" << YAML::Value << topic;
  }
}

