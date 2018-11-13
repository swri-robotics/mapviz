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

#include <mapviz_plugins/robot_image_plugin.h>

// C++ standard libraries
#include <cstdio>
#include <algorithm>
#include <vector>

// QT libraries
#include <QGLWidget>
#include <QPalette>
#include <QImage>
#include <QFileDialog>

// ROS libraries
#include <ros/master.h>

#include <mapviz/select_frame_dialog.h>

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mapviz_plugins::RobotImagePlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{
  RobotImagePlugin::RobotImagePlugin() :
    config_widget_(new QWidget()),
    width_(2.0),
    height_(1.0),
    offset_x_(0.0),
    offset_y_(0.0),
    image_ratio_(1.0),
    texture_loaded_(false),
    transformed_(false)
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

    UpdateShape();

    QObject::connect(ui_.browse, SIGNAL(clicked()), this, SLOT(SelectFile()));
    QObject::connect(ui_.selectframe, SIGNAL(clicked()), this, SLOT(SelectFrame()));
    QObject::connect(ui_.frame, SIGNAL(editingFinished()), this, SLOT(FrameEdited()));
    QObject::connect(ui_.width, SIGNAL(valueChanged(double)), this, SLOT(WidthChanged(double)));
    QObject::connect(ui_.height, SIGNAL(valueChanged(double)), this, SLOT(HeightChanged(double)));
    QObject::connect(ui_.offset_x, SIGNAL(valueChanged(double)), this, SLOT(OffsetXChanged(double)));
    QObject::connect(ui_.offset_y, SIGNAL(valueChanged(double)), this, SLOT(OffsetYChanged(double)));
    ui_.offset_x->setMinimum(-99.99); //default is 0.0 but negative offset must be supported
    ui_.offset_y->setMinimum(-99.99);
    QObject::connect(ui_.ratio_equal, SIGNAL(toggled(bool)), this, SLOT(RatioEqualToggled(bool)));
    QObject::connect(ui_.ratio_custom, SIGNAL(toggled(bool)), this, SLOT(RatioCustomToggled(bool)));
    QObject::connect(ui_.ratio_original, SIGNAL(toggled(bool)), this, SLOT(RatioOriginalToggled(bool)));
  }

  RobotImagePlugin::~RobotImagePlugin()
  {
  }

  void RobotImagePlugin::SelectFile()
  {
    QFileDialog dialog(config_widget_, "Select PNG Image");
    dialog.setFileMode(QFileDialog::ExistingFile);
    dialog.setNameFilter(tr("PNG Image Files (*.png)"));

    dialog.exec();

    if (dialog.result() == QDialog::Accepted && dialog.selectedFiles().count() == 1)
    {
      ui_.image->setText(dialog.selectedFiles().first());
      filename_ = dialog.selectedFiles().first().toStdString();
      LoadImage();
    }
  }

  void RobotImagePlugin::SelectFrame()
  {
    std::string frame = mapviz::SelectFrameDialog::selectFrame(tf_);
    if (!frame.empty())
    {
      ui_.frame->setText(QString::fromStdString(frame));
      FrameEdited();
    }
  }

  void RobotImagePlugin::FrameEdited()
  {
    source_frame_ = ui_.frame->text().toStdString();
    PrintWarning("Waiting for transform.");

    ROS_INFO("Setting target frame to to %s", source_frame_.c_str());

    initialized_ = true;

    UpdateShape();
  }

  void RobotImagePlugin::WidthChanged(double value)
  {
    width_ = value;
    if( ui_.ratio_equal->isChecked()){
      ui_.height->setValue( width_ );
    }
    else if( ui_.ratio_original->isChecked()){
      ui_.height->setValue( width_ * image_ratio_ );
    }
    UpdateShape();
  }

  void RobotImagePlugin::HeightChanged(double value)
  {
    height_ = value;
    UpdateShape();
  }

  void RobotImagePlugin::OffsetXChanged(double value)
  {
    offset_x_ = value;
    UpdateShape();
  }

  void RobotImagePlugin::OffsetYChanged(double value)
  {
    offset_y_ = value;
    UpdateShape();
  }

  void RobotImagePlugin::RatioEqualToggled(bool toggled)
  {
    if( toggled )
    {
      ui_.height->setValue(width_);
      ui_.height->setEnabled(false);
      UpdateShape();
    }
  }

  void RobotImagePlugin::RatioCustomToggled(bool toggled)
  {
    if( toggled )
    {
      ui_.height->setEnabled(true);
      UpdateShape();
    }
  }

  void RobotImagePlugin::RatioOriginalToggled(bool toggled)
  {
    if( toggled )
    {
      ui_.height->setValue(width_*image_ratio_);
      ui_.height->setEnabled(false);
      UpdateShape();
    }
  }

  void RobotImagePlugin::UpdateShape()
  {
    double hw = 0.5*width_; //half width
    double hh = 0.5*height_; //half height
    top_left_ = tf::Point(offset_x_ - hw, offset_y_ + hh, 0);
    top_right_ = tf::Point(offset_x_ + hw, offset_y_ + hh, 0);
    bottom_left_ = tf::Point(offset_x_ - hw, offset_y_ - hh, 0);
    bottom_right_ = tf::Point(offset_x_ + hw, offset_y_ - hh, 0);
  }

  void RobotImagePlugin::PrintError(const std::string& message)
  {
    PrintErrorHelper(ui_.status, message);
  }

  void RobotImagePlugin::PrintInfo(const std::string& message)
  {
    PrintInfoHelper(ui_.status, message);
  }

  void RobotImagePlugin::PrintWarning(const std::string& message)
  {
    PrintWarningHelper(ui_.status, message);
  }

  QWidget* RobotImagePlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool RobotImagePlugin::Initialize(QGLWidget* canvas)
  {
    canvas_ = canvas;

    return true;
  }

  void RobotImagePlugin::Draw(double x, double y, double scale)
  {
    if (texture_loaded_ && transformed_)
    {
      glColor3f(1.0f, 1.0f, 1.0f);
      glEnable(GL_TEXTURE_2D);
      glBindTexture(GL_TEXTURE_2D, static_cast<GLuint>(texture_id_));

      glBegin(GL_QUADS);

      glTexCoord2f(0, 1); glVertex2d(top_left_transformed_.x(), top_left_transformed_.y());
      glTexCoord2f(1, 1); glVertex2d(top_right_transformed_.x(), top_right_transformed_.y());
      glTexCoord2f(1, 0); glVertex2d(bottom_right_transformed_.x(), bottom_right_transformed_.y());
      glTexCoord2f(0, 0); glVertex2d(bottom_left_transformed_.x(), bottom_left_transformed_.y());

      glEnd();

      glDisable(GL_TEXTURE_2D);

      PrintInfo("OK");
    }
  }

  void RobotImagePlugin::Transform()
  {
    transformed_ = false;

    swri_transform_util::Transform transform;
    if (GetTransform(ros::Time(), transform))
    {
      top_left_transformed_ = transform * top_left_;
      top_right_transformed_ = transform * top_right_;
      bottom_left_transformed_ = transform * bottom_left_;
      bottom_right_transformed_ = transform * bottom_right_;
      transformed_ = true;
    }
    else
    {
      PrintError("No transform between " + source_frame_ + " and " + target_frame_);
    }
  }

  void RobotImagePlugin::LoadImage()
  {
    ROS_INFO("Loading image");
    try
    {
      QImage nullImage;
      image_ = nullImage;

      if (texture_loaded_)
      {
        GLuint ids[1];
        ids[0] = static_cast<GLuint>(texture_id_);
        glDeleteTextures(1, &ids[0]);
        texture_loaded_ = false;
      }

      if (image_.load(filename_.c_str()))
      {
        int width = image_.width();
        int height = image_.height();
        image_ratio_ = (double)height / (double)width;

        float max_dim = std::max(width, height);
        dimension_ = static_cast<int>(std::pow(2, std::ceil(std::log(max_dim) / std::log(2.0f))));

        if (width != dimension_ || height != dimension_)
        {
          image_ = image_.scaled(dimension_, dimension_, Qt::IgnoreAspectRatio, Qt::FastTransformation);
        }

        image_ = QGLWidget::convertToGLFormat(image_);

        GLuint ids[1];
        glGenTextures(1, &ids[0]);
        texture_id_ = ids[0];

        glBindTexture(GL_TEXTURE_2D, static_cast<GLuint>(texture_id_));
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, dimension_, dimension_, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_.bits());

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        texture_loaded_ = true;
        if( ui_.ratio_original->isChecked() )
        {
          RatioOriginalToggled(true);
        }
      }
      else
      {
        PrintError("Failed to load image.");
      }
    }
    catch (const std::exception& e)
    {
      PrintError("Failed to load image.  Exception occured.");
    }
  }

  void RobotImagePlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    if (node["frame"])
    {
      node["frame"] >> source_frame_;
      ui_.frame->setText(source_frame_.c_str());
    }
    if (node["offset_x"])
    {
      node["offset_x"] >> offset_x_;
      ui_.offset_x->setValue(offset_x_);
    }

    if (node["offset_y"])
    {
      node["offset_y"] >> offset_y_;
      ui_.offset_y->setValue(offset_y_);
    }

    if (node["image"])
    {
      node["image"] >> filename_;
      ui_.image->setText(filename_.c_str());
    }

    if (node["width"])
    {
      node["width"] >> width_;
      ui_.width->setValue(width_);
    }

    if (node["height"])
    {
      node["height"] >> height_;
      ui_.height->setValue(height_);
    }

    if (node["ratio"])
    {
      std::string value;
      node["ratio"] >> value;
      if(value == "equal")
      {
        ui_.ratio_equal->setChecked(true);
      }
      else if(value == "custom")
      {
        ui_.ratio_custom->setChecked(true);
      }
      else if(value == "original")
      {
        ui_.ratio_original->setChecked(true);
      }
    }

    UpdateShape();
    LoadImage();
    FrameEdited();
  }

  void RobotImagePlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    emitter << YAML::Key << "frame" << YAML::Value << ui_.frame->text().toStdString();
    emitter << YAML::Key << "image" << YAML::Value << ui_.image->text().toStdString();
    emitter << YAML::Key << "width" << YAML::Value << width_;
    emitter << YAML::Key << "height" << YAML::Value << height_;
    emitter << YAML::Key << "offset_x" << YAML::Value << offset_x_;
    emitter << YAML::Key << "offset_y" << YAML::Value << offset_y_;
    if( ui_.ratio_custom->isChecked())
    {
      emitter << YAML::Key << "ratio" << YAML::Value << "custom";
    }
    else if( ui_.ratio_equal->isChecked())
    {
      emitter << YAML::Key << "ratio" << YAML::Value << "equal";
    }
    else if( ui_.ratio_original->isChecked())
    {
      emitter << YAML::Key << "ratio" << YAML::Value << "original";
    }
  }
}

