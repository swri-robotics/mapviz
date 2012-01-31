// C++ standard libraries
#include <cstdio>
#include <vector>

// QT libraries
#include <QDialog>
#include <QGLWidget>
#include <QPalette>
#include <QImage>
#include <QFileDialog>

// ROS libraries
#include <ros/master.h>

#include "robot_image_plugin.h"

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(mapviz_plugins, robot_image, mapviz_plugins::RobotImagePlugin, mapviz::MapvizPlugin);

namespace mapviz_plugins
{

  RobotImagePlugin::RobotImagePlugin() :
    config_widget_(new QWidget()),
    width_(1),
    height_(1),
    texture_loaded_(false)
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
    QDialog dialog;
    Ui::topicselect ui;
    ui.setupUi(&dialog);

    std::vector<std::string> frames;
    transform_listener_.getFrameStrings(frames);

    for (unsigned int i = 0; i < frames.size(); i++)
    {
      ui.displaylist->addItem(frames[i].c_str());
    }
    ui.displaylist->setCurrentRow(0);

    dialog.exec();

    if (dialog.result() == QDialog::Accepted && ui.displaylist->selectedItems().count() == 1)
    {
      ui_.frame->setText(ui.displaylist->selectedItems().first()->text());
      FrameEdited();
    }
  }

  void RobotImagePlugin::FrameEdited()
  {
    source_frame_ = ui_.frame->text().toStdString();
    PrintWarning("Waiting for transform.");

    ROS_INFO("Setting target frame to to %s", source_frame_.c_str());

    initialized_ = true;

    canvas_->update();
  }

  void RobotImagePlugin::WidthChanged(double value)
  {
    width_ = value;

    UpdateShape();

    canvas_->update();
  }

  void RobotImagePlugin::HeightChanged(double value)
  {
    height_ = value;

    UpdateShape();

    canvas_->update();
  }

  void RobotImagePlugin::UpdateShape()
  {
    top_left_.setX(-width_/2.0);
    top_left_.setY(height_ / 2.0);
    top_right_.setX(width_/2.0);
    top_right_.setY(height_ / 2.0);
    bottom_left_.setX(-width_/2.0);
    bottom_left_.setY(-height_/2.0);
    bottom_right_.setX(width_/2.0);
    bottom_right_.setY(-height_/2.0);

    Transform();
  }

  void RobotImagePlugin::PrintError(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_ERROR("Error: %s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  void RobotImagePlugin::PrintInfo(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_INFO("%s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::green);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  void RobotImagePlugin::PrintWarning(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_WARN("%s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::darkYellow);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
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
    Transform();

    glColor3f(1.0f, 1.0f, 1.0f);

    glPointSize(4);

    if (texture_loaded_)
    {
      glEnable(GL_TEXTURE_2D);
      glBindTexture(GL_TEXTURE_2D, texture_id_);

      glBegin(GL_QUADS);


      glTexCoord2f(0, 1); glVertex2f(top_left_transformed_.x(), top_left_transformed_.y());
      glTexCoord2f(1, 1); glVertex2f(top_right_transformed_.x(), top_right_transformed_.y());
      glTexCoord2f(1, 0); glVertex2f(bottom_right_transformed_.x(), bottom_right_transformed_.y());
      glTexCoord2f(0, 0); glVertex2f(bottom_left_transformed_.x(), bottom_left_transformed_.y());

      glEnd();

      glDisable(GL_TEXTURE_2D);
    }
  }

  void RobotImagePlugin::Transform()
  {
    top_left_transformed_ = transform_ * top_left_;
    top_right_transformed_ = transform_ * top_right_;
    bottom_left_transformed_ = transform_ * bottom_left_;
    bottom_right_transformed_ = transform_ * bottom_right_;
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
        ids[0] = texture_id_;
        glDeleteTextures(1, &ids[0]);
        texture_loaded_ = false;
      }

      if (image_.load(filename_.c_str()))
      {
        int width = image_.width();
        int height = image_.height();

        int maxDimension = std::max(width, height);
        dimension_ = static_cast<int>(pow(2, ceil(log((float)maxDimension)/log(2.0f))));

        if (width != dimension_ || height != dimension_)
        {
          image_ = image_.scaled(dimension_, dimension_, Qt::IgnoreAspectRatio, Qt::FastTransformation);
        }

        image_ = QGLWidget::convertToGLFormat(image_);

        GLuint ids[1];
        glGenTextures(1, &ids[0]);
        texture_id_ = ids[0];

        glBindTexture(GL_TEXTURE_2D, texture_id_);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, dimension_, dimension_, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_.bits());

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        texture_loaded_ = true;
      }
      else
      {
        PrintError("Failed to load image.");
      }
    }
    catch(std::exception e)
    {
      PrintError("Failed to load image.  Exception occured.");
    }
  }

  void RobotImagePlugin::LoadConfiguration(const YAML::Node& node, const std::string& config_path)
  {
    node["frame"] >> source_frame_;
    ui_.frame->setText(source_frame_.c_str());

    node["image"] >> filename_;
    ui_.image->setText(filename_.c_str());

    node["width"] >> width_;
    ui_.width->setValue(width_);

    node["height"] >> height_;
    ui_.height->setValue(height_);

    UpdateShape();
    LoadImage();
    FrameEdited();
  }

  void RobotImagePlugin::SaveConfiguration(YAML::Emitter& emitter, const std::string& config_path)
  {
    emitter << YAML::Key << "frame" << YAML::Value << ui_.frame->text().toStdString();
    emitter << YAML::Key << "image" << YAML::Value << ui_.image->text().toStdString();
    emitter << YAML::Key << "width" << YAML::Value << width_;
    emitter << YAML::Key << "height" << YAML::Value << height_;
  }

}

