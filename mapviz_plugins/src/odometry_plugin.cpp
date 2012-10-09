// C++ standard libraries
#include <cstdio>
#include <vector>

// QT libraries
#include <QDialog>
#include <QColorDialog>
#include <QGLWidget>
#include <QPalette>

// ROS libraries
#include <ros/master.h>

#include "odometry_plugin.h"

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(mapviz_plugins, odometry, mapviz_plugins::OdometryPlugin, mapviz::MapvizPlugin);

namespace mapviz_plugins
{

  OdometryPlugin::OdometryPlugin() :
    config_widget_(new QWidget()),
    color_(Qt::green),
    draw_style_(LINES)
  {
    ui_.setupUi(config_widget_);

    // Set background white
    QPalette p(config_widget_->palette());
    p.setColor(QPalette::Background, Qt::white);
    config_widget_->setPalette(p);

    // Initialize color selector color
    ui_.selectcolor->setStyleSheet("background: " + color_.name() + ";");

    // Set status text red
    QPalette p3(ui_.status->palette());
    p3.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p3);

    QObject::connect(ui_.selectcolor, SIGNAL(clicked()), this, SLOT(SelectColor()));
    QObject::connect(ui_.selecttopic, SIGNAL(clicked()), this, SLOT(SelectTopic()));
    QObject::connect(ui_.topic, SIGNAL(editingFinished()), this, SLOT(TopicEdited()));
    QObject::connect(ui_.positiontolerance, SIGNAL(valueChanged(double)), this, SLOT(PositionToleranceChanged(double)));
    QObject::connect(ui_.buffersize, SIGNAL(valueChanged(int)), this, SLOT(BufferSizeChanged(int)));
    QObject::connect(ui_.drawstyle, SIGNAL(activated(QString)), this, SLOT(SetDrawStyle(QString)));
  }

  OdometryPlugin::~OdometryPlugin()
  {

  }

  void OdometryPlugin::SetDrawStyle(QString style)
  {
    if (style == "lines")
    {
      draw_style_ = LINES;
    }
    else if (style == "points")
    {
      draw_style_ = POINTS;
    }

    canvas_->update();
  }

  void OdometryPlugin::SelectTopic()
  {
    QDialog dialog;
    Ui::topicselect ui;
    ui.setupUi(&dialog);

    std::vector<ros::master::TopicInfo> topics;
    ros::master::getTopics(topics);

    for (unsigned int i = 0; i < topics.size(); i++)
    {
      if (topics[i].datatype == "nav_msgs/Odometry")
      {
        ui.displaylist->addItem(topics[i].name.c_str());
      }
    }
    ui.displaylist->setCurrentRow(0);
    
    dialog.exec();

    if (dialog.result() == QDialog::Accepted && ui.displaylist->selectedItems().count() == 1)
    {
      ui_.topic->setText(ui.displaylist->selectedItems().first()->text());
      TopicEdited();
    }
  }
  
  void OdometryPlugin::SelectColor()
  {
    QColorDialog dialog(color_, config_widget_);
    dialog.exec();

    if (dialog.result() == QDialog::Accepted)
    {
      color_ = dialog.selectedColor();
      ui_.selectcolor->setStyleSheet("background: " + color_.name() + ";");
      canvas_->update();
    }
  }
  
  void OdometryPlugin::TopicEdited()
  {
    if (ui_.topic->text().toStdString() != topic_)
    {
      initialized_ = false;
      points_.clear();
      transformed_points_.clear();
      has_message_ = false;
      topic_ = ui_.topic->text().toStdString();
      PrintWarning("No messages received.");

      odometry_sub_.shutdown();
      odometry_sub_ = node_.subscribe(topic_, 1, &OdometryPlugin::odometryCallback, this);

      ROS_INFO("Subscribing to %s", topic_.c_str());
    }
  }
  
  void OdometryPlugin::odometryCallback(const nav_msgs::OdometryConstPtr odometry)
  {
    if (!has_message_)
    {
      source_frame_ = odometry->header.frame_id;
      initialized_ = true;
      has_message_ = true;
    }

    tf::Point point(
      odometry->pose.pose.position.x, 
      odometry->pose.pose.position.y,
      odometry->pose.pose.position.z);
      
    current_point_ = point;
    current_point_transformed_ = transform_ * point;

    if (points_.empty() || point.distance(points_.back()) >= position_tolerance_)
    {
      points_.push_back(point);
      transformed_points_.push_back(transform_ * point);
    }

    if (buffer_size_ > 0)
    {
      while (static_cast<int>(points_.size()) > buffer_size_)
      {
        points_.pop_front();
        transformed_points_.pop_front();
      }
    }

    canvas_->update();
  }

  void OdometryPlugin::PositionToleranceChanged(double value)
  {
    position_tolerance_ = value;
  }

  void OdometryPlugin::BufferSizeChanged(int value)
  {
    buffer_size_ = value;

    if (buffer_size_ > 0)
    {
      while (static_cast<int>(points_.size()) > buffer_size_)
      {
        points_.pop_front();
        transformed_points_.pop_front();
      }
    }

    canvas_->update();
  }

  void OdometryPlugin::PrintError(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_ERROR("Error: %s", message.c_str()); 
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  void OdometryPlugin::PrintInfo(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_INFO("%s", message.c_str()); 
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::green);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  void OdometryPlugin::PrintWarning(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_WARN("%s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::darkYellow);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  QWidget* OdometryPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool OdometryPlugin::Initialize(QGLWidget* canvas)
  {
    canvas_ = canvas;
  
    return true;
  }
    
  void OdometryPlugin::Draw(double x, double y, double scale)
  {
    
    glColor3f(color_.redF(), color_.greenF(), color_.blueF());

    if (draw_style_ == LINES)
    {
      glLineWidth(3);
      glBegin(GL_LINE_STRIP);
    }
    else
    {
      glPointSize(6);
      glBegin(GL_POINTS);
    }

      std::list<tf::Point>::iterator transformed_it = transformed_points_.begin();
      for (; transformed_it != transformed_points_.end(); ++transformed_it)
      {
        glVertex2f(transformed_it->getX(), transformed_it->getY());
      }

      glVertex2f(current_point_transformed_.getX(), current_point_transformed_.getY());

    glEnd();
  }

  void OdometryPlugin::Transform()
  {
    std::list<tf::Point>::iterator points_it = points_.begin();
    std::list<tf::Point>::iterator transformed_it = transformed_points_.begin();
    for (; points_it != points_.end() && transformed_it != transformed_points_.end(); ++points_it)
    {
      (*transformed_it) = transform_ * (*points_it);

      ++transformed_it;
    }

    current_point_transformed_ = transform_ * current_point_;
  }
    
  void OdometryPlugin::LoadConfiguration(const YAML::Node& node, const std::string& config_path)
  {
    std::string topic;
    node["topic"] >> topic;
    ui_.topic->setText(topic.c_str());

    std::string color;
    node["color"] >> color;
    color_ = QColor(color.c_str());
    ui_.selectcolor->setStyleSheet("background: " + color_.name() + ";");

    std::string draw_style;
    node["draw_style"] >> draw_style;
    
    if (draw_style == "lines")
    {
      draw_style_ = LINES;
      ui_.drawstyle->setCurrentIndex(0);
    }
    else if (draw_style == "points")
    {
      draw_style_ = POINTS;
      ui_.drawstyle->setCurrentIndex(1);
    }

    node["position_tolerance"] >> position_tolerance_;
    ui_.positiontolerance->setValue(position_tolerance_);

    node["buffer_size"] >> buffer_size_;
    ui_.buffersize->setValue(buffer_size_);

    TopicEdited();
  }
  
  void OdometryPlugin::SaveConfiguration(YAML::Emitter& emitter, const std::string& config_path)
  {
    emitter << YAML::Key << "topic" << YAML::Value << ui_.topic->text().toStdString();
    emitter << YAML::Key << "color" << YAML::Value << color_.name().toStdString();
    emitter << YAML::Key << "draw_style" << YAML::Value << ui_.drawstyle->currentText().toStdString();
    emitter << YAML::Key << "position_tolerance" << YAML::Value << position_tolerance_;
    emitter << YAML::Key << "buffer_size" << YAML::Value << buffer_size_;
  }
  
}

