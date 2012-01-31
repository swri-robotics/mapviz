// C++ standard libraries
#include <cstdio>
#include <vector>
#include <cmath>

// QT libraries
#include <QDialog>
#include <QGLWidget>

// ROS libraries
#include <ros/master.h>

#include "marker_plugin.h"


// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(mapviz_plugins, marker, mapviz_plugins::MarkerPlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{

  MarkerPlugin::MarkerPlugin() :
    config_widget_(new QWidget()),
    is_marker_array_(false)
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

    QObject::connect(ui_.selecttopic, SIGNAL(clicked()), this, SLOT(SelectTopic()));
    QObject::connect(ui_.topic, SIGNAL(editingFinished()), this, SLOT(TopicEdited()));
  }

  MarkerPlugin::~MarkerPlugin()
  {

  }

  void MarkerPlugin::SelectTopic()
  {
    QDialog dialog;
    Ui::topicselect ui;
    ui.setupUi(&dialog);

    std::vector<ros::master::TopicInfo> topics;
    ros::master::getTopics(topics);

    for (unsigned int i = 0; i < topics.size(); i++)
    {
      if (topics[i].datatype == "visualization_msgs/Marker" || topics[i].datatype == "visualization_msgs/MarkerArray")
      {
        ui.displaylist->addItem(topics[i].name.c_str());
      }
    }
    ui.displaylist->setCurrentRow(0);

    dialog.exec();

    if (dialog.result() == QDialog::Accepted && ui.displaylist->selectedItems().count() == 1)
    {
      ui_.topic->setText(ui.displaylist->selectedItems().first()->text());

      // Determine if this is a marker array
      is_marker_array_ = false;
      for (unsigned int i = 0; i < topics.size(); i++)
      {
        if (topics[i].datatype == "visualization_msgs/MarkerArray")
        {
          if (ui.displaylist->selectedItems().first()->text().toStdString() == topics[i].name)
          {
            is_marker_array_ = true;
          }
        }
      }

      TopicEdited();
    }
  }

  void MarkerPlugin::TopicEdited()
  {
    if (ui_.topic->text().toStdString() != topic_)
    {
      initialized_ = false;
      markers_.clear();
      has_message_ = false;
      topic_ = ui_.topic->text().toStdString();
      PrintWarning("No messages received.");

      marker_sub_.shutdown();

      if (is_marker_array_)
      {
        marker_sub_ = node_.subscribe(topic_, 8, &MarkerPlugin::markerArrayCallback, this);
      }
      else
      {
        marker_sub_ = node_.subscribe(topic_, 8, &MarkerPlugin::markerCallback, this);
      }

      ROS_INFO("Subscribing to %s", topic_.c_str());
    }
  }

  void MarkerPlugin::markerCallback(const visualization_msgs::MarkerConstPtr marker)
  {
    if (!has_message_)
    {
      source_frame_ = marker->header.frame_id;
      initialized_ = true;
      has_message_ = true;
    }

    if (marker->action == visualization_msgs::Marker::ADD)
    {
      //ROS_WARN("Add/modify marker message, id = %d", marker->id);

      MarkerData& markerData = markers_[marker->id];
      markerData.display_type_ = marker->type;
      markerData.color_ = QColor::fromRgbF(marker->color.r, marker->color.g, marker->color.b, marker->color.a);
      markerData.scale_x_ = marker->scale.x;
      markerData.scale_y_ = marker->scale.y;
      markerData.scale_z_ = marker->scale.z;

      markerData.points_.clear();
      markerData.transformed_points_.clear();
      markerData.colors_.clear();

      // TODO handle lifetime parameter

      // TODO correctly transform points based on the pose
      double x = marker->pose.position.x;
      double y = marker->pose.position.y;

      if (markerData.display_type_ == visualization_msgs::Marker::CYLINDER)
      {
        tf::Point point(x, y, 0);
        markerData.points_.push_back(point);
        markerData.transformed_points_.push_back(transform_ * point);
      }
      else
      {
        for (unsigned int i = 0; i < marker->points.size(); i++)
        {
          tf::Point point(marker->points[i].x + x, marker->points[i].y + y, 0);
          markerData.points_.push_back(point);
          markerData.transformed_points_.push_back(transform_ * point);
        }


        for (unsigned int i = 0; i < marker->colors.size(); i++)
        {
          markerData.colors_.push_back(QColor::fromRgbF(marker->colors[i].r, marker->colors[i].g, marker->colors[i].b, marker->colors[i].a));
        }
      }
    }
    else
    {
      //ROS_WARN("delete marker message, id = %d", marker->id);
      markers_.erase(marker->id);
    }

    canvas_->update();
  }

  void MarkerPlugin::markerArrayCallback(const visualization_msgs::MarkerArrayConstPtr markers)
  {
    for (int i = 0; i < markers->markers.size(); i++)
    {
      markerCallback(visualization_msgs::MarkerConstPtr(new visualization_msgs::Marker(markers->markers[i])));
    }
  }

  void MarkerPlugin::PrintError(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_ERROR("Error: %s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  void MarkerPlugin::PrintInfo(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_INFO("%s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::green);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  void MarkerPlugin::PrintWarning(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_WARN("%s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::darkYellow);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  QWidget* MarkerPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool MarkerPlugin::Initialize(QGLWidget* canvas)
  {
    canvas_ = canvas;

    return true;
  }

  void MarkerPlugin::Draw(double x, double y, double scale)
  {
    std::map<int, MarkerData>::iterator markerIter;
    for (markerIter = markers_.begin(); markerIter != markers_.end(); ++markerIter)
    {
      MarkerData& marker = markerIter->second;

      glColor4f(marker.color_.redF(), marker.color_.greenF(), marker.color_.blueF(), 1.0f);

      if (marker.display_type_ == visualization_msgs::Marker::LINE_STRIP)
      {
        glLineWidth(marker.scale_x_);
        glBegin(GL_LINE_STRIP);

          std::list<tf::Point>::iterator transformed_it = marker.transformed_points_.begin();
          std::list<QColor>::iterator color_it = marker.colors_.begin();
          for (; transformed_it != marker.transformed_points_.end(); ++transformed_it)
          {
            if (color_it != marker.colors_.end())
            {
              glColor4f(color_it->redF(), color_it->greenF(), color_it->blueF(), 1.0f);
              ++color_it;
            }

            glVertex2f(transformed_it->getX(), transformed_it->getY());
          }

        glEnd();
      }
      else if (marker.display_type_ == visualization_msgs::Marker::POINTS)
      {
        glPointSize(marker.scale_x_);
        glBegin(GL_POINTS);

          std::list<tf::Point>::iterator transformed_it = marker.transformed_points_.begin();
          std::list<QColor>::iterator color_it = marker.colors_.begin();
          for (; transformed_it != marker.transformed_points_.end(); ++transformed_it)
          {
            if (color_it != marker.colors_.end())
            {
              glColor4f(color_it->redF(), color_it->greenF(), color_it->blueF(), 1.0f);
              ++color_it;
            }

            glVertex2f(transformed_it->getX(), transformed_it->getY());
          }

        glEnd();
      }
      else if (marker.display_type_ == visualization_msgs::Marker::TRIANGLE_LIST)
      {
        glBegin(GL_TRIANGLES);

          std::list<tf::Point>::iterator transformed_it = marker.transformed_points_.begin();
          std::list<QColor>::iterator color_it = marker.colors_.begin();
          for (; transformed_it != marker.transformed_points_.end(); ++transformed_it)
          {
            if (color_it != marker.colors_.end())
            {
              glColor4f(color_it->redF(), color_it->greenF(), color_it->blueF(), 1.0f);
              ++color_it;
            }

            //ROS_ERROR("Drawing point at %f %f", transformed_it->getX(), transformed_it->getY());

            glVertex2f(transformed_it->getX(), transformed_it->getY());
          }

        glEnd();
      }
      else if (marker.display_type_ == visualization_msgs::Marker::CYLINDER)
      {
        std::list<tf::Point>::iterator transformed_it = marker.transformed_points_.begin();
        std::list<QColor>::iterator color_it = marker.colors_.begin();
        for (; transformed_it != marker.transformed_points_.end(); ++transformed_it)
        {
          if (color_it != marker.colors_.end())
          {
            glColor4f(color_it->redF(), color_it->greenF(), color_it->blueF(), 1.0f);
            ++color_it;
          }

          glBegin(GL_TRIANGLE_FAN);

            float x = transformed_it->getX();
            float y = transformed_it->getY();

            glVertex2f(x, y);

            for (int i = 0; i <= 360; i += 10)
            {
              glVertex2f(x + std::sin((float)i * 0.0174532925) * marker.scale_x_, y + std::cos((float)i * 0.0174532925) * marker.scale_y_);
            }

          glEnd();
        }
      }
    }
  }

  void MarkerPlugin::Transform()
  {
    std::map<int, MarkerData>::iterator markerIter;
    for (markerIter = markers_.begin(); markerIter != markers_.end(); ++markerIter)
    {
      MarkerData& marker = markerIter->second;

      std::list<tf::Point>::iterator points_it = marker.points_.begin();
      std::list<tf::Point>::iterator transformed_it = marker.transformed_points_.begin();
      for (; points_it != marker.points_.end() && transformed_it != marker.transformed_points_.end(); ++points_it)
      {
        (*transformed_it) = transform_ * (*points_it);

        ++transformed_it;
      }
    }
  }

  void MarkerPlugin::LoadConfiguration(const YAML::Node& node, const std::string& config_path)
  {
    std::string topic;
    node["topic"] >> topic;
    ui_.topic->setText(topic.c_str());

    node["is_marker_array"] >> is_marker_array_;

    TopicEdited();
  }

  void MarkerPlugin::SaveConfiguration(YAML::Emitter& emitter, const std::string& config_path)
  {
    emitter << YAML::Key << "topic" << YAML::Value << ui_.topic->text().toStdString();
    emitter << YAML::Key << "is_marker_array" << YAML::Value << is_marker_array_;
  }

}

