#include <mapviz_plugins/marker_plugin.h>

// C++ standard libraries
#include <cmath>
#include <cstdio>
#include <vector>

// QT libraries
#include <QDialog>
#include <QGLWidget>

// ROS libraries
#include <ros/master.h>

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(
    mapviz_plugins,
    marker,
    mapviz_plugins::MarkerPlugin,
    mapviz::MapvizPlugin)

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
      markerData.stamp = marker->header.stamp;
      markerData.display_type = marker->type;
      markerData.color = QColor::fromRgbF(marker->color.r, marker->color.g, marker->color.b, marker->color.a);
      markerData.scale_x = marker->scale.x;
      markerData.scale_y = marker->scale.y;
      markerData.scale_z = marker->scale.z;
      markerData.transformed = true;

      markerData.points.clear();

      tf::StampedTransform transform;
      if (!GetTransform(marker->header.stamp, transform))
      {
        markerData.transformed = false;
        PrintError("No transform between " + source_frame_ + " and " + target_frame_);
      }

      // TODO handle lifetime parameter

      // TODO correctly transform points based on the pose
      double x = marker->pose.position.x;
      double y = marker->pose.position.y;
      double z = marker->pose.position.z;

      if (markerData.display_type == visualization_msgs::Marker::CYLINDER)
      {
        StampedPoint point;
        point.point = tf::Point(x, y, z);
        point.transformed_point = transform * point.point;
        point.color = markerData.color;

        markerData.points.push_back(point);
      }
      else
      {
        for (unsigned int i = 0; i < marker->points.size(); i++)
        {
          StampedPoint point;
          point.point = tf::Point(marker->points[i].x + x, marker->points[i].y + y, marker->points[i].z + z);
          point.transformed_point = transform * point.point;

          if (i < marker->colors.size())
          {
            point.color = QColor::fromRgbF(
                marker->colors[i].r,
                marker->colors[i].g,
                marker->colors[i].b,
                marker->colors[i].a);
          }
          else
          {
            point.color = markerData.color;
          }

          markerData.points.push_back(point);
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
    for (unsigned int i = 0; i < markers->markers.size(); i++)
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

      if (marker.transformed)
      {
        glColor4f(marker.color.redF(), marker.color.greenF(), marker.color.blueF(), 1.0f);

        if (marker.display_type == visualization_msgs::Marker::LINE_STRIP)
        {
          glLineWidth(marker.scale_x);
          glBegin(GL_LINE_STRIP);

            std::list<StampedPoint>::iterator point_it = marker.points.begin();
            for (; point_it != marker.points.end(); ++point_it)
            {
              glColor4f(
                  point_it->color.redF(),
                  point_it->color.greenF(),
                  point_it->color.blueF(),
                  point_it->color.alphaF());

              glVertex2f(
                  point_it->transformed_point.getX(),
                  point_it->transformed_point.getY());
            }

          glEnd();
        }
        if (marker.display_type == visualization_msgs::Marker::LINE_LIST)
        {
          glLineWidth(marker.scale_x);
          glBegin(GL_LINES);

            std::list<StampedPoint>::iterator point_it = marker.points.begin();
            for (; point_it != marker.points.end(); ++point_it)
            {
              glColor4f(
                  point_it->color.redF(),
                  point_it->color.greenF(),
                  point_it->color.blueF(),
                  point_it->color.alphaF());

              glVertex2f(
                  point_it->transformed_point.getX(),
                  point_it->transformed_point.getY());
            }

          glEnd();
        }
        else if (marker.display_type == visualization_msgs::Marker::POINTS)
        {
          glPointSize(marker.scale_x);
          glBegin(GL_POINTS);

          std::list<StampedPoint>::iterator point_it = marker.points.begin();
          for (; point_it != marker.points.end(); ++point_it)
          {
            glColor4f(
                point_it->color.redF(),
                point_it->color.greenF(),
                point_it->color.blueF(),
                point_it->color.alphaF());

            glVertex2f(
                point_it->transformed_point.getX(),
                point_it->transformed_point.getY());
          }

          glEnd();
        }
        else if (marker.display_type == visualization_msgs::Marker::TRIANGLE_LIST)
        {
          glBegin(GL_TRIANGLES);

          std::list<StampedPoint>::iterator point_it = marker.points.begin();
          for (; point_it != marker.points.end(); ++point_it)
          {
            glColor4f(
                point_it->color.redF(),
                point_it->color.greenF(),
                point_it->color.blueF(),
                point_it->color.alphaF());

            glVertex2f(
                point_it->transformed_point.getX(),
                point_it->transformed_point.getY());
          }

          glEnd();
        }
        else if (marker.display_type == visualization_msgs::Marker::CYLINDER)
        {
          std::list<StampedPoint>::iterator point_it = marker.points.begin();
          for (; point_it != marker.points.end(); ++point_it)
          {
            glColor4f(
                point_it->color.redF(),
                point_it->color.greenF(),
                point_it->color.blueF(),
                point_it->color.alphaF());


            glBegin(GL_TRIANGLE_FAN);


            float x = point_it->transformed_point.getX();
            float y = point_it->transformed_point.getY();

            glVertex2f(x, y);

            for (int i = 0; i <= 360; i += 10)
            {
              glVertex2f(
                  x + std::sin((float)i * 0.0174532925) * marker.scale_x,
                  y + std::cos((float)i * 0.0174532925) * marker.scale_y);
            }

            glEnd();
          }
        }

        PrintInfo("OK");
      }
    }
  }

  void MarkerPlugin::Transform()
  {
    std::map<int, MarkerData>::iterator markerIter;
    for (markerIter = markers_.begin(); markerIter != markers_.end(); ++markerIter)
    {
      MarkerData& marker = markerIter->second;

      tf::StampedTransform transform;
      if (GetTransform(marker.stamp, transform))
      {
        marker.transformed = true;

        std::list<StampedPoint>::iterator point_it = marker.points.begin();
        for (; point_it != marker.points.end(); ++point_it)
        {
          point_it->transformed_point = transform * point_it->point;
        }
      }
      else
      {
        marker.transformed = false;
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

