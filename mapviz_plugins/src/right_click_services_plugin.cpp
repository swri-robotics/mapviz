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

// *****************************************************************************
//  handling right click event, calls service which will display a
//  number of services that are available on that specific location
//
// *****************************************************************************
#include <mapviz_plugins/right_click_services_plugin.h>
#include <geometry_msgs/PointStamped.h>
#include <swri_transform_util/frames.h>
#include <swri_yaml_util/yaml_util.h>
#include <boost/shared_ptr.hpp>
#include <mapviz/map_canvas.h>
#include <Qt>
#include <mapviz_plugins/GPSCommand.h>
#include <std_srvs/Trigger.h>
#include <iostream>
#include <fstream>
#include <std_msgs/Bool.h>

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mapviz_plugins::RightClickServicesPlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{
  RightClickServicesPlugin::RightClickServicesPlugin() :
    config_widget_(new QWidget()),
    canvas_(NULL)
  {
    ui_.setupUi(config_widget_);

    connect(&click_filter_, SIGNAL(pointClicked(const QPointF&, const Qt::MouseButton&)),
            this, SLOT(pointClicked(const QPointF&, const Qt::MouseButton&)));
    connect(ui_.topic, SIGNAL(textEdited(const QString&)),
            this, SLOT(topicChanged(const QString&)));

    connect(ui_.availableservicestopic, SIGNAL(textEdited(const QString&)), this, SLOT(availableServiceTopicChanged(const QString&)));
    connect(ui_.gpscommandtopic, SIGNAL(textEdited(const QString&)), this, SLOT(gpsCommandTopicChanged(const QString&)));


    frame_timer_.start(1000);
    connect(&frame_timer_, SIGNAL(timeout()), this, SLOT(updateFrames()));

  }

  RightClickServicesPlugin::~RightClickServicesPlugin()
  {
    if (canvas_)
    {
      canvas_->removeEventFilter(&click_filter_);
    }
  }

  bool RightClickServicesPlugin::Initialize(QGLWidget* canvas)
  {
    canvas_ = static_cast<mapviz::MapCanvas*>(canvas);
    canvas_->installEventFilter(&click_filter_);

    PrintInfo("Ready.");


    return true;
  }
  void RightClickServicesPlugin::availableServiceTopicChanged(const QString& topic)
  {
    std::stringstream ss;
    RightClickServicesPlugin::available_service_topic_=topic.toStdString().c_str();
    ss << "switched available service topic to: " << topic.toStdString().c_str();
    PrintInfo(ss.str());
  }
  void RightClickServicesPlugin::gpsCommandTopicChanged(const QString& topic)
  {
    std::stringstream ss;
    RightClickServicesPlugin::gps_command_execute_topic_=topic.toStdString().c_str();
    ss << "switched gps command service topic to: " << topic.toStdString().c_str();
    PrintInfo(ss.str());
  }

  void RightClickServicesPlugin::Draw(double x, double y, double scale)
  {
  }

  void RightClickServicesPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    std::string tmp;
    if (swri_yaml_util::FindValue(node, "topic"))
    {
      node["topic"] >> tmp;
      ui_.topic->setText(QString(tmp.c_str()));
      topicChanged(ui_.topic->text());
    }

    if (swri_yaml_util::FindValue(node, "output_frame"))
    {
      node["output_frame"] >> tmp;
      ui_.outputframe->addItem(QString(tmp.c_str()));
    }
    if (swri_yaml_util::FindValue(node, "gpscommandtopic"))
    {
      node["gpscommandtopic"] >> tmp;
      ui_.gpscommandtopic->setText(QString(tmp.c_str()));
      gpsCommandTopicChanged(ui_.gpscommandtopic->text());
    }
    if (swri_yaml_util::FindValue(node, "availableservicestopic"))
    {
      node["availableservicestopic"] >> tmp;
      ui_.availableservicestopic->setText(QString(tmp.c_str()));
      availableServiceTopicChanged(ui_.availableservicestopic->text());
    }

  }

  void RightClickServicesPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    emitter << YAML::Key << "topic" << YAML::Value << ui_.topic->text().toStdString();
    emitter << YAML::Key << "output_frame" << YAML::Value << ui_.outputframe->currentText().toStdString();
    emitter << YAML::Key << "availableservicestopic" << YAML::Value << ui_.availableservicestopic->text().toStdString();
    emitter << YAML::Key << "gpscommandtopic" << YAML::Value << ui_.gpscommandtopic->text().toStdString();
  }

  QWidget* RightClickServicesPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }


  void RightClickServicesPlugin::pointClicked(const QPointF & point, const Qt::MouseButton& button)
  {
    QPointF transformed = canvas_->MapGlCoordToFixedFrame(point);

    std::string output_frame = ui_.outputframe->currentText().toStdString();

    if (target_frame_ != output_frame)
    {
      swri_transform_util::Transform tf;
      tf::Point tfPoint(transformed.x(), transformed.y(), 0.0);
      if (tf_manager_.GetTransform(output_frame, target_frame_, tf))
      {
        tfPoint = tf * tfPoint;
      }
      else
      {
        std::stringstream error;
        error << "Unable to find transform from " << target_frame_ << " to " << output_frame << ".";
        PrintError(error.str());
        return;
      }
      transformed.setX(tfPoint.x());
      transformed.setY(tfPoint.y());
    }

    std::stringstream ss;
    ss << "Point in " << output_frame.c_str() << ": " << transformed.x() << "," << transformed.y();
    PrintInfo(ss.str());


    boost::shared_ptr<geometry_msgs::PointStamped> stamped = boost::make_shared<geometry_msgs::PointStamped>();
    stamped->header.frame_id = output_frame;
    stamped->header.stamp = ros::Time::now();
    stamped->point.x = transformed.x();
    stamped->point.y = transformed.y();
    stamped->point.z = 0.0;

    point_publisher_.publish(stamped);

    if (button==Qt::RightButton){
        const QPoint pos=point.toPoint();

        showContextMenu(pos,stamped);
    }
  }

  void RightClickServicesPlugin::SetNode(const ros::NodeHandle& node)
  {
    mapviz::MapvizPlugin::SetNode(node);

    // We override this method so that we can initialize our publisher after
    // our node has been set, ensuring that it's in mapviz's namespace.
    topicChanged(ui_.topic->text());
  }

  void RightClickServicesPlugin::PrintError(const std::string& message)
  {
    PrintErrorHelper(ui_.status, message);
  }

  void RightClickServicesPlugin::PrintInfo(const std::string& message)
  {
    PrintInfoHelper(ui_.status, message);
  }

  void RightClickServicesPlugin::PrintWarning(const std::string& message)
  {
    PrintWarningHelper(ui_.status, message);
  }


  void RightClickServicesPlugin::topicChanged(const QString& topic)
  {
    std::stringstream ss;
    ss << "Publishing points to topic: " << topic.toStdString().c_str();
    PrintInfo(ss.str());

    if (!topic.isEmpty())
    {
      point_publisher_ = node_.advertise<geometry_msgs::PointStamped>(topic.toStdString(), 1000);
    }
  }

  void RightClickServicesPlugin::updateFrames()
  {
    std::vector<std::string> frames;
    tf_->getFrameStrings(frames);

    bool supports_wgs84 = tf_manager_.SupportsTransform(
        swri_transform_util::_local_xy_frame,
        swri_transform_util::_wgs84_frame);

    if (supports_wgs84)
    {
      frames.push_back(swri_transform_util::_wgs84_frame);
    }

    if (ui_.outputframe->count() >= 0 &&
        static_cast<size_t>(ui_.outputframe->count()) == frames.size())
    {
      bool changed = false;
      for (size_t i = 0; i < frames.size(); i++)
      {
        if (frames[i] != ui_.outputframe->itemText(static_cast<int>(i)).toStdString())
        {
          changed = true;
        }
      }

      if (!changed)
        return;
    }

    std::string current_output = ui_.outputframe->currentText().toStdString();

    ui_.outputframe->clear();
    for (size_t i = 0; i < frames.size(); i++)
    {
      ui_.outputframe->addItem(frames[i].c_str());
    }

    if (current_output != "")
    {
      int index = ui_.outputframe->findText(current_output.c_str());
      if (index < 0)
      {
        ui_.outputframe->addItem(current_output.c_str());
      }

      index = ui_.outputframe->findText(current_output.c_str());
      ui_.outputframe->setCurrentIndex(index);
    }
  }
  void RightClickServicesPlugin::showContextMenu(const QPoint& pos,boost::shared_ptr<geometry_msgs::PointStamped> stamped)
  {
      // retreive Available Services,Commands
      ros::NodeHandle n;
      ros::service::waitForService(RightClickServicesPlugin::available_service_topic_,10);
      ros::ServiceClient client = n.serviceClient<mapviz_plugins::GPSCommand>(RightClickServicesPlugin::available_service_topic_);
      mapviz_plugins::GPSCommand srv;
      srv.request.command = "";
      srv.request.location.longitude =stamped->point.x;
      srv.request.location.latitude =stamped->point.y;
      std::vector<std::string> service_name_list;
      if(client.exists())
      {
          if (client.call(srv))
          {
          service_name_list= srv.response.message;
          }
      }
      //call service to execute chosen command
      ros::service::waitForService(RightClickServicesPlugin::gps_command_execute_topic_,10);
      ros::ServiceClient command_client = n.serviceClient<mapviz_plugins::GPSCommand>(RightClickServicesPlugin::gps_command_execute_topic_);

      mapviz_plugins::GPSCommand  gps_command;
      gps_command.request.command = canvas_->showCustomContextMenu(canvas_->mapToGlobal(pos),service_name_list);
      gps_command.request.location.longitude =stamped->point.x;
      gps_command.request.location.latitude =stamped->point.y;
      if (gps_command.request.command.length()<=0){
        PrintInfo("Empty Service was not called");
      }else{
          if (command_client.exists())
          {
              if(command_client.call(gps_command))
              {
                  std::stringstream ss;
                  if(gps_command.response.success)
                  {
                     ss << "Calling " << gps_command.request.command << " was successful " << gps_command.response.message[0] ;
                  }else{
                      ss << "Calling " << gps_command.request.command << " was unsuccessful "<< gps_command.response.message[0];
                  }
                  PrintInfo(ss.str());
          }
          else
          {
            ss << "Calling " << gps_command.request.command << " was unsuccessful "<< gps_command.response.message;
          }
          PrintInfo(ss.str());
        }
      }
    }
  }
