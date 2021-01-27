// *****************************************************************************
//
// Copyright (c) 2021, Trinity University
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

#include <follow_waypoints_mapviz_plugin.h>


// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mapviz_plugins::FollowWaypointsPlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{

FollowWaypointsPlugin::FollowWaypointsPlugin() :
    config_widget_(new QWidget())
{
    ui_.setupUi(config_widget_);

    // Set background white
    QPalette p(config_widget_->palette());
    p.setColor(QPalette::Background, Qt::white);
    config_widget_->setPalette(p);

    // Set status text green
    ui_.status->setText("OK");
    QPalette p3(ui_.status->palette());
    p3.setColor(QPalette::Text, Qt::green);
    ui_.status->setPalette(p3);

    // Execute block
    QObject::connect(ui_.pushButtonClearQueue, SIGNAL(toggled(bool)),
                     this, SLOT(on_pushButtonClearQueue_toggled(bool)));
    QObject::connect(ui_.pushButtonExecute, SIGNAL(toggled(bool)),
                     this, SLOT(on_pushButtonExecute_toggled(bool)));
    QObject::connect(ui_.pushButtonCancelNav, SIGNAL(toggled(bool)),
                     this, SLOT(on_pushButtonCancelNav_toggled(bool)));
}

FollowWaypointsPlugin::~FollowWaypointsPlugin()
{
}

void FollowWaypointsPlugin::PrintError(const std::string& message)
{
    PrintErrorHelper( ui_.status, message);
}

void FollowWaypointsPlugin::PrintInfo(const std::string& message)
{
    PrintInfoHelper( ui_.status, message);
}

void FollowWaypointsPlugin::PrintWarning(const std::string& message)
{
    PrintWarningHelper( ui_.status, message);
}

QWidget* FollowWaypointsPlugin::GetConfigWidget(QWidget* parent)
{
    config_widget_->setParent(parent);
    return config_widget_;
}

bool FollowWaypointsPlugin::Initialize(QGLWidget* canvas)
{
    initialized_ = true;
    PrintInfo("FWP Initialized....");
    return true;
}

void FollowWaypointsPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
{
    PrintInfo("LoadConfig()");
}

void FollowWaypointsPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
{
}

void FollowWaypointsPlugin::on_pushButtonCancelNav_toggled(bool checked)
{
    if(checked)
    {
      PrintInfo("Cancel Navigation...");
      actionlib_msgs::GoalID myMsg;
      ros::Publisher cancel_pub_ = node_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1, true);
      sleep(1); // give ROS time to get set up

      if (cancel_pub_.getNumSubscribers()==0)
      {
        PrintWarning("No subscriber to /move_base/cancel topic");
      }
      cancel_pub_.publish(myMsg);
      PrintInfo("published empty msg to /move_base/cancel.... STOP!");
      ui_.pushButtonCancelNav->setChecked(false);
    }
}

void FollowWaypointsPlugin::on_pushButtonClearQueue_toggled(bool checked)
{
    if(checked)
    {
      PrintInfo("Clear Waypoints...");
      std_msgs::Empty myMsg;
      ros::Publisher path_pub_ = node_.advertise<std_msgs::Empty>("/path_reset", 1, true);
      sleep(1); // give ROS time to get set up

      if (path_pub_.getNumSubscribers()==0 )
      {
        PrintWarning("No subscriber to /path_reset topic");
      }
      path_pub_.publish(myMsg);
      PrintInfo("published empty msg to /path_reset.... CLEAR!");
      ui_.pushButtonClearQueue->setChecked(false);
    }
}
void FollowWaypointsPlugin::on_pushButtonExecute_toggled(bool checked)
{
    if(checked)
    {
      PrintInfo("Execute Waypoints...");
      std_msgs::Empty myMsg;
      ros::Publisher path_pub_ = node_.advertise<std_msgs::Empty>("/path_ready", 1, true);
      sleep(1); // give ROS time to get set up
      if (path_pub_.getNumSubscribers()==0 )
      {
        PrintWarning("No subscriber to /path_ready topic");
      }
      path_pub_.publish(myMsg);
      PrintInfo("published empty msg to /path_ready.... GO!");
      ui_.pushButtonExecute->setChecked(false);
    }
}

}
