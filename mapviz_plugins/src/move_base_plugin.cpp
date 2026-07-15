// *****************************************************************************
//
// Copyright (c) 2026, Southwest Research Institute® (SwRI®)
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

#include <mapviz_plugins/move_base_plugin.hpp>
#include <mapviz/qt_mouse_event_compat.hpp>

// C++ standard libraries
#include <array>
#include <cstdio>
#include <string>

// QT libraries
#include <QApplication>
#include <QCursor>
#include <QMouseEvent>
#include <QPainter>
#include <QPalette>
#include <QPixmap>

// ROS libraries
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Declare plugin
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mapviz_plugins::MoveBasePlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{

MoveBasePlugin::MoveBasePlugin()
: config_widget_(new QWidget())
, map_canvas_(nullptr)
, action_status_(IDLE)
, is_mouse_down_(false)
, arrow_angle_(0.0)
{
  ui_.setupUi(config_widget_);

  // Set background white
  QPalette p(config_widget_->palette());
  p.setColor(QPalette::Window, Qt::white);
  config_widget_->setPalette(p);

  ui_.status->setText("OK");
  QPalette p3(ui_.status->palette());
  p3.setColor(QPalette::Text, Qt::green);
  ui_.status->setPalette(p3);

  QObject::connect(ui_.pushButtonInitialPose, &QPushButton::toggled,
                   this, &MoveBasePlugin::on_pushButtonInitialPose_toggled);
  QObject::connect(ui_.pushButtonGoalPose, &QPushButton::toggled,
                   this, &MoveBasePlugin::on_pushButtonGoalPose_toggled);
  QObject::connect(ui_.pushButtonAbort, &QPushButton::clicked,
                   this, &MoveBasePlugin::on_pushButtonAbort_clicked);

  // The action-client callbacks fire on the ROS spin thread; hand the status
  // to the GUI thread, which owns action_status_ and the widgets.  Qt makes
  // this a queued connection because the emitting thread differs from this
  // object's thread.
  QObject::connect(this, &MoveBasePlugin::ActionStatusChanged,
                   this, &MoveBasePlugin::handleActionStatus);
}

MoveBasePlugin::~MoveBasePlugin()
{
  if (map_canvas_)
  {
    map_canvas_->removeEventFilter(this);
  }
}

void MoveBasePlugin::PrintError(const std::string& message)
{
  PrintErrorHelper(ui_.status, message, 1.0);
}

void MoveBasePlugin::PrintInfo(const std::string& message)
{
  PrintInfoHelper(ui_.status, message, 1.0);
}

void MoveBasePlugin::PrintWarning(const std::string& message)
{
  PrintWarningHelper(ui_.status, message, 1.0);
}

QWidget* MoveBasePlugin::GetConfigWidget(QWidget* parent)
{
  config_widget_->setParent(parent);
  return config_widget_;
}

bool MoveBasePlugin::Initialize(QOpenGLWidget* canvas)
{
  map_canvas_ = dynamic_cast<mapviz::MapCanvas*>(canvas);
  map_canvas_->installEventFilter(this);
  canvas->makeCurrent();
  initializeOpenGLFunctions();
  canvas->doneCurrent();

  init_pose_pub_ = Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", rclcpp::QoS(1));
  move_base_client_ = rclcpp_action::create_client<NavigateToPose>(
      NodeUnsafe(), "navigate_to_pose");

  // A QTimer (not a ROS wall timer) so timerCallback() runs on the GUI thread,
  // which owns the widgets and status it touches.
  QObject::connect(&timer_, &QTimer::timeout, this, &MoveBasePlugin::timerCallback);
  timer_.start(1000);

  initialized_ = true;
  return true;
}

bool MoveBasePlugin::eventFilter(QObject* object, QEvent* event)
{
  switch (event->type())
  {
    case QEvent::MouseButtonPress:
      return handleMousePress(dynamic_cast<QMouseEvent*>(event));
    case QEvent::MouseButtonRelease:
      return handleMouseRelease(dynamic_cast<QMouseEvent*>(event));
    case QEvent::MouseMove:
      return handleMouseMove(dynamic_cast<QMouseEvent*>(event));
    default:
      return false;
  }
}

void MoveBasePlugin::handleActionStatus(int status)
{
  MAPVIZ_ASSERT_GUI_THREAD();
  action_status_ = static_cast<ActionStatus>(status);
}

void MoveBasePlugin::timerCallback()
{
  MAPVIZ_ASSERT_GUI_THREAD();
  const bool connected = move_base_client_ && move_base_client_->action_server_is_ready();
  ui_.pushButtonAbort->setEnabled(connected);
  ui_.pushButtonGoalPose->setEnabled(connected);
  ui_.pushButtonInitialPose->setEnabled(connected);

  if (!connected)
  {
    PrintError("[navigate_to_pose] server not connected");
    return;
  }

  switch (action_status_)
  {
    case ACTIVE:
      PrintInfo("Goal active");
      break;
    case SUCCEEDED:
      PrintInfo("Goal succeeded");
      break;
    case CANCELED:
      PrintWarning("Goal canceled");
      break;
    case REJECTED:
      PrintError("Goal rejected by server");
      break;
    case ABORTED:
      PrintError("Goal aborted by server");
      break;
    case IDLE:
    default:
      PrintInfo("Ready to send command");
      break;
  }
}

bool MoveBasePlugin::handleMousePress(QMouseEvent* event)
{
  const bool init_checked = ui_.pushButtonInitialPose->isChecked();
  const bool goal_checked = ui_.pushButtonGoalPose->isChecked();
  if (!init_checked && !goal_checked)
  {
    return false;
  }

  if (event->button() == Qt::LeftButton)
  {
    is_mouse_down_ = true;
    arrow_angle_ = 0;
    arrow_tail_position_ = map_canvas_->MapGlCoordToFixedFrame(mapviz::MouseEventPosition(event));
    return true;
  }
  return false;
}

bool MoveBasePlugin::handleMouseMove(QMouseEvent* event)
{
  if (is_mouse_down_)
  {
    QPointF head_pos = map_canvas_->MapGlCoordToFixedFrame(mapviz::MouseEventPosition(event));
    arrow_angle_ = atan2(head_pos.y() - arrow_tail_position_.y(),
                         head_pos.x() - arrow_tail_position_.x());
  }
  return false;
}

bool MoveBasePlugin::handleMouseRelease(QMouseEvent* event)
{
  if (!is_mouse_down_)
  {
    return false;
  }
  is_mouse_down_ = false;

  const bool init_checked = ui_.pushButtonInitialPose->isChecked();
  const bool goal_checked = ui_.pushButtonGoalPose->isChecked();
  if (!init_checked && !goal_checked)
  {
    return false;
  }

  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, arrow_angle_);

  if (goal_checked)
  {
    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = target_frame_;
    goal.pose.header.stamp = Now();
    goal.pose.pose.position.x = arrow_tail_position_.x();
    goal.pose.pose.position.y = arrow_tail_position_.y();
    goal.pose.pose.position.z = 0.0;
    goal.pose.pose.orientation = tf2::toMsg(quat);

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    // Both callbacks run on the ROS spin thread; they only emit a queued
    // signal, so they never touch plugin state or widgets directly.
    options.goal_response_callback =
      [this](GoalHandle::SharedPtr handle)
      {
        Q_EMIT ActionStatusChanged(handle ? ACTIVE : REJECTED);
      };
    options.result_callback =
      [this](const GoalHandle::WrappedResult& result)
      {
        int status = ABORTED;
        switch (result.code)
        {
          case rclcpp_action::ResultCode::SUCCEEDED:
            status = SUCCEEDED;
            break;
          case rclcpp_action::ResultCode::CANCELED:
            status = CANCELED;
            break;
          case rclcpp_action::ResultCode::ABORTED:
          default:
            status = ABORTED;
            break;
        }
        Q_EMIT ActionStatusChanged(status);
      };

    move_base_client_->async_send_goal(goal, options);
    action_status_ = ACTIVE;
    ui_.pushButtonGoalPose->setChecked(false);
  }
  if (init_checked)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped initpose;
    initpose.header.frame_id = target_frame_;
    initpose.header.stamp = Now();
    initpose.pose.pose.position.x = arrow_tail_position_.x();
    initpose.pose.pose.position.y = arrow_tail_position_.y();
    initpose.pose.pose.position.z = 0.0;
    initpose.pose.pose.orientation = tf2::toMsg(quat);

    init_pose_pub_->publish(initpose);
    ui_.pushButtonInitialPose->setChecked(false);
  }
  return true;
}

void MoveBasePlugin::Draw(double x, double y, double scale)
{
  std::array<QPointF, 7> arrow_points;
  arrow_points[0] = QPointF(10, 0);
  arrow_points[1] = QPointF(6, -2.5);
  arrow_points[2] = QPointF(6.5, -1);
  arrow_points[3] = QPointF(0, -1);
  arrow_points[4] = QPointF(0, 1);
  arrow_points[5] = QPointF(6.5, 1);
  arrow_points[6] = QPointF(6, 2.5);

  if (is_mouse_down_)
  {
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, arrow_angle_);
    tf2::Transform transform(quat);

    QPointF transformed_points[7];
    for (size_t i = 0; i < 7; i++)
    {
      tf2::Vector3 point(arrow_points[i].x(), arrow_points[i].y(), 0);
      point *= scale * 10;
      point = transform * point;
      transformed_points[i] = QPointF(point.x() + arrow_tail_position_.x(),
                                      point.y() + arrow_tail_position_.y());
    }
    glColor3f(0.1, 0.9, 0.1);
    glLineWidth(2);
    glBegin(GL_TRIANGLE_FAN);
    for (const QPointF& point : transformed_points)
    {
      glVertex2d(point.x(), point.y());
    }
    glEnd();

    glColor3f(0.0, 0.6, 0.0);
    glBegin(GL_LINE_LOOP);
    for (const QPointF& point : transformed_points)
    {
      glVertex2d(point.x(), point.y());
    }
    glEnd();
  }
}

void MoveBasePlugin::LoadConfig(const YAML::Node& /*node*/, const std::string& /*path*/)
{
}

void MoveBasePlugin::SaveConfig(YAML::Emitter& /*emitter*/, const std::string& /*path*/)
{
}

void MoveBasePlugin::on_pushButtonInitialPose_toggled(bool checked)
{
  const bool other_checked = ui_.pushButtonGoalPose->isChecked();

  if (checked)
  {
    if (other_checked)
    {
      ui_.pushButtonGoalPose->setChecked(false);
    }
    else
    {
      QPixmap cursor_pixmap = QPixmap(":/images/green-arrow.png");
      QApplication::setOverrideCursor(QCursor(cursor_pixmap));
    }
  }
  if (!checked && !other_checked)
  {
    QApplication::restoreOverrideCursor();
  }
}

void MoveBasePlugin::on_pushButtonGoalPose_toggled(bool checked)
{
  const bool other_checked = ui_.pushButtonInitialPose->isChecked();
  if (checked)
  {
    if (other_checked)
    {
      ui_.pushButtonInitialPose->setChecked(false);
    }
    else
    {
      QPixmap cursor_pixmap = QPixmap(":/images/green-arrow.png");
      QApplication::setOverrideCursor(QCursor(cursor_pixmap));
    }
  }
  if (!checked && !other_checked)
  {
    QApplication::restoreOverrideCursor();
  }
}

void MoveBasePlugin::on_pushButtonAbort_clicked()
{
  if (move_base_client_)
  {
    move_base_client_->async_cancel_all_goals();
  }
}

}   // namespace mapviz_plugins
