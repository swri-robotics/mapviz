// *****************************************************************************
//
// Copyright (c) 2017, Southwest Research Institute® (SwRI®)
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

#include <mapviz_plugins/move_base_plugin.h>

// C++ standard libraries
#include <cstdio>
#include <vector>

// QT libraries
#include <QDateTime>
#include <QDialog>
#include <QGLWidget>
#include <QMouseEvent>
#include <QPainter>
#include <QPalette>
#include <QStaticText>
#include <QDebug>
#include <QSettings>
#include <fstream>

// ROS libraries
#include <ros/master.h>
#include <swri_transform_util/frames.h>

// Declare plugin
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(mapviz_plugins::MoveBasePlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{

MoveBasePlugin::MoveBasePlugin() :
    config_widget_(new QWidget()),
    map_canvas_(NULL),
    is_mouse_down_(false),
    move_base_client_("move_base", true),
    monitoring_action_state_(false)
{
    init_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, false);

    ui_.setupUi(config_widget_);

    // Set background white
    QPalette p(config_widget_->palette());
    p.setColor(QPalette::Background, Qt::white);
    config_widget_->setPalette(p);
    // Set status text red

    ui_.status->setText("OK");
    QPalette p3(ui_.status->palette());
    p3.setColor(QPalette::Text, Qt::green);
    ui_.status->setPalette(p3);

    QObject::connect(ui_.pushButtonInitialPose, SIGNAL(&QPushButton::toggled),
                     this, SLOT(&MoveBasePlugin::on_pushButtonInitialPose_toggled));

    QObject::connect(ui_.pushButtonGoalPose, SIGNAL(&QPushButton::toggled),
                     this, SLOT(&MoveBasePlugin::on_pushButtonGoalPose_toggled));

    QObject::connect(ui_.pushButtonAbort, SIGNAL(&QPushButton::clicked),
                     this, SLOT(&MoveBasePlugin::on_pushButtonAbort_clicked));

    timer_ = nh_.createTimer(ros::Duration(1.0), &MoveBasePlugin::timerCallback, this);

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
    PrintErrorHelper( ui_.status, message);
}

void MoveBasePlugin::PrintInfo(const std::string& message)
{
    PrintInfoHelper( ui_.status, message);
}

void MoveBasePlugin::PrintWarning(const std::string& message)
{
    PrintWarningHelper( ui_.status, message);
}

QWidget* MoveBasePlugin::GetConfigWidget(QWidget* parent)
{
    config_widget_->setParent(parent);
    return config_widget_;
}

bool MoveBasePlugin::Initialize(QGLWidget* canvas)
{
    map_canvas_ = static_cast<mapviz::MapCanvas*>(canvas);
    map_canvas_->installEventFilter(this);
    initialized_ = true;
    return true;
}

bool MoveBasePlugin::eventFilter(QObject *object, QEvent* event)
{
    switch (event->type())
    {
    case QEvent::MouseButtonPress:
        return handleMousePress(static_cast<QMouseEvent*>(event));
    case QEvent::MouseButtonRelease:
        return handleMouseRelease(static_cast<QMouseEvent*>(event));
    case QEvent::MouseMove:
        return handleMouseMove(static_cast<QMouseEvent*>(event));
    default:
        return false;
    }
}

void MoveBasePlugin::timerCallback(const ros::TimerEvent &)
{
    bool connected =  move_base_client_.isServerConnected();
    ui_.pushButtonAbort->setEnabled( connected );
    ui_.pushButtonGoalPose->setEnabled( connected );
    ui_.pushButtonInitialPose->setEnabled( connected );

    if(!connected)
    {
        PrintErrorHelper( ui_.status, "[move_base] server not connected");
    }
    else if( !monitoring_action_state_ ){
        PrintInfoHelper( ui_.status, "Ready to send command");
    }
    else{
        actionlib::SimpleClientGoalState state = move_base_client_.getState();
        switch( state.state_ )
        {
        case actionlib::SimpleClientGoalState::PENDING:
            PrintWarningHelper( ui_.status, state.toString() );
            break;

        case actionlib::SimpleClientGoalState::PREEMPTED:
            PrintWarningHelper( ui_.status, state.toString() );
            monitoring_action_state_ = false;
            break;

        case actionlib::SimpleClientGoalState::ACTIVE:
            PrintInfoHelper( ui_.status, state.toString() );
            break;

        case actionlib::SimpleClientGoalState::SUCCEEDED:
            PrintInfoHelper( ui_.status, state.toString() );
            monitoring_action_state_ = false;
            break;

        case actionlib::SimpleClientGoalState::REJECTED:
        case actionlib::SimpleClientGoalState::ABORTED:
        case actionlib::SimpleClientGoalState::LOST:
        case actionlib::SimpleClientGoalState::RECALLED:
            PrintErrorHelper( ui_.status, state.toString() );
            monitoring_action_state_ = false;
            break;
        }
    }
}


bool MoveBasePlugin::handleMousePress(QMouseEvent* event)
{
    bool init_checked = ui_.pushButtonInitialPose->isChecked();
    bool goal_checked = ui_.pushButtonGoalPose->isChecked();
    if( !init_checked && !goal_checked)
    {
        return false;
    }

    if (event->button() == Qt::LeftButton)
    {
        is_mouse_down_ = true;
        arrow_angle_ = 0;
#if QT_VERSION >= 0x050000
      arrow_tail_position_= map_canvas_->MapGlCoordToFixedFrame( event->localPos() );
#else
      arrow_tail_position_= map_canvas_->MapGlCoordToFixedFrame( event->posF() );
#endif
        return true;
    }
    return false;
}

bool MoveBasePlugin::handleMouseMove(QMouseEvent* event)
{
    if (is_mouse_down_)
    {
#if QT_VERSION >= 0x050000
        QPointF head_pos = map_canvas_->MapGlCoordToFixedFrame( event->localPos() );
#else
        QPointF head_pos = map_canvas_->MapGlCoordToFixedFrame( event->posF() );
#endif
        arrow_angle_ = atan2( head_pos.y() - arrow_tail_position_.y(),
                              head_pos.x() - arrow_tail_position_.x() );
    }
    return false;
}

bool MoveBasePlugin::handleMouseRelease(QMouseEvent* event)
{
    if( !is_mouse_down_ )
    {
        return false;
    }

    is_mouse_down_ = false;

    bool init_checked = ui_.pushButtonInitialPose->isChecked();
    bool goal_checked = ui_.pushButtonGoalPose->isChecked();
    if( !init_checked && !goal_checked)
    {
        return false;
    }

    tf::Quaternion quat = tf::createQuaternionFromYaw(arrow_angle_);

    if( goal_checked ){

        move_base_msg_.action_goal.header.frame_id = target_frame_;
        move_base_msg_.action_goal.header.stamp = ros::Time::now();
        move_base_msg_.action_goal.goal_id.stamp = move_base_msg_.action_goal.header.stamp;
        move_base_msg_.action_goal.goal_id.id = "mapviz_goal";
        move_base_msg_.action_goal.goal.target_pose.header = move_base_msg_.action_goal.header;

        geometry_msgs::Pose& pose = move_base_msg_.action_goal.goal.target_pose.pose;
        pose.position.x = arrow_tail_position_.x();
        pose.position.y = arrow_tail_position_.y();
        pose.position.z = 0.0;
        tf::quaternionTFToMsg( quat, pose.orientation );

        move_base_client_.sendGoal(move_base_msg_.action_goal.goal);
        ui_.pushButtonGoalPose->setChecked(false);
        monitoring_action_state_ = true;
    }
    if( init_checked ){
        geometry_msgs::PoseWithCovarianceStamped initpose;
        initpose.header.frame_id = target_frame_;
        initpose.header.stamp = ros::Time::now();
        initpose.pose.pose.position.x = arrow_tail_position_.x();
        initpose.pose.pose.position.y = arrow_tail_position_.y();
        initpose.pose.pose.position.z = 0.0;
        tf::quaternionTFToMsg( quat, initpose.pose.pose.orientation );

        init_pose_pub_.publish(initpose);
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

    if( is_mouse_down_ )
    {
        QPointF transformed_points[7];
        for (size_t i=0; i<7; i++ )
        {
            tf::Vector3 point(arrow_points[i].x(), arrow_points[i].y(), 0);
            point *= scale*10;
            point = tf::Transform( tf::createQuaternionFromYaw(arrow_angle_)  ) * point;
            transformed_points[i] = QPointF(point.x() + arrow_tail_position_.x(),
                                            point.y() + arrow_tail_position_.y() );
        }
        glColor3f(0.1, 0.9, 0.1);
        glLineWidth(2);
        glBegin(GL_TRIANGLE_FAN);
        for (const QPointF& point: transformed_points )
        {
            glVertex2d(point.x(), point.y());
        }
        glEnd();

        glColor3f(0.0, 0.6, 0.0);
        glBegin(GL_LINE_LOOP);
        for (const QPointF& point: transformed_points )
        {
            glVertex2d(point.x(), point.y());
        }
        glEnd();
    }
}


void MoveBasePlugin::LoadConfig(const YAML::Node& node, const std::string& path)
{

}

void MoveBasePlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
{

}

void MoveBasePlugin::on_pushButtonInitialPose_toggled(bool checked)
{
    const bool other_checked = ui_.pushButtonGoalPose->isChecked();

    if(checked){
        if(other_checked){
            ui_.pushButtonGoalPose->setChecked(false);
        }
        else{
            QPixmap cursor_pixmap = QPixmap(":/images/green-arrow.png");
            QApplication::setOverrideCursor(QCursor(cursor_pixmap));
        }
    }
    if( !checked && !other_checked )
    {
        QApplication::restoreOverrideCursor();
    }
}

void MoveBasePlugin::on_pushButtonGoalPose_toggled(bool checked)
{
    const bool other_checked = ui_.pushButtonInitialPose->isChecked();
    if(checked){
        if( other_checked){
            ui_.pushButtonInitialPose->setChecked(false);
        }
        else{
            QPixmap cursor_pixmap = QPixmap(":/images/green-arrow.png");
            QApplication::setOverrideCursor(QCursor(cursor_pixmap));
        }
    }
    if( !checked && !other_checked )
    {
        QApplication::restoreOverrideCursor();
    }

}

void MoveBasePlugin::on_pushButtonAbort_clicked()
{
    move_base_client_.cancelGoal();
}

}
