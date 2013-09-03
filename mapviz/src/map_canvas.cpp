// *****************************************************************************
//
// Copyright (C) 2013 All Right Reserved, Southwest Research Institute® (SwRI®)
//
// Contract No.  10-58058A
// Contractor    Southwest Research Institute® (SwRI®)
// Address       6220 Culebra Road, San Antonio, Texas 78228-0510
// Contact       Steve Dellenback <sdellenback@swri.org> (210) 522-3914
//
// This code was developed as part of an internal research project fully funded
// by Southwest Research Institute®.
//
// THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
// KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
//
// *****************************************************************************

// C++ standard libraries
#include <cmath>

#include <mapviz/map_canvas.h>

namespace mapviz
{
bool compare_plugins(MapvizPluginPtr a, MapvizPluginPtr b)
{
  return a->DrawOrder() < b->DrawOrder();
}

MapCanvas::MapCanvas(QWidget* parent) :
  QGLWidget(parent),
  initialized_(false),
  fix_orientation_(false),
  mouse_pressed_(false),
  mouse_x_(0),
  mouse_y_(0),
  offset_x_(0),
  offset_y_(0),
  drag_x_(0),
  drag_y_(0),
  view_center_x_(0),
  view_center_y_(0),
  view_scale_(1),
  view_left_(-25),
  view_right_(25),
  view_top_(10),
  view_bottom_(-10),
  scene_left_(-10),
  scene_right_(10),
  scene_top_(10),
  scene_bottom_(-10)
{
  ROS_INFO("View scale: %f meters/pixel", view_scale_);
}

MapCanvas::~MapCanvas()
{
}

void MapCanvas::InitializeTf(boost::shared_ptr<tf::TransformListener> tf)
{
  tf_ = tf;
}

void MapCanvas::initializeGL()
{
  glClearColor(0.58f, 0.56f, 0.5f, 1);
  glEnable(GL_POINT_SMOOTH);
  glEnable(GL_LINE_SMOOTH);
  glDisable(GL_POLYGON_SMOOTH);
  glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
  glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDepthFunc(GL_NEVER);
  glDisable(GL_DEPTH_TEST);
  initialized_ = true;
}

void MapCanvas::resizeGL(int w, int h)
{
  UpdateView();
}

void MapCanvas::paintGL()
{
  glClearColor(bg_color_.redF(), bg_color_.greenF(), bg_color_.blueF(), 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glPushMatrix();
  TransformTarget();

  // Draw test pattern
  glLineWidth(3);
  glBegin(GL_LINES);
    // Red line to the right
    glColor3f(1, 0, 0);
    glVertex2f(0, 0);
    glVertex2f(20, 0);

    // Green line to the top
    glColor3f(0, 1, 0);
    glVertex2f(0, 0);
    glVertex2f(0, 20);
  glEnd();

  std::list<MapvizPluginPtr>::iterator it;
  for (it = plugins_.begin(); it != plugins_.end(); ++it)
  {
    (*it)->DrawPlugin(view_center_x_, view_center_y_, view_scale_);
  }

  glPopMatrix();
}

void MapCanvas::wheelEvent(QWheelEvent* e)
{
  float numDegrees = e->delta() / -8;

  view_scale_ *= std::pow(1.1, numDegrees / 10.0);

  UpdateView();
}

void MapCanvas::mousePressEvent(QMouseEvent* e)
{
  mouse_x_ = e->x();
  mouse_y_ = e->y();
  drag_x_ = 0;
  drag_y_ = 0;
  mouse_pressed_ = true;
}

void MapCanvas::mouseReleaseEvent(QMouseEvent* e)
{
  mouse_pressed_ = false;
  offset_x_ += drag_x_;
  offset_y_ += drag_y_;
  drag_x_ = 0;
  drag_y_ = 0;
}

void MapCanvas::mouseMoveEvent(QMouseEvent* e)
{
  if (mouse_pressed_ && ((mouse_x_ -  e->x()) != 0 || (mouse_y_ - e->y()) != 0))
  {
    drag_x_ = -((mouse_x_ - e->x()) * view_scale_);
    drag_y_ = ((mouse_y_ - e->y()) * view_scale_);
    update();
  }
}

void MapCanvas::SetFixedFrame(const std::string& frame)
{
  fixed_frame_ = frame;
  std::list<MapvizPluginPtr>::iterator it;
  for (it = plugins_.begin(); it != plugins_.end(); ++it)
  {
    (*it)->SetTargetFrame(frame);
  }
  update();
}

void MapCanvas::SetTargetFrame(const std::string& frame)
{
  offset_x_ = 0;
  offset_y_ = 0;
  drag_x_ = 0;
  drag_y_ = 0;

  target_frame_ = frame;
  update();
}

void MapCanvas::ToggleFixOrientation(bool on)
{
  fix_orientation_ = on;
  update();
}

void MapCanvas::ToggleUseLatestTransforms(bool on)
{
  std::list<MapvizPluginPtr>::iterator it;
  for (it = plugins_.begin(); it != plugins_.end(); ++it)
  {
    (*it)->SetUseLatestTransforms(on);
  }

  update();
}

void MapCanvas::AddPlugin(MapvizPluginPtr plugin, int order)
{
  plugins_.push_back(plugin);
}

void MapCanvas::RemovePlugin(MapvizPluginPtr plugin)
{
  plugin->Shutdown();
  plugins_.remove(plugin);
  update();
}

void MapCanvas::TransformTarget()
{
  glTranslatef(offset_x_ + drag_x_, offset_y_ + drag_y_, 0);

  view_center_x_ = -offset_x_ - drag_x_;
  view_center_y_ = -offset_y_ - drag_y_;

  if (!tf_ || fixed_frame_.empty() || target_frame_.empty() || target_frame_ == "<none>")
    return;

  tf::StampedTransform transform;
  try
  {
    tf_->lookupTransform(fixed_frame_, target_frame_, ros::Time(0), transform);

    double roll, pitch, yaw;
    transform.getBasis().getRPY(roll, pitch, yaw);

    if (!fix_orientation_)
    {
      glRotatef(-yaw * 57.2957795, 0, 0, 1);
    }

    glTranslatef(-transform.getOrigin().getX(), -transform.getOrigin().getY(), 0);

    tf::Point point(view_center_x_, view_center_y_, 0);

    // If the viewer orientation is fixed don't rotate the center point.
    if (fix_orientation_)
    {
      transform.setRotation(tf::Transform::getIdentity().getRotation());
    }

    tf::Point center = transform * point;

    view_center_x_ = center.getX();
    view_center_y_ = center.getY();
  }
  catch (const tf::LookupException& e)
  {
    ROS_ERROR("%s", e.what());
  }
  catch (const tf::ConnectivityException& e)
  {
    ROS_ERROR("%s", e.what());
  }
  catch (const tf::ExtrapolationException& e)
  {
    ROS_ERROR("%s", e.what());
  }
  catch (...)
  {
    ROS_ERROR("Error looking up transform");;
  }
}

void MapCanvas::UpdateView()
{
  if (initialized_)
  {
    Recenter();

    glViewport(0, 0, width(), height());
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(view_left_, view_right_, view_top_, view_bottom_, -0.5f, 0.5f);

    update();
  }
}

void MapCanvas::ReorderDisplays()
{
  plugins_.sort(compare_plugins);
}

void MapCanvas::Recenter()
{
  // Recalculate the bounds of the view
  view_left_ = -(width() * view_scale_ * 0.5);
  view_top_ = -(height() * view_scale_ * 0.5);
  view_right_ = (width() * view_scale_ * 0.5);
  view_bottom_ = (height() * view_scale_ * 0.5);
}
}
