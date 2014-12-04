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
//     * Neither the name of the <organization> nor the
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

#include <mapviz/map_canvas.h>

// C++ standard libraries
#include <cmath>

namespace mapviz
{
bool compare_plugins(MapvizPluginPtr a, MapvizPluginPtr b)
{
  return a->DrawOrder() < b->DrawOrder();
}

MapCanvas::MapCanvas(QWidget* parent) :
  QGLWidget(parent),
  has_pixel_buffers_(false),
  pixel_buffer_size_(0),
  pixel_buffer_index_(0),
  capture_frames_(false),
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
  setMouseTracking(true);
}

MapCanvas::~MapCanvas()
{
  if(pixel_buffer_size_ != 0)
  {
    glDeleteBuffersARB(2, pixel_buffer_ids_);
  }
}

void MapCanvas::InitializeTf(boost::shared_ptr<tf::TransformListener> tf)
{
  tf_ = tf;
}

void MapCanvas::InitializePixelBuffers()
{
  if(has_pixel_buffers_)
  {
    int32_t buffer_size = width() * height() * 4;
    
    if (pixel_buffer_size_ != buffer_size)
    {
      if (pixel_buffer_size_ != 0)
      {
        glDeleteBuffersARB(2, pixel_buffer_ids_);
      }
    
      glGenBuffersARB(2, pixel_buffer_ids_);
      glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, pixel_buffer_ids_[0]);
      glBufferDataARB(GL_PIXEL_PACK_BUFFER_ARB, buffer_size, 0, GL_STREAM_READ_ARB);
      glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, pixel_buffer_ids_[1]);
      glBufferDataARB(GL_PIXEL_PACK_BUFFER_ARB, buffer_size, 0, GL_STREAM_READ_ARB);
      glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, 0);
      
      pixel_buffer_size_ = buffer_size;
    }
  }
}

void MapCanvas::initializeGL()
{
  GLenum err = glewInit();
  if (GLEW_OK != err)
  {
    ROS_ERROR("Error: %s\n", glewGetErrorString(err));
  }
  else
  {
    // Check if pixel buffers are available for asynchronous capturing
    std::string extensions = (const char*)glGetString(GL_EXTENSIONS);
    has_pixel_buffers_ = extensions.find("GL_ARB_pixel_buffer_object") != std::string::npos;  
  }

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

void MapCanvas::CaptureFrame(bool force)
{
  // Ensure the pixel size is actually 4
  glPixelStorei(GL_PACK_ALIGNMENT, 4);
  
  if (has_pixel_buffers_ && !force)
  {
    InitializePixelBuffers();
    
    pixel_buffer_index_ = (pixel_buffer_index_ + 1) % 2;
    int32_t next_index = (pixel_buffer_index_ + 1) % 2;
    
    glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, pixel_buffer_ids_[pixel_buffer_index_]);
    glReadPixels(0, 0, width(), height(), GL_BGRA, GL_UNSIGNED_BYTE, 0);
    glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, pixel_buffer_ids_[next_index]);
    GLubyte* data = (GLubyte*)glMapBufferARB(GL_PIXEL_PACK_BUFFER_ARB, GL_READ_ONLY_ARB);
    if(data)
    {
      capture_buffer_.clear();
      capture_buffer_.resize(pixel_buffer_size_);
      
      memcpy(&capture_buffer_[0], data, pixel_buffer_size_);
      
      glUnmapBufferARB(GL_PIXEL_PACK_BUFFER_ARB);
    }
    glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, 0);
  }
  else
  {
    int32_t buffer_size = width() * height() * 4;
    capture_buffer_.clear();
    capture_buffer_.resize(buffer_size);

    glReadPixels(0, 0, width(), height(), GL_BGRA, GL_UNSIGNED_BYTE, &capture_buffer_[0]);
  }
}

void MapCanvas::paintGL()
{
  if (capture_frames_)
  {
    CaptureFrame();
  }
  
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
  
  double center_x = -offset_x_ - drag_x_;
  double center_y = -offset_y_ - drag_y_;
  double x = center_x + (e->x() - width() / 2.0) * view_scale_;
  double y = center_y + (height() / 2.0  - e->y()) * view_scale_;
  Q_EMIT Hover(x, y, view_scale_);
}

void MapCanvas::leaveEvent(QEvent* e)
{
  Q_EMIT Hover(0, 0, 0);
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
    ROS_ERROR("Error looking up transform");
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
