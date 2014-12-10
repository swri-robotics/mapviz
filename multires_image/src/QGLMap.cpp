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

#include <multires_image/QGLMap.h>

// C++ standard libraries
#include <cmath>

namespace multires_image
{
QGLMap::QGLMap(QWidget *parent) :
  QGLWidget(parent),
  m_initialized(false),
  m_scale(1.0),
  m_mouseDown(false),
  m_mouseDownX(0),
  m_mouseDownY(0),
  m_tileView(NULL),
  m_view_top_left(0, 0, 0),
  m_view_bottom_right(0, 0, 0),
  m_view_center(0, 0, 0),
  m_scene_top_left(0, 0, 0),
  m_scene_bottom_right(0, 0, 0),
  m_scene_center(0, 0, 0)
{
  ui.setupUi(this);
}

QGLMap::~QGLMap()
{
}

void QGLMap::Exit()
{
  if (m_tileView != NULL)
  {
    m_tileView->Exit();
  }
}

void QGLMap::UpdateView()
{
  if (m_initialized)
  {
    Recenter();

    if (m_tileView != NULL)
    {
      m_tileView->SetView(m_view_center.x(), m_view_center.y(), 1, m_scale);
    }

    glViewport(0, 0, width(), height());
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(m_view_top_left.x(), m_view_bottom_right.x(),
        m_view_bottom_right.y(), m_view_top_left.y(), -0.5f, 0.5f);

    update();

    // Signal a view change as occured.  The minimap listens for this
    // so that it can update its view box.
    emit SignalViewChange(m_view_top_left.x(), m_view_top_left.y(),
        m_view_bottom_right.x(), m_view_bottom_right.y());
  }
}

void QGLMap::SetTiles(TileSet* tiles)
{
  double top, left, bottom, right;
  tiles->GeoReference().GetCoordinate(0, 0, left, top);
  tiles->GeoReference().GetCoordinate(tiles->GeoReference().Width(), tiles->GeoReference().Height(), right, bottom);

  m_scene_top_left = tf::Point(left, top, 0);
  m_scene_bottom_right = tf::Point(right, bottom, 0);
  m_scene_center = (m_scene_top_left + m_scene_bottom_right) / 2.0;

  m_view_center = m_scene_center;

  m_tileView = new TileView(tiles, this);

  connect(m_tileView->Cache(), SIGNAL(SignalMemorySize(int64_t)),
    SLOT(SetTextureMemory(int64_t)));

  // Create connections for the texture loading functions which must
  // be executed on this object's thread.

  m_tileView->SetView(m_view_center.x(), m_view_center.y(), 1, m_scale);
}

void QGLMap::wheelEvent(QWheelEvent* e)
{
  float numDegrees = e->delta() / -8;

  m_scale *= pow(1.1, numDegrees / 10.0);

  UpdateView();
}

void QGLMap::LoadTexture(Tile* tile)
{
  tile->LoadTexture();
}

void QGLMap::DeleteTexture(Tile* tile)
{
  tile->UnloadTexture();
}

void QGLMap::SetTextureMemory(int64_t bytes)
{
  // Signal that the texture memory size has changed.  The status bar listens
  // to this so that the user can see how much memory the map is using.
  emit SignalMemorySize(bytes);
}

void QGLMap::ChangeCenter(double x, double y)
{
  if (x != 0)
    m_view_center.setX(x);

  if (y != 0)
    m_view_center.setY(y);

  UpdateView();
}

void QGLMap::initializeGL()
{
  glClearColor(0.58f, 0.56f, 0.5f, 1);
  glEnable(GL_POINT_SMOOTH);
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_POLYGON_SMOOTH);
  glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
  glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
  glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDepthFunc(GL_NEVER);
    glDisable(GL_DEPTH_TEST);
  m_initialized = true;
}

void QGLMap::resizeGL(int w, int h)
{
  UpdateView();
}

void QGLMap::paintGL()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  if (m_tileView != NULL)
  {
    m_tileView->Draw();
  }
}

void QGLMap::mousePressEvent(QMouseEvent* e)
{
  m_mouseDownX = e->x();
  m_mouseDownY = e->y();
  m_mouseDown = true;

  update();
}

void QGLMap::mouseDoubleClickEvent(QMouseEvent* e)
{
  update();
}

void QGLMap::mouseReleaseEvent(QMouseEvent* e)
{
  m_mouseDown = false;

  update();
}

void QGLMap::mouseMoveEvent(QMouseEvent* e)
{
  if (m_mouseDown)
    MousePan(e->x(), e->y());
}

void QGLMap::MousePan(int x, int y)
{
  bool changed = false;
  if (m_mouseDown)
  {
    double diffX = ((m_mouseDownX - x) * m_scale);
    double diffY = ((m_mouseDownY - y) * m_scale);

    if (diffX != 0)
    {
      m_view_center.setX(m_view_center.x() + diffX);
      m_mouseDownX = x;
      changed = true;
    }
    if (diffY != 0)
    {
      m_view_center.setY(m_view_center.y() + diffY);
      m_mouseDownY = y;
      changed = true;
    }
  }

  if (changed)
  {
    UpdateView();
  }
}

void QGLMap::Recenter()
{
  double scene_width = std::fabs(m_scene_top_left.x() - m_scene_bottom_right.x());
  double scene_height = std::fabs(m_scene_top_left.y() - m_scene_bottom_right.y());
  double view_width = width() * m_scale;
  double view_height = height() * m_scale;

  m_view_top_left.setX(m_view_center.x() - (view_width * 0.5));
  m_view_top_left.setY(m_view_center.y() - (view_width * 0.5));

  m_view_bottom_right.setX(m_view_center.x() + (view_width * 0.5));
  m_view_bottom_right.setY(m_view_center.y() + (view_width * 0.5));

  if (view_width > scene_width)
  {
    m_view_center.setX(m_scene_center.x());
    m_view_top_left.setX(m_view_center.x() - (view_width * 0.5));
    m_view_bottom_right.setX(m_view_center.x() + (view_width * 0.5));
  }
  else
  {
    if (m_view_top_left.x() < m_scene_top_left.x())
    {
      m_view_top_left.setX(m_scene_top_left.x());
      m_view_bottom_right.setX(m_view_top_left.x() + view_width);
      m_view_center.setX(m_view_top_left.x() + (view_width * 0.5));
    }

    if (m_view_bottom_right.x() > m_scene_bottom_right.x())
    {
      m_view_bottom_right.setX(m_scene_bottom_right.x());
      m_view_top_left.setX(m_view_bottom_right.x() - view_width);
      m_view_center.setX(m_view_top_left.x() + (view_width * 0.5));
    }
  }

  if (view_height < scene_height)
  {
    m_view_center.setY(m_scene_center.y());
    m_view_top_left.setY(m_scene_center.y() - (view_height * 0.5));
    m_view_bottom_right.setY(m_scene_center.y() + (view_height * 0.5));
  }
  else
  {
    if (m_view_top_left.y() > m_scene_top_left.y())
    {
      m_view_top_left.setY(m_scene_top_left.y());
      m_view_bottom_right.setY(m_view_top_left.y() + (view_height));
      m_view_center.setY(m_view_top_left.y() + (view_height * 0.5));
    }

    if (m_view_bottom_right.y() < m_scene_bottom_right.y())
    {
      m_view_bottom_right.setY(m_scene_bottom_right.y());
      m_view_top_left.setY(m_view_bottom_right.y() - (view_height));
      m_view_center.setY(m_view_top_left.y() + (view_height * 0.5));
    }
  }
}

}

