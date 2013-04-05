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

#include <multires_image/QGLMap.h>

// C++ standard libraries
#include <cmath>

namespace multires_image
{

QGLMap::QGLMap(QWidget *parent) :
  QGLWidget(parent),
  m_viewCenter(0,0),
  m_initialized(false),
  m_scale(1.0),
  m_mouseDown(false),
  m_mouseDownX(0),
  m_mouseDownY(0),
  m_tileView(NULL)
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
      m_tileView->SetView(m_viewBox.Center.X, m_viewBox.Center.Y, 1, m_scale);
    }

    glViewport(0, 0, width(), height());
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(m_viewBox.topLeft.X, m_viewBox.bottomRight.X,
      m_viewBox.bottomRight.Y, m_viewBox.topLeft.Y, -0.5f, 0.5f);

    update();

    // Signal a view change as occured.  The minimap listens for this
    // so that it can update its view box.
    emit SignalViewChange(m_viewBox.topLeft.X, m_viewBox.topLeft.Y,
      m_viewBox.bottomRight.X, m_viewBox.bottomRight.Y);
  }
}

void QGLMap::SetTiles(TileSet* tiles)
{
  double top, left, bottom, right;
  tiles->GeoReference().GetCoordinate(0, 0, left, top);
  tiles->GeoReference().GetCoordinate(tiles->GeoReference().Width(),tiles->GeoReference().Height(), right, bottom);

  m_sceneBox.topLeft = PointT<double>(left, top);
  m_sceneBox.bottomRight = PointT<double>(right, bottom);
  m_sceneBox.Update();

  m_viewCenter = m_sceneBox.Center;

  m_tileView = new TileView(tiles, this);

  connect(m_tileView->Cache(), SIGNAL(SignalMemorySize(long)),
    SLOT(SetTextureMemory(long)));

  // Create connections for the texture loading functions which must
  // be executed on this object's thread.

  m_tileView->SetView(m_viewBox.Center.X, m_viewBox.Center.Y, 1, m_scale);
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

void QGLMap::SetTextureMemory(long bytes)
{
  // Signal that the texture memory size has changed.  The status bar listens
  // to this so that the user can see how much memory the map is using.
  emit SignalMemorySize(bytes);
}

void QGLMap::ChangeCenter(double x, double y)
{
  if (x != 0)
    m_viewCenter.X = x;

  if (y != 0)
    m_viewCenter.Y = y;

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
            m_viewCenter.X += diffX;
            m_mouseDownX = x;
            changed = true;
        }
        if (diffY != 0)
        {
            m_viewCenter.Y += diffY;
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
  m_viewBox.topLeft.X = m_viewCenter.X - (width() * m_scale * 0.5);
  m_viewBox.topLeft.Y = m_viewCenter.Y - (height() * m_scale * 0.5);

  m_viewBox.bottomRight.X = m_viewCenter.X + (width() * m_scale * 0.5);
    m_viewBox.bottomRight.Y = m_viewCenter.Y + (height() * m_scale * 0.5);

  m_viewBox.Update();

  if (m_viewBox.Width > m_sceneBox.Width)
    {
    m_viewCenter.X = m_sceneBox.Center.X;
        m_viewBox.topLeft.X = m_viewCenter.X - (width() * m_scale * 0.5);
        m_viewBox.bottomRight.X = m_viewCenter.X + (width() * m_scale * 0.5);
    }
    else
    {
    if (m_viewBox.topLeft.X < m_sceneBox.topLeft.X)
        {
            m_viewBox.topLeft.X = m_sceneBox.topLeft.X;
            m_viewBox.bottomRight.X = m_viewBox.topLeft.X + (width() * m_scale);
            m_viewCenter.X = m_viewBox.topLeft.X + (width() * m_scale*0.5);
        }

    if (m_viewBox.bottomRight.X > m_sceneBox.bottomRight.X)
        {
            m_viewBox.bottomRight.X = m_sceneBox.bottomRight.X;
            m_viewBox.topLeft.X = m_viewBox.bottomRight.X - (width() * m_scale);
            m_viewCenter.X = m_viewBox.topLeft.X + (width() * m_scale * 0.5);
        }
    }

  if (m_viewBox.Height < m_sceneBox.Height)
    {
    m_viewCenter.Y = m_sceneBox.Center.Y;
        m_viewBox.topLeft.Y = m_viewCenter.Y - (height() * m_scale * 0.5);
        m_viewBox.bottomRight.Y = m_viewCenter.Y + (height() * m_scale * 0.5);
    }
    else
    {
        if (m_viewBox.topLeft.Y > m_sceneBox.topLeft.Y)
        {
            m_viewBox.topLeft.Y = m_sceneBox.topLeft.Y;
            m_viewBox.bottomRight.Y = m_viewBox.topLeft.Y + (height() * m_scale);
            m_viewCenter.Y = m_viewBox.topLeft.Y + (height() * m_scale * 0.5);
        }

        if (m_viewBox.bottomRight.Y < m_sceneBox.bottomRight.Y)
        {
            m_viewBox.bottomRight.Y = m_sceneBox.bottomRight.Y;
            m_viewBox.topLeft.Y = m_viewBox.bottomRight.Y - (height() * m_scale);
            m_viewCenter.Y = m_viewBox.topLeft.Y + (height() * m_scale * 0.5);
        }
    }

  m_viewBox.Update();
}

}

