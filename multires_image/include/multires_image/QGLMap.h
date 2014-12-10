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

#ifndef MULTIRES_IMAGE_QGLMAP_H_
#define MULTIRES_IMAGE_QGLMAP_H_

// QT libraries
#include <QGLWidget>
#include <QMouseEvent>
#include <QWheelEvent>

// QT auto-generated headers
#include "ui_QGLMap.h"

#include <tf/transform_datatypes.h>

#include <multires_image/tile.h>
#include <multires_image/tile_view.h>

namespace multires_image
{
  class QGLMap : public QGLWidget
  {
    Q_OBJECT

  public:
    explicit QGLMap(QWidget *parent = 0);
    ~QGLMap();

    void Exit();
    void UpdateView();
    void SetTiles(TileSet* tiles);

    tf::Point SceneCenter() { return m_scene_center; }
    tf::Point ViewCenter() { return m_view_center; }

  signals:
    void SignalZoomChange(double z);
    void SignalViewChange(double x1, double y1, double x2, double y2);
    void SignalMemorySize(int64_t bytes);

  public slots:
    void LoadTexture(Tile* tile);
    void DeleteTexture(Tile* tile);
    void ChangeCenter(double x, double y);
    void SetTextureMemory(int64_t bytes);

  protected:
    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();
    void mousePressEvent(QMouseEvent* e);
    void mouseDoubleClickEvent(QMouseEvent* e);
    void mouseReleaseEvent(QMouseEvent* e);
    void mouseMoveEvent(QMouseEvent* e);
    void wheelEvent(QWheelEvent* e);

  private:
    Ui::QGLMapClass ui;

    bool            m_initialized;

    double          m_scale;

    bool            m_mouseDown;
    int             m_mouseDownX;
    int             m_mouseDownY;

    TileView*       m_tileView;

    tf::Point m_view_top_left;
    tf::Point m_view_bottom_right;
    tf::Point m_view_center;

    tf::Point m_scene_top_left;
    tf::Point m_scene_bottom_right;
    tf::Point m_scene_center;

    void Recenter();
    void MousePan(int x, int y);
  };
}

#endif  // MULTIRES_IMAGE_QGLMAP_H_
