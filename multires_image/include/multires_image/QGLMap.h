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
    QGLMap(QWidget *parent = 0);
    ~QGLMap();

    void Exit();
    void UpdateView();
    void SetTiles(TileSet* tiles);

    tf::Point SceneCenter() { return m_scene_center; }
    tf::Point ViewCenter() { return m_view_center; }

  signals:
    void SignalZoomChange(double z);
    void SignalViewChange(double x1, double y1, double x2, double y2);
    void SignalMemorySize(long);

  public slots:
    void LoadTexture(Tile* tile);
    void DeleteTexture(Tile* tile);
    void ChangeCenter(double x, double y);
    void SetTextureMemory(long);

  protected:
    void initializeGL();
    void resizeGL( int w, int h );
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
