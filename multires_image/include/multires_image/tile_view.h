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

#ifndef MULTIRES_IMAGE_TILE_VIEW_H_
#define MULTIRES_IMAGE_TILE_VIEW_H_

// QT libraries
#include <QGLWidget>

#include <multires_image/tile_set.h>
#include <multires_image/tile_cache.h>

namespace multires_image
{
  class TileView
  {
  public:
    TileView(TileSet* tiles, QGLWidget* widget);
    ~TileView(void);

    const TileCache* Cache() { return &m_cache; }

    void SetView(double x, double y, double radius, double scale);

    void Draw();

    void Exit() { m_cache.Exit(); }

  private:
    TileSet*   m_tiles;
    TileCache  m_cache;
    int        m_currentLayer;
    int        m_startRow;
    int        m_startColumn;
    int        m_endRow;
    int        m_endColumn;
    double     min_scale_;
  };
}

#endif  // MULTIRES_IMAGE_TILE_VIEW_H_
