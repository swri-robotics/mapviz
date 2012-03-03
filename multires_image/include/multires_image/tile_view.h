/*
* ImageManip.h
*
* Copyright (C) 2010 Southwest Research Institute <malban@swri.org>
*
* Created on: Mar 01, 2010
*     Author: Marc Alban
*/

#ifndef MULTIRES_IMAGE_TILE_VIEW_H_
#define MULTIRES_IMAGE_TILE_VIEW_H_

// QT libraries
#include <QGLWidget>

#include <multires_image/tile_set.h>
#include <multires_image/tile_cache.h>
#include <multires_image/point.h>
#include <multires_image/bounding_box.h>

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
