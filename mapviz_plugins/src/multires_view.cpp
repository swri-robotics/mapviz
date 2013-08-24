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

#include <mapviz_plugins/multires_view.h>

// C++ standard libraries
#include <cmath>
#include <iostream>

#include <ros/ros.h>

namespace mapviz_plugins
{
  MultiresView::MultiresView(multires_image::TileSet* tiles, QGLWidget* widget) :
      m_tiles(tiles),
      m_cache(tiles, widget),
      m_currentLayer(tiles->LayerCount() - 1),
      m_startRow(0),
      m_startColumn(0),
      m_endRow(0),
      m_endColumn(0)
  {
    double top, left, bottom, right;
    tiles->GeoReference().GetCoordinate(0, 0, left, top);
    tiles->GeoReference().GetCoordinate(tiles->GeoReference().Width(),tiles->GeoReference().Height(), right, bottom);

    double scale_x = std::fabs(right - left) / (double)tiles->GeoReference().Width();
    double scale_y = std::fabs(top - bottom) / (double)tiles->GeoReference().Height();

    min_scale_ = scale_x;
    if (scale_y > scale_x)
      min_scale_ = scale_y;

    ROS_INFO("min_scale: %lf", min_scale_);
  }

  MultiresView::~MultiresView(void)
  {
  }

  void MultiresView::SetView(double x, double y, double radius, double scale)
  {
    int layer = 0;
    while (min_scale_ * std::pow(2.0, layer + 1) < scale) layer++;

    if (layer >= m_tiles->LayerCount())
      layer = m_tiles->LayerCount() - 1;

    if (layer != m_currentLayer)
    {
      m_currentLayer = layer;
      m_cache.SetCurrentLayer(layer);
    }

    int row, column;
    m_tiles->GetLayer(m_currentLayer)->GetTileIndex(x, y, row, column);

    int size = 3;

    m_startRow = row - size;
    if (m_startRow < 0)
      m_startRow = 0;
    if (m_startRow >= m_tiles->GetLayer(m_currentLayer)->RowCount())
      m_startRow = m_tiles->GetLayer(m_currentLayer)->RowCount() - 1;

    m_endRow = row + size;
    if (m_endRow < 0)
      m_endRow = 0;
    if (m_endRow >= m_tiles->GetLayer(m_currentLayer)->RowCount())
      m_endRow = m_tiles->GetLayer(m_currentLayer)->RowCount() - 1;

    m_startColumn = column - size;
    if (m_startColumn < 0)
      m_startColumn = 0;
    if (m_startColumn >= m_tiles->GetLayer(m_currentLayer)->ColumnCount())
      m_startColumn = m_tiles->GetLayer(m_currentLayer)->ColumnCount() - 1;

    m_endColumn = column + size;
    if (m_endColumn < 0)
      m_endColumn = 0;
    if (m_endColumn >= m_tiles->GetLayer(m_currentLayer)->ColumnCount())
      m_endColumn = m_tiles->GetLayer(m_currentLayer)->ColumnCount() - 1;

    m_cache.Precache(x, y);
  }

  void MultiresView::Draw()
  {
    glEnable(GL_TEXTURE_2D);

    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

    // Always draw bottom layers

    multires_image::TileSetLayer* baseLayer = m_tiles->GetLayer(m_tiles->LayerCount() - 1);
      multires_image::Tile* tile = baseLayer->GetTile(0,0);
      if (tile->TextureLoaded())
      {
        tile->Draw();
      }
      else
      {
        m_cache.Load(tile);
      }

      baseLayer = m_tiles->GetLayer(m_tiles->LayerCount() - 2);
      for (int c = 0; c < baseLayer->ColumnCount(); c++)
      {
        for (int r = 0; r <  baseLayer->RowCount(); r++)
        {
          multires_image::Tile* tile = baseLayer->GetTile(c, r);
          if (tile->TextureLoaded())
          {
            tile->Draw();
          }
          else
          {
            m_cache.Load(tile);
          }
        }
      }

    if (m_currentLayer < m_tiles->LayerCount() - 2)
    {
    multires_image::TileSetLayer* layer = m_tiles->GetLayer(m_currentLayer);
    if (m_endColumn < layer->ColumnCount() && m_endRow < layer->RowCount())
    {
        for (int c = m_startColumn; c <= m_endColumn; c++)
        {
          for (int r = m_startRow; r <= m_endRow; r++)
          {

            multires_image::Tile* tile = layer->GetTile(c, r);
            if (tile->TextureLoaded())
            {
              tile->Draw();
            }
            else
            {
              m_cache.Load(tile);
            }
          }
        }
      }
    }

    glDisable(GL_TEXTURE_2D);
  }
}
