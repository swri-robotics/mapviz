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

#ifndef MAPVIZ_PLUGINS_MULTIRES_VIEW_H_
#define MAPVIZ_PLUGINS_MULTIRES_VIEW_H_

// QT libraries
#include <QGLWidget>

#include <multires_image/tile_set.h>
#include <multires_image/tile_cache.h>

namespace mapviz_plugins
{
class MultiresView
{
  public:
    MultiresView(multires_image::TileSet* tiles, QGLWidget* widget);
    ~MultiresView(void);

    const multires_image::TileCache* Cache() { return &m_cache; }

    void SetView(double x, double y, double radius, double scale);

    void Draw();

    void Exit() { m_cache.Exit(); }

  private:
    multires_image::TileSet*   m_tiles;
    multires_image::TileCache  m_cache;
    int        m_currentLayer;
    int        m_startRow;
    int        m_startColumn;
    int        m_endRow;
    int        m_endColumn;

    double min_scale_;
  };
}

#endif  // MAPVIZ_PLUGINS_MULTIRES_VIEW_H_
