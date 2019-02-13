// *****************************************************************************
//
// Copyright (c) 2015, Southwest Research Institute® (SwRI®)
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
// ARE DISCLAIMED. IN NO EVENT SHALL Southwest Research Institute® BE LIABLE 
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
// OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.
//
// *****************************************************************************

#include <tile_map/wmts_source.h>

#include <boost/functional/hash.hpp>

namespace tile_map
{
  const QString WmtsSource::WMTS_TYPE = "wmts";

  WmtsSource::WmtsSource(const QString& name,
                         const QString& base_url,
                         bool is_custom,
                         int32_t max_zoom) :
                         TileSource()
  {
    name_ = name;
    base_url_ = base_url;
    is_custom_ = is_custom;
    max_zoom_ = max_zoom;
    min_zoom_ = 1;
  }

  QString WmtsSource::GetType() const
  {
    return WMTS_TYPE;
  }

  size_t WmtsSource::GenerateTileHash(int32_t level, int64_t x, int64_t y)
  {
    return hash_(GenerateTileUrl(level, x, y).toStdString());
  }

  QString WmtsSource::GenerateTileUrl(int32_t level, int64_t x, int64_t y)
  {
    QString url(base_url_);
    url.replace(QString::fromStdString("{level}"), QString::number(level));
    url.replace(QString::fromStdString("{x}"), QString::number(x));
    url.replace(QString::fromStdString("{y}"), QString::number(y));

    return url;
  }
}
