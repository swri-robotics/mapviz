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

#include <tile_map/tile_source.h>

namespace tile_map
{
  const QString& TileSource::GetBaseUrl() const
  {
    return base_url_;
  }

  void TileSource::SetBaseUrl(const QString& base_url)
  {
    base_url_ = base_url;
  }

  bool TileSource::IsCustom() const
  {
    return is_custom_;
  }

  void TileSource::SetCustom(bool is_custom)
  {
    is_custom_ = is_custom;
  }

  int32_t TileSource::GetMaxZoom() const
  {
    return max_zoom_;
  }

  void TileSource::SetMaxZoom(int32_t max_zoom)
  {
    max_zoom_ = max_zoom;
  }

  int32_t TileSource::GetMinZoom() const
  {
    return min_zoom_;
  }

  void TileSource::SetMinZoom(int32_t min_zoom)
  {
    min_zoom_ = min_zoom;
  }

  const QString& TileSource::GetName() const
  {
    return name_;
  }

  void TileSource::SetName(const QString& name)
  {
    name_ = name;
  }
}
