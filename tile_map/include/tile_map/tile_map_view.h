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
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#ifndef TILE_MAP_TILE_MAP_VIEW_H_
#define TILE_MAP_TILE_MAP_VIEW_H_

#include <string>

#include <boost/functional/hash.hpp>

#include <tile_map/texture_cache.h>

#include <transform_util/transform.h>

namespace tile_map
{
  struct Tile
  {
  public:
    std::string url;
    size_t url_hash;
    int32_t level;
    
    TexturePtr texture;
    
    tf::Vector3 top_left;
    tf::Vector3 top_right;
    tf::Vector3 bottom_left;
    tf::Vector3 bottom_right;
    
    tf::Vector3 top_left_t;
    tf::Vector3 top_right_t;
    tf::Vector3 bottom_left_t;
    tf::Vector3 bottom_right_t;
  };

  class TileMapView
  {
  public:
    TileMapView();
    
    void SetBaseUrl(const std::string& url);
    
    void SetMaxLevel(int32_t level);
    
    void SetExtension(const std::string& extension);
    
    void SetTransform(const transform_util::Transform& transform);
    
    void SetView(
      double latitude, 
      double longitude, 
      double scale, 
      int32_t width,
      int32_t height);
      
    void Draw();
    
  private:
    std::string base_url_;
    
    std::string extension_;
    
    transform_util::Transform transform_;
    
    int32_t max_level_;
    
    int32_t level_;
    
    int64_t center_x_;
    int64_t center_y_;
    
    int64_t size_;
    
    int32_t width_;
    int32_t height_;
    
    std::vector<Tile> tiles_;
    std::vector<Tile> precache_below_;
    std::vector<Tile> precache_above_;
    
    boost::hash<std::string> hash_function_;
    TextureCachePtr tile_cache_;
    
    void ToLatLon(int32_t level, int32_t x, int32_t y, double& latitude, double& longitude);
    
    void InitializeTile(int32_t level, int64_t x, int64_t y, Tile& tile);
  };
}

#endif  // TILE_MAP_TILE_MAP_VIEW_H_
