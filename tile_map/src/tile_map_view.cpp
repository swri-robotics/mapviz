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

#include <tile_map/tile_map_view.h>

#include <cmath>

#include <boost/lexical_cast.hpp>
#include <boost/make_shared.hpp>

#include <GL/gl.h>

#include <ros/ros.h>

#include <math_util/constants.h>
#include <math_util/trig_util.h>
#include <transform_util/earth_constants.h>

#include <tile_map/image_cache.h>

namespace tile_map
{
  TileMapView::TileMapView() :
    base_url_("localhost"),
    extension_(".jpg"),
    max_level_(19),
    level_(-1),
    width_(100),
    height_(100)
  {
    ImageCachePtr image_cache = boost::make_shared<ImageCache>("/tmp/tile_map");
    tile_cache_ = boost::make_shared<TextureCache>(image_cache);
  }
  
  void TileMapView::SetMaxLevel(int32_t level)
  {
    max_level_ = level;
  }
  
  void TileMapView::SetExtension(const std::string& extension)
  {
    extension_ = extension;
  }
  
  void TileMapView::SetBaseUrl(const std::string& url)
  {
    if (base_url_ != url)
    {
      base_url_ = url;
      level_ = -1;
    }
  }
  
  void TileMapView::SetTransform(const transform_util::Transform& transform)
  {
    transform_ = transform;
    
    // TODO: check if modified.
    
    for (size_t i = 0; i < tiles_.size(); i++)
    {
      tiles_[i].top_left_t = transform_ * tiles_[i].top_left;
      tiles_[i].top_right_t = transform_ * tiles_[i].top_right;
      tiles_[i].bottom_right_t = transform_ * tiles_[i].bottom_right;
      tiles_[i].bottom_left_t = transform_ * tiles_[i].bottom_left;
    }
    
    for (size_t i = 0; i < precache_above_.size(); i++)
    {
      precache_above_[i].top_left_t = transform_ * precache_above_[i].top_left;
      precache_above_[i].top_right_t = transform_ * precache_above_[i].top_right;
      precache_above_[i].bottom_right_t = transform_ * precache_above_[i].bottom_right;
      precache_above_[i].bottom_left_t = transform_ * precache_above_[i].bottom_left;
    }
  }
  
  void TileMapView::SetView(
    double latitude, 
    double longitude, 
    double scale, 
    int32_t width,
    int32_t height)
  {
    latitude = std::max(-90.0, std::min(90.0, latitude));
    longitude = std::max(-180.0, std::min(180.0, longitude));
  
    double lat = math_util::ToRadians(latitude);
  
    // Calculate the current zoom level:
    // 
    // According to http://wiki.openstreetmap.org/wiki/Zoom_levels:
    //   meters_per_pixel = earth_circumference * cos(lat) / 2^(level + 8)
    //
    // Therefore,
    //   level = log2(earth_circumference * cos(lat) / meters_per_pixel) - 8
    //
    double lat_circumference = 
      transform_util::_earth_equator_circumference * std::cos(lat) / scale;
    int32_t level = std::min((double)max_level_, std::max(0.0, std::ceil(std::log(lat_circumference) / std::log(2) - 8)));
    
    int64_t max_size = std::pow(2, level);
    
    int64_t center_x = std::min((double)max_size - 1.0, std::floor(((longitude + 180.0) / 360.0) * std::pow(2.0, level)));
    int64_t center_y = std::min((long double)max_size - 1.0, std::floor((1.0 - std::log(std::tan(lat) + 1.0 / std::cos(lat)) / math_util::_pi) / 2.0 * std::pow(2.0, level))); 
        
    width_ = width;
    height_ = height;
    
    double max_dimension = std::max(width, height);
    
    double meters_per_pixel = transform_util::_earth_equator_circumference * std::cos(lat) / std::pow(2, level + 8);    
    double tile_size = 256.0 * (meters_per_pixel / scale);
    
    int64_t size = std::max(1.0, std::min((double)max_size, std::ceil(0.5 * max_dimension / tile_size) * 2 + 1));
    
    if (size > 50)
    {
      ROS_ERROR("Invalid map size: %ld", size);
      return;
    }
    
    if (size_ != size_ || level_ != level || center_x_ != center_x || center_y_ != center_y)
    {
      size_ = size;
      level_ = level;
      center_x_ = center_x;
      center_y_ = center_y;
      
      int64_t top = std::max(0L, center_y_ - size_ / 2);
      int64_t left = std::max(0L, center_x_ - size_ / 2);
      
      int64_t right = std::min(max_size, left + size_);
      int64_t bottom = std::min(max_size, top + size_);

      for (size_t i = 0; i < tiles_.size(); i++)
      {
        tile_cache_->AddTexture(tiles_[i].texture);
      }
      tiles_.clear();
      
      for (int64_t i = top; i < bottom; i++)
      {
        for (int64_t j = left; j < right; j++)
        {
          Tile tile;
          InitializeTile(level_, j, i, tile);
          tiles_.push_back(tile);
        }
      }
      
      for (size_t i = 0; i < precache_above_.size(); i++)
      {
        tile_cache_->AddTexture(precache_above_[i].texture);
      }
      precache_above_.clear();

      if (level_ > 0)
      {
        int64_t above_x = std::floor(((longitude + 180.0) / 360.0) * std::pow(2.0, level - 1));
        int64_t above_y = std::floor((1.0 - std::log(std::tan(lat) + 1.0 / std::cos(lat)) / math_util::_pi) / 2.0 * std::pow(2.0, level - 1)); 
        
        int64_t above_max_size = std::pow(2, level - 1);
        
        int64_t above_top = std::max(0L, above_y - (size_ - 1) / 2);
        int64_t above_left = std::max(0L, above_x - (size_ - 1) / 2);
      
        int64_t above_right = std::min(above_max_size, above_left + size_);
        int64_t above_bottom = std::min(above_max_size, above_top + size_);
        
        for (int64_t i = above_top; i < above_bottom; i++)
        {
          for (int64_t j = above_left; j < above_right; j++)
          {
            Tile tile;
            InitializeTile(level_ - 1, j, i, tile);
            precache_above_.push_back(tile);
          }
        }
      }
    }
  }
  
  void TileMapView::Draw()
  {
    glEnable(GL_TEXTURE_2D);
    
    for (size_t i = 0; i < precache_above_.size(); i++)
    {
      TexturePtr texture = precache_above_[i].texture;
      
      if (!texture)
      {
        bool failed;
        texture = tile_cache_->GetTexture(precache_above_[i].url_hash, precache_above_[i].url, failed);
        if (failed)
        {
          max_level_ = std::min(max_level_, precache_above_[i].level - 1);
          ROS_WARN("===== SETTING MAX LEVEL TO %d =====", max_level_);
        }
      }
    
      if (texture)
      {
        glBindTexture(GL_TEXTURE_2D, texture->id);

        glBegin(GL_TRIANGLES);
                
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

        glTexCoord2f(0, 1); glVertex2d(precache_above_[i].top_left_t.x(), precache_above_[i].top_left_t.y());
        glTexCoord2f(1, 1); glVertex2d(precache_above_[i].top_right_t.x(), precache_above_[i].top_right_t.y());
        glTexCoord2f(1, 0); glVertex2d(precache_above_[i].bottom_right_t.x(), precache_above_[i].bottom_right_t.y());
          
        glTexCoord2f(0, 1); glVertex2d(precache_above_[i].top_left_t.x(), precache_above_[i].top_left_t.y());
        glTexCoord2f(1, 0); glVertex2d(precache_above_[i].bottom_right_t.x(), precache_above_[i].bottom_right_t.y());
        glTexCoord2f(0, 0); glVertex2d(precache_above_[i].bottom_left_t.x(), precache_above_[i].bottom_left_t.y());
      
        glEnd();
      
        glBindTexture(GL_TEXTURE_2D, 0);
      }
    }
    
    for (size_t i = 0; i < tiles_.size(); i++)
    {
      TexturePtr texture = tiles_[i].texture;
      
      if (!texture)
      {
        bool failed;
        texture = tile_cache_->GetTexture(tiles_[i].url_hash, tiles_[i].url, failed);
        if (failed)
        {
          max_level_ = std::min(max_level_, tiles_[i].level - 1);
          ROS_WARN("===== SETTING MAX LEVEL TO %d =====", max_level_);
        }
      }
    
      if (texture)
      {
        glBindTexture(GL_TEXTURE_2D, texture->id);

        glBegin(GL_TRIANGLES);
                
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

        glTexCoord2f(0, 1); glVertex2d(tiles_[i].top_left_t.x(), tiles_[i].top_left_t.y());
        glTexCoord2f(1, 1); glVertex2d(tiles_[i].top_right_t.x(), tiles_[i].top_right_t.y());
        glTexCoord2f(1, 0); glVertex2d(tiles_[i].bottom_right_t.x(), tiles_[i].bottom_right_t.y());
          
        glTexCoord2f(0, 1); glVertex2d(tiles_[i].top_left_t.x(), tiles_[i].top_left_t.y());
        glTexCoord2f(1, 0); glVertex2d(tiles_[i].bottom_right_t.x(), tiles_[i].bottom_right_t.y());
        glTexCoord2f(0, 0); glVertex2d(tiles_[i].bottom_left_t.x(), tiles_[i].bottom_left_t.y());
      
        glEnd();
      
        glBindTexture(GL_TEXTURE_2D, 0);
      }
    }
      
    glDisable(GL_TEXTURE_2D);
  }
  
  void TileMapView::ToLatLon(int32_t level, int32_t x, int32_t y, double& latitude, double& longitude)
  {
    double n = std::pow(2, level);
    longitude = (double)x / n * 360.0 - 180.0;
    
    double r = math_util::_pi - math_util::_2pi * (double)y / n;
    latitude = math_util::_rad_2_deg * std::atan(0.5 * (std::exp(r) - std::exp(-r)));
  }
  
  void TileMapView::InitializeTile(int32_t level, int64_t x, int64_t y, Tile& tile)
  {
    tile.url = base_url_ + 
      boost::lexical_cast<std::string>(level) + "/" +
      boost::lexical_cast<std::string>(x) + "/" +
      boost::lexical_cast<std::string>(y) + extension_;
  
  
    tile.url_hash = hash_function_(tile.url);
  
    tile.level = level;
  
    bool failed;
    tile.texture = tile_cache_->GetTexture(tile.url_hash, tile.url, failed);
    if (failed)
    {
      max_level_ = std::min(max_level_, level - 1);
      ROS_WARN("===== SETTING MAX LEVEL TO %d =====", max_level_);
    }
  
    double t_lat, t_lon;
    ToLatLon(level, x, y, t_lat, t_lon);
    tile.top_left = tf::Vector3(t_lon, t_lat, 0);
    tile.top_left_t = transform_ * tile.top_left;
      
    ToLatLon(level, x + 1, y, t_lat, t_lon);
    tile.top_right = tf::Vector3(t_lon, t_lat, 0);
    tile.top_right_t = transform_ * tile.top_right;
      
    ToLatLon(level, x + 1, y + 1, t_lat, t_lon);
    tile.bottom_right = tf::Vector3(t_lon, t_lat, 0);
    tile.bottom_right_t = transform_ * tile.bottom_right;
      
    ToLatLon(level, x, y + 1, t_lat, t_lon);
    tile.bottom_left = tf::Vector3(t_lon, t_lat, 0);
    tile.bottom_left_t = transform_ * tile.bottom_left;
  }
}
