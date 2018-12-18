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

#include <boost/make_shared.hpp>

#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <ros/ros.h>

#include <swri_math_util/constants.h>
#include <swri_math_util/trig_util.h>
#include <swri_transform_util/earth_constants.h>

#include <tile_map/image_cache.h>

namespace tile_map
{
  TileMapView::TileMapView() :
    level_(-1),
    width_(100),
    height_(100)
  {
    ImageCachePtr image_cache = boost::make_shared<ImageCache>("/tmp/tile_map");
    tile_cache_ = boost::make_shared<TextureCache>(image_cache);
  }

  bool TileMapView::IsReady()
  {
    return tile_source_ && tile_source_->IsReady();
  }

  void TileMapView::ResetCache()
  {
    tile_cache_->Clear();
  }

  void TileMapView::SetTileSource(const boost::shared_ptr<TileSource>& tile_source)
  {
    tile_source_ = tile_source;
    level_ = -1;
  }

  void TileMapView::SetTransform(const swri_transform_util::Transform& transform)
  {
    if (transform.GetOrigin() == transform_.GetOrigin() &&
        transform.GetOrientation() == transform_.GetOrientation())
    {
      return;
    }

    transform_ = transform;

    for (size_t i = 0; i < tiles_.size(); i++)
    {
      for (size_t j = 0; j < tiles_[i].points_t.size(); j++)
      {
        tiles_[i].points_t[j] = transform_ * tiles_[i].points[j];
      }
    }

    for (size_t i = 0; i < precache_.size(); i++)
    {
      for (size_t j = 0; j < precache_[i].points_t.size(); j++)
      {
        precache_[i].points_t[j] = transform_ * precache_[i].points[j];
      }
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

    double lat = swri_math_util::ToRadians(latitude);

    // Calculate the current zoom level:
    //
    // According to http://wiki.openstreetmap.org/wiki/Zoom_levels:
    //   meters_per_pixel = earth_circumference * cos(lat) / 2^(level + 8)
    //
    // Therefore,
    //   level = log2(earth_circumference * cos(lat) / meters_per_pixel) - 8
    //
    double lat_circumference =
      swri_transform_util::_earth_equator_circumference * std::cos(lat) / scale;
    int32_t level =  std::min(tile_source_->GetMaxZoom(),
                             std::max(tile_source_->GetMinZoom(), static_cast<int32_t>(std::ceil(std::log(lat_circumference) / std::log(2) - 8))));
    int64_t max_size = std::pow(2, level);

    int64_t center_x = std::min(max_size - 1, static_cast<int64_t>(
      std::floor(((longitude + 180.0) / 360.0) * std::pow(2.0, level))));
    int64_t center_y = std::min(max_size - 1, static_cast<int64_t>(
      std::floor((1.0 - std::log(std::tan(lat) + 1.0 / std::cos(lat)) / swri_math_util::_pi) / 2.0 * std::pow(2.0, level))));

    width_ = width;
    height_ = height;

    double max_dimension = std::max(width, height);

    double meters_per_pixel = swri_transform_util::_earth_equator_circumference * std::cos(lat) / std::pow(2, level + 8);
    double tile_size = 256.0 * (meters_per_pixel / scale);

    int64_t size = std::max(static_cast<int64_t>(1L), std::min(max_size, static_cast<int64_t>(
      std::ceil(0.5 * max_dimension / tile_size) * 2 + 1)));

    if (size > 50)
    {
      ROS_ERROR("Invalid map size: %ld", size);
      return;
    }

    if (size_ != size || level_ != level || center_x_ != center_x || center_y_ != center_y)
    {
      size_ = size;
      level_ = level;
      center_x_ = center_x;
      center_y_ = center_y;

      int64_t top = std::max(static_cast<int64_t>(0L), center_y_ - size_ / 2);
      int64_t left = std::max(static_cast<int64_t>(0L), center_x_ - size_ / 2);

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
          InitializeTile(level_, j, i, tile, 10000);
          tiles_.push_back(tile);
        }
      }

      for (size_t i = 0; i < precache_.size(); i++)
      {
        tile_cache_->AddTexture(precache_[i].texture);
      }
      precache_.clear();

      if (level_ > 0)
      {
        int64_t precache_x = std::floor(((longitude + 180.0) / 360.0) * std::pow(2.0, level - 1));
        int64_t precache_y = std::floor((1.0 - std::log(std::tan(lat) + 1.0 / std::cos(lat)) / swri_math_util::_pi) / 2.0 * std::pow(2.0, level - 1));

        int64_t precache_max_size = std::pow(2, level - 1);

        int64_t precache_top = std::max(static_cast<int64_t>(0L), precache_y - (size_ - 1) / 2);
        int64_t precache_left = std::max(static_cast<int64_t>(0L), precache_x - (size_ - 1) / 2);

        int64_t precache_right = std::min(precache_max_size, precache_left + size_);
        int64_t precache_bottom = std::min(precache_max_size, precache_top + size_);

        for (int64_t i = precache_top; i < precache_bottom; i++)
        {
          for (int64_t j = precache_left; j < precache_right; j++)
          {
            Tile tile;
            InitializeTile(level_ - 1, j, i, tile, 0);
            precache_.push_back(tile);
          }
        }
      }
    }
  }

  void TileMapView::DrawTiles(std::vector<Tile>& tiles, int priority)
  {
    for (size_t i = 0; i < tiles.size(); i++)
    {
      TexturePtr& texture = tiles[i].texture;

      if (!texture)
      {
        bool failed;
        texture = tile_cache_->GetTexture(tiles[i].url_hash, tiles[i].url, failed, priority);
      }

      if (texture)
      {
        glBindTexture(GL_TEXTURE_2D, texture->id);

        glBegin(GL_TRIANGLES);

        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

        for (int32_t row = 0; row < tiles[i].subdiv_count; row++)
        {
          for (int32_t col = 0; col < tiles[i].subdiv_count; col++)
          {
            double u_0 = col * tiles[i].subwidth;
            double v_0 = 1.0 - row * tiles[i].subwidth;
            double u_1 = (col + 1.0) * tiles[i].subwidth;
            double v_1 = 1.0 - (row + 1.0) * tiles[i].subwidth;

            const tf::Vector3& tl = tiles[i].points_t[row * (tiles[i].subdiv_count + 1) + col];
            const tf::Vector3& tr = tiles[i].points_t[row * (tiles[i].subdiv_count + 1) + col + 1];
            const tf::Vector3& br = tiles[i].points_t[(row + 1) * (tiles[i].subdiv_count + 1) + col + 1];
            const tf::Vector3& bl = tiles[i].points_t[(row + 1) * (tiles[i].subdiv_count + 1) + col];

            // Triangle 1
            glTexCoord2f(u_0, v_0); glVertex2d(tl.x(), tl.y());
            glTexCoord2f(u_1, v_0); glVertex2d(tr.x(), tr.y());
            glTexCoord2f(u_1, v_1); glVertex2d(br.x(), br.y());

            // Triangle 2
            glTexCoord2f(u_0, v_0); glVertex2d(tl.x(), tl.y());
            glTexCoord2f(u_1, v_1); glVertex2d(br.x(), br.y());
            glTexCoord2f(u_0, v_1); glVertex2d(bl.x(), bl.y());
          }
        }

        glEnd();

        glBindTexture(GL_TEXTURE_2D, 0);
      }
    }
  }

  void TileMapView::Draw()
  {
    if (!tile_source_)
    {
      return;
    }

    glEnable(GL_TEXTURE_2D);

    DrawTiles( precache_, 0 );
    DrawTiles( tiles_, 10000 );

    glDisable(GL_TEXTURE_2D);
  }

  void TileMapView::ToLatLon(int32_t level, double x, double y, double& latitude, double& longitude)
  {
    double n = std::pow(2, level);
    longitude = x / n * 360.0 - 180.0;

    double r = swri_math_util::_pi - swri_math_util::_2pi * y / n;
    latitude = swri_math_util::_rad_2_deg * std::atan(0.5 * (std::exp(r) - std::exp(-r)));
  }

  void TileMapView::InitializeTile(int32_t level, int64_t x, int64_t y, Tile& tile, int priority)
  {
    tile.url = tile_source_->GenerateTileUrl(level, x, y);

    tile.url_hash = tile_source_->GenerateTileHash(level, x, y);

    tile.level = level;

    bool failed;
    tile.texture = tile_cache_->GetTexture(tile.url_hash, tile.url, failed, priority);

    int32_t subdivs = std::max(0, 4 - level);
    tile.subwidth = 1.0 / (subdivs + 1.0);
    tile.subdiv_count = std::pow(2, subdivs);
    for (int32_t row = 0; row <= tile.subdiv_count; row++)
    {
      for (int32_t col = 0; col <= tile.subdiv_count; col++)
      {
        double t_lat, t_lon;
        ToLatLon(level, x + col * tile.subwidth, y + row * tile.subwidth, t_lat, t_lon);
        tile.points.push_back(tf::Vector3(t_lon, t_lat, 0));
      }
    }

    tile.points_t = tile.points;
    for (size_t i = 0; i < tile.points_t.size(); i++)
    {
      tile.points_t[i] = transform_ * tile.points_t[i];
    }
  }
}
