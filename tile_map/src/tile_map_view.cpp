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
    level_(-1),
    width_(100),
    height_(100)
  {
    ImageCachePtr image_cache = boost::make_shared<ImageCache>("/tmp/");
    tile_cache_ = boost::make_shared<TextureCache>(image_cache);
  }
  
  void TileMapView::SetBaseUrl(const std::string& url)
  {
    if (base_url_ != url)
    {
      base_url_ = url;
      level_ = -1;
    }
  }
  
  void TileMapView::SetView(
    double latitude, 
    double longitude, 
    double scale, 
    int32_t width,
    int32_t height)
  {
    ROS_ERROR("SetView(%lf, %lf, %lf, %d, %d)", latitude, longitude, scale, width, height);
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
    int32_t level = std::ceil(std::log(lat_circumference) / std::log(2) - 8);
    
    int64_t center_x = std::floor(((longitude + 180.0) / 360.0) * std::pow(2.0, level));
    int64_t center_y = std::floor((1.0 - std::log(std::tan(lat) + 1.0 / std::cos(lat)) / math_util::_pi) / 2.0 * std::pow(2.0, level)); 
    
    tmp_url_ = base_url_ + 
      boost::lexical_cast<std::string>(level) + "/" +
      boost::lexical_cast<std::string>(center_x) + "/" +
      boost::lexical_cast<std::string>(center_y) + ".png";
      
    ROS_WARN("%s",tmp_url_.c_str()); 
    
    width_ = width;
    height_ = height;
  }
  
  void TileMapView::Draw()
  {
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, width_, height_, 0, -0.5f, 0.5f);
    
    glEnable(GL_TEXTURE_2D);
    
    glBegin(GL_TRIANGLES);
            
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    glVertex2d(width_ * 0.25, height_ * 0.25);
    glVertex2d(width_ * 0.75, height_ * 0.25);
    glVertex2d(width_ * 0.75, height_ * 0.75);
    
    glVertex2d(width_ * 0.25, height_ * 0.25);
    glVertex2d(width_ * 0.75, height_ * 0.75);
    glVertex2d(width_ * 0.25, height_ * 0.75);
    
    glEnd();
    
    glDisable(GL_TEXTURE_2D);
    
    glPopMatrix();
  }
}
