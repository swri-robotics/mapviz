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
//     * Neither the name of the <organization> nor the
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

#ifndef MAPVIZ_MAP_CANVAS_H_
#define MAPVIZ_MAP_CANVAS_H_

// C++ standard libraries
#include <cstring>
#include <list>
#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <GL/glew.h>
#include <GL/gl.h>

// QT libraries
#include <QGLWidget>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QColor>

// ROS libraries
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <mapviz/mapviz_plugin.h>

namespace mapviz
{
  class MapCanvas : public QGLWidget
  {
    Q_OBJECT

  public:
    MapCanvas(QWidget *parent = 0);
    ~MapCanvas();

    void InitializeTf(boost::shared_ptr<tf::TransformListener> tf);

    void AddPlugin(MapvizPluginPtr plugin, int order);
    void RemovePlugin(MapvizPluginPtr plugin);
    void SetFixedFrame(const std::string& frame);
    void SetTargetFrame(const std::string& frame);
    void ToggleFixOrientation(bool on);
    void ToggleUseLatestTransforms(bool on);
    void UpdateView();
    void ReorderDisplays();

    float ViewScale() const { return view_scale_; }
    float OffsetX() const { return offset_x_; }
    float OffsetY() const { return offset_y_; }

    void SetViewScale(float scale)
    {
      view_scale_ = scale;
      UpdateView();
    }

    void SetOffsetX(float x)
    {
      offset_x_ = x;
      UpdateView();
    }

    void SetOffsetY(float y)
    {
      offset_y_ = y;
      UpdateView();
    }

    void SetBackground(const QColor& color)
    {
      bg_color_ = color;
      update();
    }
    
    void CaptureFrames(bool enabled)
    {
      capture_frames_ = enabled;
      update();
    }
    
    bool CopyCaptureBuffer(std::vector<uint8_t>& buffer)
    {
      buffer.clear();
      if (!capture_buffer_.empty())
      {
        buffer.resize(capture_buffer_.size());
        memcpy(&buffer[0], &capture_buffer_[0], buffer.size());
        
        return true;
      }
      
      return false;
    }
    
    void CaptureFrame(bool force = false);
    
  Q_SIGNALS:
    void Hover(double x, double y, double scale);

  protected:
    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();
    void wheelEvent(QWheelEvent* e);
    void mousePressEvent(QMouseEvent* e);
    void mouseReleaseEvent(QMouseEvent* e);
    void mouseMoveEvent(QMouseEvent* e);
    void leaveEvent(QEvent* e);

    void Recenter();
    void TransformTarget();

    void InitializePixelBuffers();

    bool has_pixel_buffers_;
    int32_t pixel_buffer_size_;
    GLuint pixel_buffer_ids_[2];
    int32_t pixel_buffer_index_;
    bool capture_frames_;

    bool initialized_;
    bool fix_orientation_;

    QColor bg_color_;

    bool mouse_pressed_;
    int mouse_x_;
    int mouse_y_;

    // Offset based on previous mouse drags
    double offset_x_;
    double offset_y_;

    // Offset based on current mouse drag
    double drag_x_;
    double drag_y_;

    // The center of the view
    float view_center_x_;
    float view_center_y_;

    // View scale in meters per pixel
    float view_scale_;

    // The bounds of the view
    float view_left_;
    float view_right_;
    float view_top_;
    float view_bottom_;

    // The bounds of the scene
    float scene_left_;
    float scene_right_;
    float scene_top_;
    float scene_bottom_;

    std::string fixed_frame_;
    std::string target_frame_;

    boost::shared_ptr<tf::TransformListener> tf_;
    std::list<MapvizPluginPtr> plugins_;
    
    std::vector<uint8_t> capture_buffer_;
  };
}

#endif  // MAPVIZ_MAP_CANVAS_H_
