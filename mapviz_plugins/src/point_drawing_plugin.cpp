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

#include <mapviz_plugins/point_drawing_plugin.h>

#include <vector>
#include <list>

#include <QDialog>
#include <QGLWidget>
#include <QPalette>
#include <QPainter>

#include <opencv2/core/core.hpp>

#include <swri_image_util/geometry_util.h>
#include <swri_transform_util/transform_util.h>

namespace mapviz_plugins
{
  PointDrawingPlugin::PointDrawingPlugin()
      : arrow_size_(25),
        draw_style_(LINES),
        position_tolerance_(0.0),
        buffer_size_(0),
        covariance_checked_(false),
        new_lap_(true),
        lap_checked_(false),
        buffer_holder_(false),
        scale_(1.0),
        static_arrow_sizes_(false),
        got_begin_(false)
  {
  }

  void PointDrawingPlugin::DrawIcon()
  {
    if (icon_)
    {
      QPixmap icon(16, 16);
      icon.fill(Qt::transparent);

      QPainter painter(&icon);
      painter.setRenderHint(QPainter::Antialiasing, true);

      QPen pen(color_);

      if (draw_style_ == POINTS)
      {
        pen.setWidth(7);
        pen.setCapStyle(Qt::RoundCap);
        painter.setPen(pen);
        painter.drawPoint(8, 8);
      }
      else if (draw_style_ == LINES)
      {
        pen.setWidth(3);
        pen.setCapStyle(Qt::FlatCap);
        painter.setPen(pen);
        painter.drawLine(1, 14, 14, 1);
      }
      else if (draw_style_ == ARROWS)
      {
        pen.setWidth(2);
        pen.setCapStyle(Qt::SquareCap);
        painter.setPen(pen);
        painter.drawLine(2, 13, 13, 2);
        painter.drawLine(13, 2, 13, 8);
        painter.drawLine(13, 2, 7, 2);
      }

      icon_->SetPixmap(icon);
    }
  }

  void PointDrawingPlugin::SetArrowSize(int arrowSize)
  {
    arrow_size_ = arrowSize;
  }

  void PointDrawingPlugin::SetDrawStyle(QString style)
  {
    if (style == "lines")
    {
      draw_style_ = LINES;
    }
    else if (style == "points")
    {
      draw_style_ = POINTS;
    }
    else if (style == "arrows")
    {
      draw_style_ = ARROWS;
    }

    DrawIcon();
  }

  void PointDrawingPlugin::SetStaticArrowSizes(bool isChecked)
  {
    static_arrow_sizes_ = isChecked;
  }

  bool PointDrawingPlugin::DrawPoints(double scale)
  {
    scale_ = scale;
    bool transformed = true;
    if (lap_checked_)
    {
      CollectLaps();

      if (draw_style_ == ARROWS)
      {
        transformed &= DrawLapsArrows();
      }
      else
      {
        transformed &= DrawLaps();
      }
    }
    else if (buffer_size_ == INT_MAX)
    {
      buffer_size_ = buffer_holder_;
      laps_.clear();
      got_begin_ = false;
    }
    if (draw_style_ == ARROWS)
    {
      transformed &= DrawArrows();
    }
    else
    {
      transformed &= DrawLines();
    }

    return transformed;
  }

  void PointDrawingPlugin::CollectLaps()
  {
    if (!got_begin_)
    {
      begin_ = cur_point_.point;
      points_.clear();
      buffer_holder_ = buffer_size_;
      buffer_size_ = INT_MAX;
      got_begin_ = true;
    }
    tf::Point check = begin_ - cur_point_.point;
    if (((std::fabs(check.x()) <= 3) && (std::fabs(check.y()) <= 3)) &&
        (new_lap_ == false))
    {
      new_lap_ = true;
      if (points_.size() > 0)
      {
        laps_.push_back(points_);
        laps_[0].pop_back();
        points_.clear();
        points_.push_back(cur_point_);
      }
    }

    if (((std::fabs(check.x()) > 25) && (std::fabs(check.y()) > 25)) &&
        (new_lap_ == true))
    {
      new_lap_ = false;
    }
  }

  bool PointDrawingPlugin::DrawLines()
  {
    bool success = cur_point_.transformed;
    glColor4f(color_.redF(), color_.greenF(), color_.blueF(), 1.0);
    if (draw_style_ == LINES)
    {
      glLineWidth(3);
      glBegin(GL_LINE_STRIP);
    }
    else
    {
      glPointSize(6);
      glBegin(GL_POINTS);
    }

    std::list<StampedPoint>::iterator it = points_.begin();
    for (; it != points_.end(); ++it)
    {
      success &= it->transformed;
      if (it->transformed)
      {
        glVertex2f(it->transformed_point.getX(), it->transformed_point.getY());
      }
    }

    if (cur_point_.transformed)
    {
      glVertex2f(cur_point_.transformed_point.getX(),
                 cur_point_.transformed_point.getY());
    }

    glEnd();

    return success;
  }

  bool PointDrawingPlugin::DrawArrow(const StampedPoint& it)
  {
      if (it.transformed)
      {
        glVertex2f(it.transformed_point.getX(),
                   it.transformed_point.getY());

        glVertex2f(it.transformed_arrow_point.getX(),
                   it.transformed_arrow_point.getY());

        glVertex2f(it.transformed_arrow_point.getX(),
                   it.transformed_arrow_point.getY());
        glVertex2f(it.transformed_arrow_left.getX(),
                   it.transformed_arrow_left.getY());

        glVertex2f(it.transformed_arrow_point.getX(),
                   it.transformed_arrow_point.getY());
        glVertex2f(it.transformed_arrow_right.getX(),
                   it.transformed_arrow_right.getY());
        return true;
       }
      return false;
  }

  bool PointDrawingPlugin::DrawArrows()
  {
    bool success = true;
    glLineWidth(2);
    glBegin(GL_LINES);
    glColor4f(color_.redF(), color_.greenF(), color_.blueF(), 0.5);
    std::list<StampedPoint>::iterator it = points_.begin();
    for (; it != points_.end(); ++it)
    {
      success &= DrawArrow(*it);
    }

    success &= DrawArrow(cur_point_);

    glEnd();

    return success;
  }

  void PointDrawingPlugin::SetColor(const QColor& color)
  {
    if (color != color_)
    {
      color_ = color;
      DrawIcon();
    }
  }

  bool PointDrawingPlugin::TransformPoint(StampedPoint& point)
  {
    swri_transform_util::Transform transform;
    if (GetTransform(point.source_frame, point.stamp, transform))
    {
      point.transformed_point = transform * point.point;

      tf::Transform orientation(tf::Transform(transform.GetOrientation()) *
                                point.orientation);

      double size = static_cast<double>(arrow_size_);
      if (static_arrow_sizes_)
      {
        size *= scale_;
      }
      else
      {
        size /= 10.0;
      }
      double arrow_width = size / 5.0;
      double head_length = size * 0.75;

      point.transformed_arrow_point =
          point.transformed_point + orientation * tf::Point(size, 0.0, 0.0);
      point.transformed_arrow_left =
          point.transformed_point + orientation * tf::Point(head_length, -arrow_width, 0.0);
      point.transformed_arrow_right =
          point.transformed_point + orientation * tf::Point(head_length, arrow_width, 0.0);

      if (covariance_checked_)
      {
        for (uint32_t i = 0; i < point.cov_points.size(); i++)
        {
          point.transformed_cov_points[i] = transform * point.cov_points[i];
        }
      }

      point.transformed = true;
      return true;
    }

    point.transformed = false;
    return false;
  }

  void PointDrawingPlugin::Transform()
  {
    bool transformed = false;

    std::list<StampedPoint>::iterator points_it = points_.begin();
    for (; points_it != points_.end(); ++points_it)
    {
      transformed = transformed | TransformPoint(*points_it);
    }

    transformed = transformed | TransformPoint(cur_point_);
    if (laps_.size() > 0)
    {
      for (size_t i = 0; i < laps_.size(); i++)
      {
        std::list<StampedPoint>::iterator lap_it = laps_[i].begin();
        for (; lap_it != laps_[i].end(); ++lap_it)
        {
          transformed = transformed | TransformPoint(*lap_it);
        }
      }
    }
    if (!points_.empty() && !transformed)
    {
      PrintError("No transform between " + cur_point_.source_frame + " and " +
                 target_frame_);
    }
  }

  bool PointDrawingPlugin::DrawLaps()
  {
    bool transformed = points_.size() != 0;
    glColor4f(color_.redF(), color_.greenF(), color_.blueF(), 0.5);
    glLineWidth(3);
    QColor base_color = color_;
    if (laps_.size() != 0)
    {
      for (size_t i = 0; i < laps_.size(); i++)
      {
        UpdateColor(base_color,i);
        if (draw_style_ == LINES)
        {
          glLineWidth(3);
          glBegin(GL_LINE_STRIP);
        }
        else
        {
          glPointSize(6);
          glBegin(GL_POINTS);
        }

        std::list<StampedPoint>::iterator it = laps_[i].begin();
        for (; it != laps_[i].end(); it++)
        {
          if (it->transformed)
          {
            glVertex2f(it->transformed_point.getX(),
                       it->transformed_point.getY());
          }
        }
        glEnd();
      }
    }

    if (draw_style_ == LINES)
    {
      glLineWidth(3);
      glBegin(GL_LINE_STRIP);
    }
    else
    {
      glPointSize(6);
      glBegin(GL_POINTS);
    }

    glColor4f(base_color.redF(), base_color.greenF(), base_color.blueF(), 0.5);

    if (points_.size() > 0)
    {
      std::list<StampedPoint>::iterator it = points_.begin();
      for (; it != points_.end(); ++it)
      {
        transformed &= it->transformed;
        if (it->transformed)
        {
          glVertex2f(it->transformed_point.getX(),
                     it->transformed_point.getY());
        }
      }
    }

    glEnd();
    return transformed;
  }

  void PointDrawingPlugin::UpdateColor(QColor base_color, int i)
  {
      int hue = color_.hue() + (i + 1) * 10 * M_PI;
      if (hue > 360)
      {
        hue %= 360;
      }
      int sat = color_.saturation();
      int v = color_.value();
      base_color.setHsv(hue, sat, v);
      glColor4f(base_color.redF(), base_color.greenF(), base_color.blueF(),
                0.5);
  }

  bool PointDrawingPlugin::DrawLapsArrows()
  {
    bool success = laps_.size() != 0 && points_.size() != 0;
    glColor4f(color_.redF(), color_.greenF(), color_.blueF(), 0.5);
    glLineWidth(2);
    QColor base_color = color_;
    if (laps_.size() != 0)
    {
      for (size_t i = 0; i < laps_.size(); i++)
      {
        UpdateColor(base_color,i);
        std::list<StampedPoint>::iterator it = laps_[i].begin();
        for (; it != laps_[i].end(); ++it)
        {
          glBegin(GL_LINE_STRIP);
          success &= DrawArrow(*it);
          glEnd();
        }
      }
      glEnd();

      int hue = color_.hue() + laps_.size() * 10 * M_PI;
      int sat = color_.saturation();
      int v = color_.value();
      base_color.setHsv(hue, sat, v);
      glColor4f(base_color.redF(), base_color.greenF(), base_color.blueF(),
                0.5);
    }

    if (points_.size() > 0)
    {
      std::list<StampedPoint>::iterator it = points_.begin();
      for (; it != points_.end(); ++it)
      {
        glBegin(GL_LINE_STRIP);
        success &= DrawArrow(*it);
        glEnd();
      }
    }

    return success;
  }
}
