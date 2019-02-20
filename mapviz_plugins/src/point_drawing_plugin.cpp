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
    QObject::connect(this,
                     SIGNAL(TargetFrameChanged(const std::string&)),
                     this,
                     SLOT(ResetTransformedPoints()));
  }

  void PointDrawingPlugin::ClearHistory()
  {
    points_.clear();
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
    ResetTransformedPoints();
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
    ResetTransformedPoints();
    DrawIcon();
  }

  void PointDrawingPlugin::SetDrawStyle(PointDrawingPlugin::DrawStyle style)
  {
     draw_style_ = style;
     DrawIcon();
  }

  void PointDrawingPlugin::SetStaticArrowSizes(bool isChecked)
  {
    static_arrow_sizes_ = isChecked;
    ResetTransformedPoints();
  }

  void PointDrawingPlugin::PositionToleranceChanged(double value)
  {
    position_tolerance_ = value;
  }

  void PointDrawingPlugin::LapToggled(bool checked)
  {
    lap_checked_ = checked;
  }

  void PointDrawingPlugin::CovariancedToggled(bool checked)
  {
    covariance_checked_ = checked;
  }

  void PointDrawingPlugin::ResetTransformedPoints()
  {
    for (std::deque<StampedPoint>& lap: laps_)
    {
      for (StampedPoint& point: lap)
      {
        point.transformed = false;
      }
    }
    for (StampedPoint& point: points_)
    {
      point.transformed = false;
    }
    Transform();
  }

  void PointDrawingPlugin::pushPoint(PointDrawingPlugin::StampedPoint stamped_point)
  {
    cur_point_ = stamped_point;

    if (points_.empty() ||
        (stamped_point.point.distance(points_.back().point)) >=
            (position_tolerance_))
    {
      points_.push_back(stamped_point);
    }

    if (buffer_size_ > 0)
    {
      while (static_cast<int>(points_.size()) >= buffer_size_)
      {
        points_.pop_front();
      }
    }
  }

  void PointDrawingPlugin::ClearPoints()
  {
    points_.clear();
  }

  double PointDrawingPlugin::bufferSize() const
  {
    if (!lap_checked_)
    {
      return buffer_size_;
    }
    else
    {
      return buffer_holder_;
    }
  }

  double PointDrawingPlugin::positionTolerance() const
  {
    return position_tolerance_;
  }

  const std::deque<PointDrawingPlugin::StampedPoint> &PointDrawingPlugin::points() const
  {
    return points_;
  }

  void PointDrawingPlugin::BufferSizeChanged(int value)
  {
    buffer_size_ = value;

    if (buffer_size_ > 0)
    {
      while (static_cast<int>(points_.size()) >= buffer_size_)
      {
        points_.pop_front();
      }
    }
  }

  bool PointDrawingPlugin::DrawPoints(double scale)
  {
    if( scale_ != scale && draw_style_ == ARROWS && static_arrow_sizes_)
    {
      ResetTransformedPoints();
    }
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
        !new_lap_)
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
        new_lap_)
    {
      new_lap_ = false;
    }
  }

  bool PointDrawingPlugin::DrawLines()
  {
    bool success = cur_point_.transformed;
    glColor4d(color_.redF(), color_.greenF(), color_.blueF(), 1.0);
    if (draw_style_ == LINES && points_.size()>0)
    {
      glLineWidth(3);
      glBegin(GL_LINE_STRIP);
    }
    else
    {
      glPointSize(6);
      glBegin(GL_POINTS);
    }

    for (const auto& pt : points_)
    {
      success &= pt.transformed;
      if (pt.transformed)
      {
        glVertex2d(pt.transformed_point.getX(), pt.transformed_point.getY());
      }
    }

    if (cur_point_.transformed)
    {
      glVertex2d(cur_point_.transformed_point.getX(),
                 cur_point_.transformed_point.getY());
    }

    glEnd();

    return success;
  }

  bool PointDrawingPlugin::DrawArrow(const StampedPoint& it)
  {
      if (it.transformed)
      {
        glVertex2d(it.transformed_point.getX(),
                   it.transformed_point.getY());

        glVertex2d(it.transformed_arrow_point.getX(),
                   it.transformed_arrow_point.getY());

        glVertex2d(it.transformed_arrow_point.getX(),
                   it.transformed_arrow_point.getY());
        glVertex2d(it.transformed_arrow_left.getX(),
                   it.transformed_arrow_left.getY());

        glVertex2d(it.transformed_arrow_point.getX(),
                   it.transformed_arrow_point.getY());
        glVertex2d(it.transformed_arrow_right.getX(),
                   it.transformed_arrow_right.getY());
        return true;
       }
      return false;
  }

  bool PointDrawingPlugin::DrawArrows()
  {
    bool success = true;
    glLineWidth(4);
    glBegin(GL_LINES);
    glColor4d(color_.redF(), color_.greenF(), color_.blueF(), 0.5);
    for (const auto &pt : points_)
    {
      success &= DrawArrow(pt);
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
    if ( point.transformed )
    {
      return true;
    }

    swri_transform_util::Transform transform;
    if( GetTransform(point.source_frame, point.stamp, transform))
    {
      point.transformed_point = transform * point.point;

      if (draw_style_ == ARROWS)
      {
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
      }

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

    for (auto &pt : points_)
    {
      transformed = transformed | TransformPoint(pt);
    }

    transformed = transformed | TransformPoint(cur_point_);
    if (laps_.size() > 0)
    {
      for (auto &lap : laps_)
      {
        for (auto &pt : lap)
        {
          transformed = transformed | TransformPoint(pt);
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
    glColor4d(color_.redF(), color_.greenF(), color_.blueF(), 0.5);
    glLineWidth(3);
    QColor base_color = color_;

    for (size_t i = 0; i < laps_.size(); i++)
    {
      UpdateColor(base_color, static_cast<int>(i));
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

      for (const auto& pt : laps_[i])
      {
        if (pt.transformed)
        {
          glVertex2d(pt.transformed_point.getX(),
                     pt.transformed_point.getY());
        }
      }
      glEnd();
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

    glColor4d(base_color.redF(), base_color.greenF(), base_color.blueF(), 0.5);

    if (points_.size() > 0)
    {
      for (const auto &pt : points_)
      {
        transformed &= pt.transformed;
        if (pt.transformed)
        {
          glVertex2d(pt.transformed_point.getX(),
                     pt.transformed_point.getY());
        }
      }
    }

    glEnd();
    return transformed;
  }

  void PointDrawingPlugin::UpdateColor(QColor base_color, int i)
  {
      int hue = static_cast<int>(color_.hue() + (i + 1.0) * 10.0 * M_PI);
      if (hue > 360)
      {
        hue %= 360;
      }
      int sat = color_.saturation();
      int v = color_.value();
      base_color.setHsv(hue, sat, v);
      glColor4d(base_color.redF(), base_color.greenF(), base_color.blueF(),
                0.5);
  }

  void PointDrawingPlugin::DrawCovariance()
  {
    glLineWidth(4);

    glColor4d(color_.redF(), color_.greenF(), color_.blueF(), 1.0);

    if (cur_point_.transformed && !cur_point_.transformed_cov_points.empty())
    {
      glBegin(GL_LINE_STRIP);

      for (uint32_t i = 0; i < cur_point_.transformed_cov_points.size(); i++)
      {
        glVertex2d(cur_point_.transformed_cov_points[i].getX(),
                   cur_point_.transformed_cov_points[i].getY());
      }

      glVertex2d(cur_point_.transformed_cov_points.front().getX(),
                 cur_point_.transformed_cov_points.front().getY());

      glEnd();
    }
  }

  bool PointDrawingPlugin::DrawLapsArrows()
  {
    bool success = laps_.size() != 0 && points_.size() != 0;
    glColor4d(color_.redF(), color_.greenF(), color_.blueF(), 0.5);
    glLineWidth(2);
    QColor base_color = color_;
    if (laps_.size() != 0)
    {
      for (size_t i = 0; i < laps_.size(); i++)
      {
        UpdateColor(base_color, static_cast<int>(i));
        for (const auto &pt : laps_[i])
        {
          glBegin(GL_LINE_STRIP);
          success &= DrawArrow(pt);
          glEnd();
        }
      }
      glEnd();

      int hue = static_cast<int>(color_.hue() + laps_.size() * 10.0 * M_PI);
      int sat = color_.saturation();
      int v = color_.value();
      base_color.setHsv(hue, sat, v);
      glColor4d(base_color.redF(), base_color.greenF(), base_color.blueF(),
                0.5);
    }

    if (points_.size() > 0)
    {
      for (const auto& pt : points_)
      {
        glBegin(GL_LINE_STRIP);
        success &= DrawArrow(pt);
        glEnd();
      }
    }

    return success;
  }
}
