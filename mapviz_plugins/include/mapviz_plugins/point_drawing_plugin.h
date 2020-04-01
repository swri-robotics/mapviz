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

#ifndef MAPVIZ_PLUGINS__POINT_DRAWING_PLUGIN_H_
#define MAPVIZ_PLUGINS__POINT_DRAWING_PLUGIN_H_

#include <mapviz/mapviz_plugin.h>
#include <mapviz/map_canvas.h>

// QT libraries
#include <QGLWidget>
#include <QObject>
#include <QWidget>

// ROS libraries
#include <rclcpp/rclcpp.hpp>
#include <tf2/transform_datatypes.h>

// C++ standard libraries
#include <deque>
#include <list>
#include <string>
#include <vector>

namespace mapviz_plugins
{
class PointDrawingPlugin : public mapviz::MapvizPlugin
{
  Q_OBJECT

  public:
  struct StampedPoint
  {
    StampedPoint(): transformed(false) {}

    tf2::Vector3 point;
    tf2::Quaternion orientation;
    tf2::Vector3 transformed_point;
    tf2::Vector3 transformed_arrow_point;
    tf2::Vector3 transformed_arrow_left;
    tf2::Vector3 transformed_arrow_right;
    std::string source_frame;
    bool transformed;
    rclcpp::Time stamp;

    std::vector<tf2::Vector3> cov_points;
    std::vector<tf2::Vector3> transformed_cov_points;
  };

  enum DrawStyle
  {
    LINES = 0,
    POINTS,
    ARROWS
  };

  PointDrawingPlugin();
  ~PointDrawingPlugin() override = default;
  void ClearHistory() override;

  void Transform() override;
  virtual bool DrawPoints(double scale);
  virtual bool DrawArrows();
  virtual bool DrawArrow(const StampedPoint& point);
  virtual bool DrawLaps();
  virtual bool DrawLines();
  virtual void CollectLaps();
  virtual bool DrawLapsArrows();
  virtual bool TransformPoint(StampedPoint& point);
  virtual void UpdateColor(QColor base_color, int i);
  virtual void DrawCovariance();

  protected Q_SLOTS:
  virtual void BufferSizeChanged(int value);
  void DrawIcon() override;
  virtual void SetColor(const QColor& color);
  virtual void SetDrawStyle(QString style);
  virtual void SetDrawStyle(DrawStyle style);
  virtual void SetStaticArrowSizes(bool isChecked);
  virtual void SetArrowSize(int arrowSize);
  virtual void PositionToleranceChanged(double value);
  virtual void LapToggled(bool checked);
  virtual void CovariancedToggled(bool checked);
  virtual void ShowAllCovariancesToggled(bool checked);
  void ResetTransformedPoints();
  void ClearPoints();

  protected:
  void pushPoint(StampedPoint point);
  double bufferSize() const;
  double positionTolerance() const;
  const std::deque<StampedPoint>& points() const;

  private:
  int arrow_size_;
  DrawStyle draw_style_;
  StampedPoint cur_point_;
  std::deque<StampedPoint> points_;
  double position_tolerance_;
  int buffer_size_;
  bool covariance_checked_;
  bool show_all_covariances_checked_;
  bool new_lap_;
  QColor color_;
  bool lap_checked_;
  int buffer_holder_;
  double scale_;
  bool static_arrow_sizes_;

  private:
  std::vector<std::deque<StampedPoint> > laps_;
  bool got_begin_;
  tf2::Vector3 begin_;
};
}   // namespace mapviz_plugins

#endif  // MAPVIZ_PLUGINS__POINT_DRAWING_PLUGIN_H_
