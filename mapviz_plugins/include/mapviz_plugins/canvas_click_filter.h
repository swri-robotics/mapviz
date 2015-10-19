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

#ifndef MAPVIZ_PLUGINS_CANVAS_CLICK_FILTER_H
#define MAPVIZ_PLUGINS_CANVAS_CLICK_FILTER_H

#include <QObject>
#include <QPoint>

/**
 * This is a very simple event filter that listens for mouseReleased events;
 * when it sees one, it emits a signal with the given point.
 *
 * Click events are filtered by how long the mouse was held down and how far the
 * cursor moved in order to prevent the user holding and dragging the map
 * from firing a click event.  By default, "clicks" that take longer than 500ms
 * or move longer than 2 pixels are ignored.
 */
namespace mapviz_plugins
{
  class CanvasClickFilter : public QObject
  {
    Q_OBJECT

  public:
    CanvasClickFilter();

    void setMaxClickTime(qint64 max_ms);
    void setMaxClickMovement(qreal max_distance);

  Q_SIGNALS:
    void pointClicked(const QPointF&);

  protected:
    bool eventFilter(QObject *object, QEvent* event);

  private:
    bool is_mouse_down_;
    QPointF mouse_down_pos_;
    qint64 mouse_down_time_;

    qint64 max_ms_;
    qreal max_distance_;
  };
}

#endif //MAPVIZ_PLUGINS_CANVAS_CLICK_FILTER_H
