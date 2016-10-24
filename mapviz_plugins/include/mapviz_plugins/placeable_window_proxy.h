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
#ifndef MAPVIZ_PLUGINS_PLACEABLE_WINDOW_PROXY_H
#define MAPVIZ_PLUGINS_PLACEABLE_WINDOW_PROXY_H

#include <QObject>
#include <QRect>
#include <QPoint>

QT_BEGIN_NAMESPACE;
class QMouseEvent;
class QResizeEvent;
QT_END_NAMESPACE;

/**
 * This object supports moving/resizing a "window" on the canvas.  It
 * should be installed as an event filter on the map canvas and be
 * given an initial rectangle.  It will watch for mouse events that
 * occur within the rectangle and move/resize the rectangle
 * accordingly.  You can listen to signals to determine when the
 * window was moved, or just use the rectangle at the start of every
 * draw.
 *
 * Since this class is designed for moving windows around the canvas,
 * it does everything in terms of pixels.
 *
 * To do: Need to deactivate the proxy when the plugin is marked
 * invisible, or is not drawing data.
 */
namespace mapviz_plugins
{
  class PlaceableWindowProxy : public QObject
  {
  Q_OBJECT

  public:
    PlaceableWindowProxy();
    ~PlaceableWindowProxy();

    void setContainer(QWidget *);

    QRect rect() const;

  Q_SIGNALS:
    void rectChanged(const QRect &);

  public Q_SLOTS:
    void setRect(const QRect &);
    void setVisible(bool visible);

  protected:
    bool eventFilter(QObject *object, QEvent *event);

    bool handleMousePress(QMouseEvent *);
    bool handleMouseRelease(QMouseEvent *);
    bool handleMouseMove(QMouseEvent *);
    bool handleResize(QResizeEvent *);

    void timerEvent(QTimerEvent *);

    void rectResize(int dx, int dy);
    void winResize(const QSize &);

    QRectF resizeHelper(const QRectF &rect,
                        const QPointF &p1,
                        const QPointF &p2,
                        const QPointF &p3) const;



  private:
    enum State {
      INACTIVE = 0,
      MOVE_ALL,
      MOVE_TOP_LEFT,
      MOVE_BOTTOM_LEFT,
      MOVE_BOTTOM_RIGHT,
      MOVE_TOP_RIGHT
    };

    QWidget *target_;
    bool visible_;

    bool has_cursor_;
    State state_;
    QRectF rect_;

    QRectF start_rect_;
    QPoint start_point_;

    int win_resize_timer_;

    State getNextState(const QPointF &pt) const;
  };  // class PlaceableWindowProxy
}  // namespace mapviz_plugins
#endif //MAPVIZ_PLUGINS_CANVAS_CLICK_FILTER_H
