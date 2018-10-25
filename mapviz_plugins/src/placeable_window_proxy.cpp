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

#include <mapviz_plugins/placeable_window_proxy.h>

#include <math.h>
#include <cmath>

#include <QApplication>
#include <QCursor>
#include <QLine>
#include <QMouseEvent>
#include <QResizeEvent>
#include <QTimerEvent>
#include <QWidget>

#include <QDebug>

namespace mapviz_plugins
{
PlaceableWindowProxy::PlaceableWindowProxy()
  :
  target_(NULL),
  visible_(true),
  has_cursor_(false),
  state_(INACTIVE),
  win_resize_timer_(-1)
{
}

PlaceableWindowProxy::~PlaceableWindowProxy()
{
  if (target_)
  {
    target_->removeEventFilter(this);
  }
}

void PlaceableWindowProxy::setContainer(QWidget *target)
{
  if (target_)
  {
    target_->removeEventFilter(this);
  }

  target_ = target;

  if (target_)
  {
    target_->installEventFilter(this);
  }
}

QRect PlaceableWindowProxy::rect() const
{
  return rect_.toRect();
}

void PlaceableWindowProxy::setRect(const QRect &rect)
{
  rect_ = QRectF(rect);
  state_ = INACTIVE;
}

void PlaceableWindowProxy::setVisible(bool visible)
{
  if (visible == visible_)
  {
    return;
  }

  visible_ = visible;

  if (!visible_ && state_ != INACTIVE)
  {
    if (has_cursor_)
    {
      QApplication::restoreOverrideCursor();
      has_cursor_ = false;
    }
    state_ = INACTIVE;
  } 
}

bool PlaceableWindowProxy::eventFilter(QObject *, QEvent *event)
{
  // This should never happen, but doesn't hurt to be defensive.
  if (!target_)
  {
    return false;
  }

  if (!visible_)
  {
    return false;
  }

  switch (event->type())
  {
  case QEvent::MouseButtonPress:
    return handleMousePress(static_cast<QMouseEvent*>(event));
  case QEvent::MouseButtonRelease:
    return handleMouseRelease(static_cast<QMouseEvent*>(event));
  case QEvent::MouseMove:
    return handleMouseMove(static_cast<QMouseEvent*>(event));
  case QEvent::Resize:
    return handleResize(static_cast<QResizeEvent*>(event));
  default:
    return false;
  }
}

bool PlaceableWindowProxy::handleMousePress(QMouseEvent *event)
{
  if (!visible_)
  {
    return false;
  }

  if (!rect_.contains(event->pos()))
  {
    // We don't care about anything outside the rect.
     return false;
  }

  if (state_ != INACTIVE)
  {
    // We're already doing something, so we don't want to enter
    // another state.  But we also don't want someone else to start
    // doing something, so we filter out the press.
    return true;
  }
  
  if (event->button() == Qt::LeftButton)
  {
    start_rect_ = rect_;
    start_point_ = event->pos();
#if QT_VERSION >= 0x050000
    state_ = getNextState(event->localPos());
#else
    state_ = getNextState(event->posF());
#endif
    qWarning("changing state to %d", state_);
    return true;
  }

  // Event if we're not doing anything, we want to present a
  // consistent interface that says "this region is belongs to me", so
  // we filter out events.
  return true;
}

bool PlaceableWindowProxy::handleMouseRelease(QMouseEvent *event)
{
  if (!visible_)
  {
    return false;
  }

  if (state_ == INACTIVE)
  {
    return false;
  }

  if (event->button() == Qt::LeftButton)
  {
    state_ = INACTIVE;
    return true;    
  }
  
  return false;
}

bool PlaceableWindowProxy::handleMouseMove(QMouseEvent *event)
{
  if (!visible_)
  {
    return false;
  }

  if (state_ == INACTIVE)
  {
#if QT_VERSION >= 0x050000
    if (!rect_.contains(event->localPos()))
#else
    if (!rect_.contains(event->posF()))
#endif
    {
      if (has_cursor_)
      {
        QApplication::restoreOverrideCursor();
        has_cursor_ = false;
      }
      return false;
    }    

    // The mouse cursor is over the rect, so we're going to change the
    // cursor to indicate the state the user would enter by clicking.

    Qt::CursorShape shape;
#if QT_VERSION >= 0x050000
    switch(getNextState(event->localPos()))
#else
    switch(getNextState(event->posF()))
#endif
    {
    case MOVE_TOP_LEFT:
    case MOVE_BOTTOM_RIGHT:
      shape = Qt::SizeFDiagCursor;
      break;
    case MOVE_TOP_RIGHT:
    case MOVE_BOTTOM_LEFT:
      shape = Qt::SizeBDiagCursor;
      break;
    default:
      shape = Qt::SizeAllCursor;
    }

    if (has_cursor_)
    {
      QApplication::changeOverrideCursor(QCursor(shape));
    }
    else
    {
      QApplication::setOverrideCursor(QCursor(shape));
      has_cursor_ = true;
    }

    return true;
  }

#if QT_VERSION >= 0x050000
  QPointF dp = event->localPos() - start_point_;
#else
  QPointF dp = event->posF() - start_point_;
#endif

  // todo: enforce minimum size & constrain aspect ratio for resizes.  
  if (state_ == MOVE_ALL)
  {
    rect_ = start_rect_.translated(dp);    
  }
  else if (state_ == MOVE_TOP_LEFT)
  {
    rect_= resizeHelper(start_rect_,
                        start_rect_.bottomRight(),
                        start_rect_.topLeft(),
#if QT_VERSION >= 0x050000
                        event->localPos());
#else
                        event->posF());
#endif
    rect_.moveBottomRight(start_rect_.bottomRight());      
  }
  else if (state_ == MOVE_BOTTOM_LEFT)
  {
    rect_= resizeHelper(start_rect_,
                        start_rect_.topRight(),
                        start_rect_.bottomLeft(),
#if QT_VERSION >= 0x050000
                        event->localPos());
#else
                        event->posF());
#endif
    rect_.moveTopRight(start_rect_.topRight());
  }
  else if (state_ == MOVE_BOTTOM_RIGHT)
  {
    rect_= resizeHelper(start_rect_,
                        start_rect_.topLeft(),
                        start_rect_.bottomRight(),
#if QT_VERSION >= 0x050000
                        event->localPos());
#else
                        event->posF());
#endif
    rect_.moveTopLeft(start_rect_.topLeft());      
  }
  else if (state_ == MOVE_TOP_RIGHT)
  {
    rect_= resizeHelper(start_rect_,
                        start_rect_.bottomLeft(),
                        start_rect_.topRight(),
#if QT_VERSION >= 0x050000
                        event->localPos());
#else
                        event->posF());
#endif
    rect_.moveBottomLeft(start_rect_.bottomLeft());      
  }
  else
  {
    qWarning("Unhandled state in PlaceableWindowProxy: %d", state_);
  }

  return true;
}

QRectF PlaceableWindowProxy::resizeHelper(const QRectF &rect,
                                          const QPointF &p1,
                                          const QPointF &p2,
                                          const QPointF &p3) const
{
  QPointF v1 = p2 - p1;
  QPointF v2 = p3 - p1;

  double d = v1.x()*v2.y() - v1.y()*v2.x();
  if (d < 0)
  {
    double new_width = std::abs(p3.x() - p1.x());
    if (new_width < 10)
    {
      new_width = 10;
    }

    double new_height = rect.height() / rect.width() * new_width;
    return QRectF(0, 0, new_width, new_height);
  }
  else
  {
    double new_height = std::abs(p3.y() - p1.y());
    if (new_height < 10)
    {
      new_height = 10;
    }

    double new_width = rect.width() / rect.height() * new_height;
    return QRectF(0, 0, new_width, new_height);
  }
}


bool PlaceableWindowProxy::handleResize(QResizeEvent *event)
{
  // We always want to pass the resize event along to other widgets.
  return false;
}

void PlaceableWindowProxy::timerEvent(QTimerEvent *event)
{
  if (event->timerId() == win_resize_timer_)
  {
    killTimer(win_resize_timer_);
    win_resize_timer_ = -1;
    if (target_)
    {
      winResize(target_->size());
    }
  }
}

void PlaceableWindowProxy::rectResize(int dx, int dy)
{
}

void PlaceableWindowProxy::winResize(const QSize &size)
{
}

PlaceableWindowProxy::State PlaceableWindowProxy::getNextState(
  const QPointF &pt) const
{
  if (!rect_.contains(pt))
  {
    return INACTIVE;
  }

  const double threshold = 10.0;  
  double near_left = pt.x() - rect_.left() < threshold;
  double near_top = pt.y() - rect_.top() < threshold;
  double near_right = rect_.right() - pt.x() < threshold;
  double near_bottom = rect_.bottom() - pt.y() < threshold;

  if (near_top && near_left)
  {
    return MOVE_TOP_LEFT;
  }
  else if (near_top && near_right)
  {
    return MOVE_TOP_RIGHT;
  }
  else if (near_bottom && near_left)
  {
    return MOVE_BOTTOM_LEFT;
  }
  else if (near_bottom && near_right)
  {
    return MOVE_BOTTOM_RIGHT;
  }
  else
  {
    return MOVE_ALL;
  }
}
}  // namespace mapviz_plugins
