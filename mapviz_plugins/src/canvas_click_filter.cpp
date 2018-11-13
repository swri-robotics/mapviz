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

#include <QMouseEvent>
#include <QLineF>
#include <QDateTime>
#include "mapviz_plugins/canvas_click_filter.h"

namespace mapviz_plugins
{
  CanvasClickFilter::CanvasClickFilter() :
    is_mouse_down_(false),
    max_ms_(Q_INT64_C(500)),
    max_distance_(2.0)
  { }

  void CanvasClickFilter::setMaxClickTime(qint64 max_ms)
  {
    max_ms_ = max_ms;
  }

  void CanvasClickFilter::setMaxClickMovement(qreal max_distance)
  {
    max_distance_ = max_distance;
  }

  bool CanvasClickFilter::eventFilter(QObject* object, QEvent* event)
  {
    if (event->type() == QEvent::MouseButtonPress)
    {
      is_mouse_down_ = true;
      QMouseEvent* me = static_cast<QMouseEvent*>(event);
#if QT_VERSION >= 0x050000
      mouse_down_pos_ = me->localPos();
#else
      mouse_down_pos_ = me->posF();
#endif
      mouse_down_time_ = QDateTime::currentMSecsSinceEpoch();
    }
    else if (event->type() == QEvent::MouseButtonRelease)
    {
      if (is_mouse_down_)
      {
        QMouseEvent* me = static_cast<QMouseEvent*>(event);

#if QT_VERSION >= 0x050000
        qreal distance = QLineF(mouse_down_pos_, me->localPos()).length();
#else
        qreal distance = QLineF(mouse_down_pos_, me->posF()).length();
#endif
        qint64 msecsDiff = QDateTime::currentMSecsSinceEpoch() - mouse_down_time_;

        // Only fire the event if the mouse has moved less than the maximum distance
        // and was held for shorter than the maximum time..  This prevents click
        // events from being fired if the user is dragging the mouse across the map
        // or just holding the cursor in place.
        if (msecsDiff < max_ms_ && distance <= max_distance_)
        {
#if QT_VERSION >= 0x050000
          Q_EMIT pointClicked(me->localPos());
#else
          Q_EMIT pointClicked(me->posF());
#endif
        }
      }
      is_mouse_down_ = false;
    }
    return false;
  }
}
