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

#ifndef MAPVIZ_WIDGETS_H_
#define MAPVIZ_WIDGETS_H_

// QT libraries
#include <QWidget>
#include <QListWidget>
#include <QLabel>
#include <QMouseEvent>
#include <QDropEvent>

namespace mapviz
{
  class PluginConfigList : public QListWidget
  {
    Q_OBJECT

  public:
    explicit PluginConfigList(QWidget *parent = 0) : QListWidget(parent) {}
    PluginConfigList();

  Q_SIGNALS:
    void ItemsMoved();

  protected:
    virtual void dropEvent(QDropEvent* event)
    {
      QListWidget::dropEvent(event);

      Q_EMIT ItemsMoved();
    }
  };

  class SingleClickLabel : public QLabel
  {
    Q_OBJECT

  public:
    SingleClickLabel(QWidget *parent = 0, Qt::WFlags flags = 0) :
      QLabel(parent, flags) {}

    ~SingleClickLabel() {}

  Q_SIGNALS:
    void Clicked();

  protected:
    virtual void mousePressEvent(QMouseEvent* event)
    {
      Q_EMIT Clicked();
    }
  };

  class DoubleClickWidget : public QWidget
  {
    Q_OBJECT

  public:
    DoubleClickWidget(QWidget *parent = 0, Qt::WFlags flags = 0) :
      QWidget(parent, flags) {}

    ~DoubleClickWidget() {}

  Q_SIGNALS:
    void DoubleClicked();
    void RightClicked();

  protected:
    virtual void mouseDoubleClickEvent(QMouseEvent* event)
    {
      if (event->button() == Qt::LeftButton)
      {
        Q_EMIT DoubleClicked();
      }
    }
    
    virtual void mouseReleaseEvent(QMouseEvent* event)
    {
      if (event->button() == Qt::RightButton)
      {
        Q_EMIT RightClicked();
      }
    }
  };
}

#endif  // MAPVIZ_WIDGETS_H_
