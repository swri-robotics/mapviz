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

#ifndef MAPVIZ_WIDGETS_H_
#define MAPVIZ_WIDGETS_H_

// QT libraries
#include <QWidget>
#include <QListWidget>
#include <QListWidgetItem>
#include <QLabel>
#include <QMouseEvent>
#include <QDropEvent>
#include <QPainter>
#include <QPixmap>

namespace mapviz
{
  class PluginConfigList : public QListWidget
  {
    Q_OBJECT

  public:
    explicit PluginConfigList(QWidget *parent = 0) : QListWidget(parent) {}
    PluginConfigList();
    
    void UpdateIndices()
    {
      for (size_t i = 0; i < count(); i++)
      {
        item(i)->setData(Qt::UserRole, QVariant((float)i));
      }
    }

  Q_SIGNALS:
    void ItemsMoved();

  protected:
    virtual void dropEvent(QDropEvent* event)
    {
      QListWidget::dropEvent(event);

      UpdateIndices();

      Q_EMIT ItemsMoved();
    }
  };
  
  class PluginConfigListItem : public QListWidgetItem
  {
   
  public:
    explicit PluginConfigListItem(QListWidget *parent = 0) : QListWidgetItem(parent) {}
    
    virtual bool operator< (const QListWidgetItem & other) const 
    {
      return data(Qt::UserRole).toFloat() < other.data(Qt::UserRole).toFloat();
    }
  };

  class SingleClickLabel : public QLabel
  {
    Q_OBJECT

  public:
    explicit SingleClickLabel(QWidget *parent = 0, Qt::WindowFlags flags = 0) :
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
    explicit DoubleClickWidget(QWidget *parent = 0, Qt::WindowFlags flags = 0) :
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
  
  class IconWidget : public QWidget
  {
    Q_OBJECT

  public:
    explicit IconWidget(QWidget *parent = 0, Qt::WindowFlags flags = 0) :
      QWidget(parent, flags)
    {
      pixmap_ = QPixmap(16, 16);
      pixmap_.fill(Qt::transparent);
    }

    ~IconWidget() {}
    
    void SetPixmap(QPixmap pixmap)
    {
      pixmap_ = pixmap;
      update();
    }

  protected:
    virtual void paintEvent(QPaintEvent* e)
    {
      QPainter painter(this);
      painter.fillRect(0, 0, width(), height(), palette().color(QPalette::Button));
      
      int x_offset = (width() - pixmap_.width()) / 2.0;
      int y_offset = (height() - pixmap_.height()) / 2.0;
      
      painter.drawPixmap(x_offset, y_offset, pixmap_);
    }
    
    QPixmap pixmap_;
  };
}

#endif  // MAPVIZ_WIDGETS_H_
