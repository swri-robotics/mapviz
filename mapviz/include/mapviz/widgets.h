// *****************************************************************************
//
// Copyright (C) 2013 All Right Reserved, Southwest Research Institute® (SwRI®)
//
// Contract No.  10-58058A
// Contractor    Southwest Research Institute® (SwRI®)
// Address       6220 Culebra Road, San Antonio, Texas 78228-0510
// Contact       Steve Dellenback <sdellenback@swri.org> (210) 522-3914
//
// This code was developed as part of an internal research project fully funded
// by Southwest Research Institute®.
//
// THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
// KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
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
