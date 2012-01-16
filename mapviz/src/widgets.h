#ifndef WIDGETS_H
#define WIDGETS_H

// QT libraries
#include <QWidget>
#include <QListWidget>
#include <QLabel>
#include <QMouseEvent>
#include <QDropEvent>

class PluginConfigList : public QListWidget
{
  Q_OBJECT

public:
  PluginConfigList(QWidget *parent = 0) : QListWidget(parent) {}
  PluginConfigList();

Q_SIGNALS:
  void ItemsMoved();

protected:
  virtual void dropEvent  (QDropEvent* event)
  {
    QListWidget::dropEvent(event);

    Q_EMIT ItemsMoved();
  }
};

class SingleClickLabel : public QLabel
{
  Q_OBJECT

public:
  SingleClickLabel(QWidget *parent = 0, Qt::WFlags flags = 0) : QLabel(parent, flags) {}
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
  DoubleClickWidget(QWidget *parent = 0, Qt::WFlags flags = 0) : QWidget(parent, flags) {}
  ~DoubleClickWidget() {}

Q_SIGNALS:
  void DoubleClicked();

protected:
  virtual void mouseDoubleClickEvent(QMouseEvent* event)
  {
    Q_EMIT DoubleClicked();
  }
};

#endif /* WIDGETS_H */
