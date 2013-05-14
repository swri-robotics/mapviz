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

#ifndef MAPVIZ_CONFIG_ITEM_H_
#define MAPVIZ_CONFIG_ITEM_H_

// C++ standard libraries
#include <string>
#include <vector>

// QT libraries
#include <QWidget>
#include <QLabel>
#include <QMouseEvent>
#include <QListWidgetItem>

// Auto-generated UI files
#include "ui_configitem.h"

class ConfigItem : public QWidget
{
  Q_OBJECT

public:
  ConfigItem(QWidget *parent = 0, Qt::WFlags flags = 0);
  ~ConfigItem();

  void SetName(QString name);
  void SetType(QString type);
  void SetWidget(QWidget* widget);
  void ToggleDraw(bool toggled);
  void SetListItem(QListWidgetItem* item) { item_ = item; }
  bool Collapsed() const { return ui_.content->isHidden(); }

Q_SIGNALS:
  void UpdateSizeHint();
  void ToggledDraw(QListWidgetItem* plugin, bool visible);

public Q_SLOTS:
  void Hide();
  void Draw(bool on) { Q_EMIT ToggledDraw(item_, on); }

protected:

  Ui::configitem ui_;

  QListWidgetItem* item_;
  QString name_;
  QString type_;

};

#endif  // MAPVIZ_CONFIG_ITEM_H_
