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

#ifndef MAPVIZ__CONFIG_ITEM_H_
#define MAPVIZ__CONFIG_ITEM_H_

// QT libraries
#include <QWidget>
#include <QLabel>
#include <QMouseEvent>
#include <QListWidgetItem>

// C++ standard libraries
#include <string>
#include <vector>

// Auto-generated UI files
#include "ui_configitem.h"

namespace mapviz
{
class ConfigItem : public QWidget
{
  Q_OBJECT

public:
  explicit ConfigItem(QWidget *parent = nullptr, Qt::WindowFlags flags = Qt::WindowFlags());
  ~ConfigItem() override = default;

  void SetName(QString name);
  void SetType(QString type);
  void SetWidget(QWidget* widget);

  void SetListItem(QListWidgetItem* item) { item_ = item; }
  bool Collapsed() const { return ui_.content->isHidden(); }
  QString Name() const { return name_; }

  Ui::configitem ui_;

Q_SIGNALS:
  void UpdateSizeHint();
  void ToggledDraw(QListWidgetItem* plugin, bool visible);
  void RemoveRequest(QListWidgetItem* plugin);

public Q_SLOTS:
  void Hide();
  void EditName();
  void Remove();
  void ToggleDraw(bool toggled);

private:
  void contextMenuEvent(QContextMenuEvent *event) override;

protected:
  QListWidgetItem* item_;
  QString name_;
  QString type_;
  QAction* edit_name_action_;
  QAction* remove_item_action_;
  bool visible_;
};
}   // namespace mapviz

#endif  // MAPVIZ__CONFIG_ITEM_H_
