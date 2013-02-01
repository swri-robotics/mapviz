#ifndef CONFIG_ITEM_H
#define CONFIG_ITEM_H

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

#endif /* CONFIG_ITEM_H */
