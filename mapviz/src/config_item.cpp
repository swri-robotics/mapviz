// QT libraries

#include "config_item.h"

ConfigItem::ConfigItem(QWidget *parent, Qt::WFlags flags) :
  QWidget(parent, flags),
  item_(0)
{
  ui_.setupUi(this);
}

ConfigItem::~ConfigItem()
{

}

void ConfigItem::ToggleDraw(bool toggled)
{
  ui_.show->setChecked(toggled);
}

void ConfigItem::SetName(QString name)
{
  name_ = name;
  ui_.namelabel->setText(type_ + " (" + name_ + ")" );
}

void ConfigItem::SetType(QString type)
{
  type_ = type;
  ui_.namelabel->setText(type_ + " (" + name_ + ")" );
}

void ConfigItem::SetWidget(QWidget* widget)
{
  //ui_.verticalLayout->removeWidget(ui_.content);
  ui_.label->hide();
  ui_.content_layout->addWidget(widget);
}

void ConfigItem::Hide()
{
  if (!ui_.content->isHidden())
  {
    ui_.content->hide();
    ui_.signlabel->setText(" + ");
  }
  else
  {
    ui_.content->show();
    ui_.signlabel->setText(" - ");
  }

  Q_EMIT UpdateSizeHint();
}
