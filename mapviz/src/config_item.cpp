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

#include <mapviz/config_item.h>

namespace mapviz
{
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
    ui_.namelabel->setText(type_ + " (" + name_ + ")");
  }

  void ConfigItem::SetType(QString type)
  {
    type_ = type;
    ui_.namelabel->setText(type_ + " (" + name_ + ")");
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
}
