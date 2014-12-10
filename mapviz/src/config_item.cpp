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

#include <mapviz/config_item.h>

#include <QInputDialog>

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
    ui_.label->hide();
    ui_.content_layout->addWidget(widget);
  }

  void ConfigItem::EditName()
  {
    bool ok;
    QString text = QInputDialog::getText(
      this, 
      tr("Set Display name"),
      tr(""), 
      QLineEdit::Normal,
      name_, &ok);
     
    if (ok && !text.isEmpty())
    {
      SetName(text);
    }
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
