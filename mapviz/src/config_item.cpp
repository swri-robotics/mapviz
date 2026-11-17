// *****************************************************************************
//
// Copyright (c) 2014, Southwest Research Institute® (SwRI®)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#include <mapviz/config_item.hpp>

#include <QMenu>
#include <QAction>
#include <QFontMetrics>
#include <QAbstractSpinBox>
#include <QInputDialog>
#include <QLayout>
#include <QList>
#include <QSizePolicy>
#include <QSpacerItem>
#include <QStyle>

#include <algorithm>

#include "mapviz/color_button.hpp"
#include "mapviz/theme.hpp"
#include "mapviz/widgets.hpp"

namespace mapviz
{
namespace
{
/// Collapses vertical spacers to zero height.  They still absorb any spare
/// height, but no longer reserve empty space at the bottom of a form.
void CollapseVerticalSpacers(QLayout * layout)
{
  if (layout == nullptr) {
    return;
  }
  for (int i = 0; i < layout->count(); i++) {
    QLayoutItem * item = layout->itemAt(i);
    if (QSpacerItem * spacer = item->spacerItem()) {
      const QSizePolicy policy = spacer->sizePolicy();
      if (spacer->expandingDirections() & Qt::Vertical) {
        spacer->changeSize(
          spacer->sizeHint().width(), 0,
          policy.horizontalPolicy(), policy.verticalPolicy());
      }
    } else if (item->layout() != nullptr) {
      CollapseVerticalSpacers(item->layout());
    }
  }
  layout->invalidate();
}
}  // namespace

void NormalizeConfigForm(QWidget * widget)
{
  // Plugin forms stretch their spin boxes across the whole panel, leaving a
  // couple of digits floating in a wide box.  A spin box's size hint already
  // fits its range, decimals, prefix and suffix, so hold it to that width.
  // The forms also mix arrow and +/- buttons; use arrows throughout.
  const QList<QAbstractSpinBox *> spin_boxes = widget->findChildren<QAbstractSpinBox *>();
  for (QAbstractSpinBox * spin_box : spin_boxes) {
    QSizePolicy policy = spin_box->sizePolicy();
    policy.setHorizontalPolicy(QSizePolicy::Fixed);
    spin_box->setSizePolicy(policy);
    spin_box->setButtonSymbols(QAbstractSpinBox::UpDownArrows);
  }

  // Color swatches are capped at 24 pixels in the forms; let them keep pace
  // with larger fonts.
  const int swatch = std::max(24, IconWidget::IconSize() + 8);
  const QList<ColorButton *> swatches = widget->findChildren<ColorButton *>();
  for (ColorButton * button : swatches) {
    button->setMaximumSize(swatch, swatch);
    button->setMinimumSize(swatch, swatch);
  }

  CollapseVerticalSpacers(widget->layout());
  const QList<QWidget *> children = widget->findChildren<QWidget *>();
  for (QWidget * child : children) {
    CollapseVerticalSpacers(child->layout());
  }
}

ConfigItem::ConfigItem(QWidget * parent, Qt::WindowFlags flags)
: QWidget(parent, flags),
  item_(nullptr),
  visible_(true)
{
  ui_.setupUi(this);

  // Let the style sheet paint the backgrounds of these custom widgets.
  setAttribute(Qt::WA_StyledBackground, true);
  ui_.header->setAttribute(Qt::WA_StyledBackground, true);
  setProperty("selected", false);
  setProperty("collapsed", false);
  updateCollapseIndicator();

  edit_name_action_ = new QAction("Edit Name", this);
  edit_name_action_->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_R));
  remove_item_action_ = new QAction("Remove", this);
  remove_item_action_->setIcon(Theme::Icon(ThemeIcon::Remove));
  remove_item_action_->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_X));
  duplicate_item_action_ = new QAction("Duplicate", this);
  duplicate_item_action_->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_D));

  connect(edit_name_action_, SIGNAL(triggered()), this, SLOT(EditName()));
  connect(duplicate_item_action_, SIGNAL(triggered()), this, SLOT(Duplicate()));
  connect(remove_item_action_, SIGNAL(triggered()), this, SLOT(Remove()));
}

void ConfigItem::ToggleDraw(bool toggled)
{
  if (visible_ != toggled) {
    visible_ = toggled;
    if (ui_.show->isChecked() != toggled) {
      ui_.show->setChecked(toggled);
    }

    Q_EMIT ToggledDraw(item_, toggled);
  }
}

void ConfigItem::contextMenuEvent(QContextMenuEvent * event)
{
  QMenu menu(this);
  menu.addAction(edit_name_action_);
  menu.addAction(duplicate_item_action_);
  menu.addAction(remove_item_action_);
  menu.exec(event->globalPos());
}

void ConfigItem::SetName(QString name)
{
  name_ = name;
  full_label_text_ = type_ + " (" + name_ + ")";
  updateNameLabel();
}

void ConfigItem::SetType(QString type)
{
  type_ = type;
  full_label_text_ = type_ + " (" + name_ + ")";
  updateNameLabel();
}

void ConfigItem::resizeEvent(QResizeEvent * event)
{
  QWidget::resizeEvent(event);
  updateNameLabel();
}

void ConfigItem::updateNameLabel()
{
  if (ui_.namelabel->width() <= 0) {
    return;
  }
  QFontMetrics fm(ui_.namelabel->font());
  ui_.namelabel->setText(
    fm.elidedText(full_label_text_, Qt::ElideRight, ui_.namelabel->width()));
}

void ConfigItem::SetWidget(QWidget * widget)
{
  ui_.label->hide();
  // Plugins' config forms come with assorted margins (some none at all); give
  // them all the same inset so the cards line up.
  if (widget->layout() != nullptr) {
    widget->layout()->setContentsMargins(8, 6, 8, 8);
  }
  NormalizeConfigForm(widget);
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

  if (ok && !text.isEmpty()) {
    SetName(text);
  }
}

void ConfigItem::Duplicate()
{
  Q_EMIT DuplicateRequest(item_);
}

void ConfigItem::Remove()
{
  Q_EMIT RemoveRequest(item_);
}

void ConfigItem::Hide()
{
  if (!ui_.content->isHidden()) {
    ui_.content->hide();
  } else {
    ui_.content->show();
  }
  setProperty("collapsed", ui_.content->isHidden());
  updateCollapseIndicator();
  restyle();

  Q_EMIT UpdateSizeHint();
}

void ConfigItem::SetSelected(bool selected)
{
  if (property("selected").toBool() != selected) {
    setProperty("selected", selected);
    restyle();
  }
}

void ConfigItem::changeEvent(QEvent * event)
{
  QWidget::changeEvent(event);
  if (event->type() == QEvent::PaletteChange || event->type() == QEvent::StyleChange) {
    // The chevron is a pixmap rendered in the palette's colors.
    updateCollapseIndicator();
  }
}

void ConfigItem::updateCollapseIndicator()
{
  const QIcon icon = Theme::Icon(
    ui_.content->isHidden() ? ThemeIcon::ChevronRight : ThemeIcon::ChevronDown);
  const int side = IconWidget::IconSize();
  ui_.signlabel->setFixedSize(side + 4, side + 4);
  ui_.signlabel->setPixmap(icon.pixmap(QSize(side, side)));
}

void ConfigItem::restyle()
{
  // Style sheet rules that match on dynamic properties are only re-evaluated
  // when a widget is re-polished.
  for (QWidget * widget : {static_cast<QWidget *>(this), static_cast<QWidget *>(ui_.header)}) {
    widget->style()->unpolish(widget);
    widget->style()->polish(widget);
    widget->update();
  }
}
}   // namespace mapviz
