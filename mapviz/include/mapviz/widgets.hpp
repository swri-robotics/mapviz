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

#ifndef MAPVIZ__WIDGETS_HPP_
#define MAPVIZ__WIDGETS_HPP_

// QT libraries
#include <QApplication>
#include <QFont>
#include <QFontMetrics>
#include <QWidget>
#include <QListWidget>
#include <QListWidgetItem>
#include <QLabel>
#include <QMouseEvent>
#include <QDropEvent>
#include <QPainter>
#include <QPixmap>

#include <algorithm>
#include <cmath>

namespace mapviz
{
class PluginConfigList : public QListWidget
{
  Q_OBJECT

public:
  explicit PluginConfigList(QWidget * parent = nullptr)
  : QListWidget(parent) {}
  PluginConfigList() = default;

  void UpdateIndices()
  {
    for (int i = 0; i < count(); i++) {
      item(i)->setData(Qt::UserRole, QVariant((static_cast<float>(i))));
    }
  }

Q_SIGNALS:
  void ItemsMoved();

protected:
  void dropEvent(QDropEvent * event) override
  {
    QListWidget::dropEvent(event);

    UpdateIndices();

    Q_EMIT ItemsMoved();
  }
};

class PluginConfigListItem : public QListWidgetItem
{
public:
  explicit PluginConfigListItem(QListWidget * parent = nullptr)
  : QListWidgetItem(parent) {}

  bool operator<(const QListWidgetItem & other) const override
  {
    return data(Qt::UserRole).toFloat() < other.data(Qt::UserRole).toFloat();
  }
};

class SingleClickLabel : public QLabel
{
  Q_OBJECT

public:
  explicit SingleClickLabel(QWidget * parent = 0, Qt::WindowFlags flags = Qt::WindowFlags())
  : QLabel(parent, flags) {}

  ~SingleClickLabel() override = default;

Q_SIGNALS:
  void Clicked();

protected:
  void mousePressEvent(QMouseEvent *) override
  {
    Q_EMIT Clicked();
  }
};

class DoubleClickWidget : public QWidget
{
  Q_OBJECT

public:
  explicit DoubleClickWidget(QWidget * parent = 0, Qt::WindowFlags flags = Qt::WindowFlags())
  : QWidget(parent, flags) {}

  ~DoubleClickWidget() override = default;

Q_SIGNALS:
  void DoubleClicked();
  void RightClicked();

protected:
  void mouseDoubleClickEvent(QMouseEvent * event) override
  {
    if (event->button() == Qt::LeftButton) {
      Q_EMIT DoubleClicked();
    }
  }

  void mouseReleaseEvent(QMouseEvent * event) override
  {
    if (event->button() == Qt::RightButton) {
      Q_EMIT RightClicked();
    }
  }
};

class IconWidget : public QWidget
{
  Q_OBJECT

public:
  explicit IconWidget(QWidget * parent = nullptr, Qt::WindowFlags flags = Qt::WindowFlags())
  : QWidget(parent, flags)
  {
    pixmap_ = QPixmap(16, 16);
    pixmap_.fill(Qt::transparent);
  }

  ~IconWidget() override = default;

  /// Size, in logical pixels, that header icons are shown at: as tall as the
  /// bold display title beside them, and never smaller than 16.
  static int IconSize()
  {
    QFont font = QApplication::font();
    font.setBold(true);
    return std::max(16, QFontMetrics(font).ascent() + 2);
  }

  /**
   * A transparent canvas for a plugin's header icon.  Plugins draw on it in
   * 16x16 units as they always have; it carries enough device pixels for the
   * result to stay sharp at the display's scale and at IconSize().
   */
  static QPixmap CreatePixmap()
  {
    const qreal ratio = qApp ? qApp->devicePixelRatio() : 1.0;
    const qreal scale = IconSize() / 16.0 * ratio;
    const int side = static_cast<int>(std::ceil(16 * scale));
    QPixmap pixmap(side, side);
    pixmap.setDevicePixelRatio(scale);
    pixmap.fill(Qt::transparent);
    return pixmap;
  }

  void SetPixmap(QPixmap pixmap)
  {
    pixmap_ = pixmap;
    update();
  }

  QSize sizeHint() const override
  {
    const int side = IconSize() + 4;
    return QSize(side, side);
  }

protected:
  void paintEvent(QPaintEvent *) override
  {
    // No background fill: the icon sits on whatever its parent paints.
    QPainter painter(this);
    painter.setRenderHint(QPainter::SmoothPixmapTransform, true);

    // Pixmaps from CreatePixmap() are drawn at IconSize(); a plain one that a
    // plugin made itself is drawn at its own size, as before.
    QSize size = pixmap_.size();
    if (pixmap_.devicePixelRatio() != 1.0) {
      const int side = IconSize();
      size = QSize(side, side);
    }
    const QRect target(
      (width() - size.width()) / 2, (height() - size.height()) / 2,
      size.width(), size.height());
    painter.drawPixmap(target, pixmap_);
  }

  QPixmap pixmap_;
};
}   // namespace mapviz

#endif  // MAPVIZ__WIDGETS_HPP_
