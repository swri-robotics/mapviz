// *****************************************************************************
//
// Copyright (c) 2026, Southwest Research Institute® (SwRI®)
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

#include "mapviz/theme.hpp"

#include <QApplication>
#include <QBrush>
#include <QByteArray>
#include <QColor>
#include <QCoreApplication>
#include <QDebug>
#include <QFile>
#include <QFontMetrics>
#include <QGuiApplication>
#include <QIODevice>
#include <QIconEngine>
#include <QLabel>
#include <QObject>
#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QPixmap>
#include <QPoint>
#include <QPointF>
#include <QProxyStyle>
#include <QRect>
#include <QRectF>
#include <QSettings>
#include <QStringAlgorithms>
#include <QStyle>
#include <QStyleFactory>
#include <QStyleHints>
#include <QStyleOption>
#include <QVariant>
#include <QWidgetList>
#include <Qt>
#include <QtGlobal>

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <iterator>
#include <utility>
#include <vector>

#include "mapviz/status_label.hpp"

namespace mapviz
{
namespace
{
const char SETTINGS_ORGANIZATION[] = "mapviz";
const char SETTINGS_APPLICATION[] = "mapviz";
const char SETTINGS_THEME_KEY[] = "ui/theme";

// Fixed rather than taken from the palette: "recording" should read as red in
// either color scheme.
const QColor RECORD_COLOR(0xe5, 0x48, 0x4d);

ThemeMode & CurrentModeRef()
{
  static ThemeMode mode = ThemeMode::System;
  return mode;
}

/// The palette the application started with, i.e. the desktop's.  It has to be
/// captured before Apply() replaces it, since it is the only hint about the
/// desktop's color scheme that older Qt versions offer.
const QPalette & DesktopPalette()
{
  static const QPalette palette = QGuiApplication::palette();
  return palette;
}

bool IsDark(const QPalette & palette)
{
  return palette.color(QPalette::Window).lightness() <
         palette.color(QPalette::WindowText).lightness();
}

/// Blends two colors; amount = 0 gives `from`, amount = 1 gives `to`.
QColor Mix(const QColor & from, const QColor & to, double amount)
{
  const double keep = 1.0 - amount;
  return QColor(
    qRound(from.red() * keep + to.red() * amount),
    qRound(from.green() * keep + to.green() * amount),
    qRound(from.blue() * keep + to.blue() * amount));
}

/// Paints one of the ThemeIcons on a 24x24 design grid scaled to fit `rect`.
void PaintIcon(
  QPainter * painter, const QRect & rect, ThemeIcon icon,
  const QColor & color, bool on)
{
  painter->save();
  painter->setRenderHint(QPainter::Antialiasing, true);
  painter->translate(rect.topLeft());
  painter->scale(rect.width() / 24.0, rect.height() / 24.0);

  QPen pen(color, 1.8, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
  painter->setPen(pen);
  painter->setBrush(Qt::NoBrush);

  switch (icon) {
    case ThemeIcon::Pin:
      {
        // Upright and filled while pinned; tilted and hollow while unpinned.
        if (!on) {
          painter->translate(12.0, 12.0);
          painter->rotate(45.0);
          painter->translate(-12.0, -12.0);
        }
        QPainterPath head;
        head.moveTo(9.0, 4.0);
        head.lineTo(15.0, 4.0);
        head.lineTo(14.5, 9.5);
        head.lineTo(17.5, 13.5);
        head.lineTo(6.5, 13.5);
        head.lineTo(9.5, 9.5);
        head.closeSubpath();
        if (on) {
          painter->setBrush(color);
        }
        painter->drawPath(head);
        painter->drawLine(QPointF(12.0, 13.5), QPointF(12.0, 20.5));
        break;
      }
    case ThemeIcon::Screenshot:
      {
        QPainterPath body;
        body.moveTo(8.0, 7.0);
        body.lineTo(9.5, 4.5);
        body.lineTo(14.5, 4.5);
        body.lineTo(16.0, 7.0);
        painter->drawPath(body);
        painter->drawRoundedRect(QRectF(3.0, 7.0, 18.0, 12.5), 2.0, 2.0);
        painter->drawEllipse(QPointF(12.0, 13.2), 3.3, 3.3);
        break;
      }
    case ThemeIcon::Record:
      painter->setPen(Qt::NoPen);
      painter->setBrush(color);
      painter->drawEllipse(QPointF(12.0, 12.0), 6.5, 6.5);
      break;
    case ThemeIcon::Pause:
      painter->setPen(Qt::NoPen);
      painter->setBrush(color);
      painter->drawRoundedRect(QRectF(6.5, 5.5, 4.0, 13.0), 1.0, 1.0);
      painter->drawRoundedRect(QRectF(13.5, 5.5, 4.0, 13.0), 1.0, 1.0);
      break;
    case ThemeIcon::Stop:
      painter->setPen(Qt::NoPen);
      painter->setBrush(color);
      painter->drawRoundedRect(QRectF(6.0, 6.0, 12.0, 12.0), 1.5, 1.5);
      break;
    case ThemeIcon::Recenter:
      painter->drawEllipse(QPointF(12.0, 12.0), 5.5, 5.5);
      painter->drawLine(QPointF(12.0, 2.5), QPointF(12.0, 6.5));
      painter->drawLine(QPointF(12.0, 17.5), QPointF(12.0, 21.5));
      painter->drawLine(QPointF(2.5, 12.0), QPointF(6.5, 12.0));
      painter->drawLine(QPointF(17.5, 12.0), QPointF(21.5, 12.0));
      painter->setPen(Qt::NoPen);
      painter->setBrush(color);
      painter->drawEllipse(QPointF(12.0, 12.0), 1.6, 1.6);
      break;
    case ThemeIcon::Remove:
      painter->drawLine(QPointF(7.0, 7.0), QPointF(17.0, 17.0));
      painter->drawLine(QPointF(17.0, 7.0), QPointF(7.0, 17.0));
      break;
    case ThemeIcon::ChevronRight:
      {
        QPainterPath path;
        path.moveTo(9.5, 6.5);
        path.lineTo(15.0, 12.0);
        path.lineTo(9.5, 17.5);
        painter->drawPath(path);
        break;
      }
    case ThemeIcon::ChevronDown:
      {
        QPainterPath path;
        path.moveTo(6.5, 9.5);
        path.lineTo(12.0, 15.0);
        path.lineTo(17.5, 9.5);
        painter->drawPath(path);
        break;
      }
  }

  painter->restore();
}

/// An icon engine that paints in the application palette's colors each time it
/// is asked, so icons follow theme changes without having to be re-created.
class PaintedIconEngine : public QIconEngine
{
public:
  explicit PaintedIconEngine(ThemeIcon icon)
  : icon_(icon) {}

  void paint(
    QPainter * painter, const QRect & rect, QIcon::Mode mode,
    QIcon::State state) override
  {
    const QPalette palette = QGuiApplication::palette();
    QColor color = palette.color(
      mode == QIcon::Disabled ? QPalette::Disabled : QPalette::Active,
      QPalette::ButtonText);
    if (icon_ == ThemeIcon::Record) {
      color = RECORD_COLOR;
      if (mode == QIcon::Disabled) {
        color.setAlphaF(0.4);
      }
    }
    PaintIcon(painter, rect, icon_, color, state == QIcon::On);
  }

  QPixmap pixmap(const QSize & size, QIcon::Mode mode, QIcon::State state) override
  {
    QPixmap pixmap(size);
    pixmap.fill(Qt::transparent);
    QPainter painter(&pixmap);
    paint(&painter, QRect(QPoint(0, 0), size), mode, state);
    return pixmap;
  }

  QIconEngine * clone() const override
  {
    return new PaintedIconEngine(icon_);
  }

private:
  ThemeIcon icon_;
};

/**
 * Fusion with clearer check boxes and radio buttons.
 *
 * Fusion outlines indicators in a shade of the window color, which all but
 * disappears on a dark background.  These are drawn from the palette instead:
 * a visible outline when off, and an accent fill with a contrasting mark when
 * on, in both color schemes.  Everything else is left to Fusion.
 */
class MapvizStyle : public QProxyStyle
{
public:
  MapvizStyle()
  : QProxyStyle(QStyleFactory::create("Fusion")) {}

  void drawPrimitive(
    PrimitiveElement element, const QStyleOption * option, QPainter * painter,
    const QWidget * widget) const override
  {
    if (element == PE_IndicatorCheckBox || element == PE_IndicatorRadioButton) {
      DrawIndicator(element == PE_IndicatorRadioButton, option, painter);
      return;
    }
    QProxyStyle::drawPrimitive(element, option, painter, widget);
  }

private:
  static void DrawIndicator(bool radio, const QStyleOption * option, QPainter * painter)
  {
    const QPalette & palette = option->palette;
    const QStyle::State state = option->state;
    const bool enabled = state & QStyle::State_Enabled;
    const bool checked = state & (QStyle::State_On | QStyle::State_NoChange);
    const bool partial = state & QStyle::State_NoChange;
    const bool hover = enabled && (state & QStyle::State_MouseOver);
    const bool sunken = enabled && (state & QStyle::State_Sunken);

    const QColor text = palette.color(QPalette::Text);
    const QColor base = palette.color(QPalette::Active, QPalette::Base);
    const QColor accent = palette.color(QPalette::Active, QPalette::Highlight);

    QColor fill = enabled ? base : palette.color(QPalette::Window);
    QColor border = Mix(base, text, enabled ? 0.45 : 0.22);
    if (checked) {
      fill = enabled ? (sunken ? accent.darker(115) : accent) : Mix(base, text, 0.25);
      border = fill;
    } else if (sunken) {
      fill = Mix(base, text, 0.1);
    }
    if (hover) {
      border = checked ? accent.lighter(120) : accent;
    }
    const QColor mark = enabled ?
      palette.color(QPalette::Active, QPalette::HighlightedText) :
      palette.color(QPalette::Window);

    const QRectF rect = option->rect;
    const double side = std::floor(std::min(rect.width(), rect.height()));
    const QRectF box(
      rect.x() + std::floor((rect.width() - side) / 2.0) + 0.5,
      rect.y() + std::floor((rect.height() - side) / 2.0) + 0.5,
      side - 1.0, side - 1.0);

    painter->save();
    painter->setRenderHint(QPainter::Antialiasing, true);
    painter->setPen(QPen(border, 1.0));
    painter->setBrush(fill);
    if (radio) {
      painter->drawEllipse(box);
      if (checked) {
        painter->setPen(Qt::NoPen);
        painter->setBrush(mark);
        painter->drawEllipse(box.center(), side * 0.2, side * 0.2);
      }
    } else {
      const double radius = std::max(2.0, side * 0.18);
      painter->drawRoundedRect(box, radius, radius);
      if (checked) {
        QPen pen(mark, std::max(1.5, side * 0.12), Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        painter->setPen(pen);
        painter->setBrush(Qt::NoBrush);
        auto at = [&box](double x, double y) {
            return QPointF(box.x() + box.width() * x, box.y() + box.height() * y);
          };
        if (partial) {
          painter->drawLine(at(0.27, 0.5), at(0.73, 0.5));
        } else {
          QPainterPath tick;
          tick.moveTo(at(0.25, 0.52));
          tick.lineTo(at(0.43, 0.7));
          tick.lineTo(at(0.76, 0.32));
          painter->drawPath(tick);
        }
      }
    }
    painter->restore();
  }
};

/// Status labels remember their severity; recolor them for the new scheme.
void RefreshStatusLabels()
{
  const QWidgetList widgets = QApplication::allWidgets();
  for (QWidget * widget : widgets) {
    QLabel * label = qobject_cast<QLabel *>(widget);
    if (label == nullptr) {
      continue;
    }
    const QVariant level = label->property(STATUS_LEVEL_PROPERTY);
    if (level.isValid()) {
      SetStatusLevel(label, static_cast<StatusLevel>(level.toInt()));
    }
  }
}
}  // namespace

void Theme::Apply(ThemeMode mode)
{
  QApplication * app = qobject_cast<QApplication *>(QCoreApplication::instance());
  if (app == nullptr) {
    return;
  }

  static bool initialized = false;
  if (!initialized) {
    initialized = true;
    DesktopPalette();
    // Fusion looks the same on every desktop and draws entirely from the
    // palette, which is what lets one palette swap restyle everything.
    QApplication::setStyle(new MapvizStyle());
#if QT_VERSION >= 0x060500
    QObject::connect(
      QGuiApplication::styleHints(), &QStyleHints::colorSchemeChanged, app,
      []() {
        if (CurrentModeRef() == ThemeMode::System) {
          Theme::Apply(ThemeMode::System);
        }
      });
#endif
  }

  CurrentModeRef() = mode;
  const bool dark =
    mode == ThemeMode::Dark || (mode == ThemeMode::System && SystemPrefersDark());
  const QPalette palette = Palette(dark);
  QApplication::setPalette(palette);
  app->setStyleSheet(StyleSheet(palette));
  RefreshStatusLabels();
}

void Theme::ApplyTo(QWidget * widget)
{
  if (widget != nullptr) {
    widget->setStyleSheet(StyleSheet(widget->palette(), widget->font()));
  }
}

ThemeMode Theme::CurrentMode()
{
  return CurrentModeRef();
}

ThemeMode Theme::SavedMode()
{
  QSettings settings(SETTINGS_ORGANIZATION, SETTINGS_APPLICATION);
  return ModeFromString(settings.value(SETTINGS_THEME_KEY).toString());
}

void Theme::SaveMode(ThemeMode mode)
{
  QSettings settings(SETTINGS_ORGANIZATION, SETTINGS_APPLICATION);
  settings.setValue(SETTINGS_THEME_KEY, ModeToString(mode));
}

bool Theme::SystemPrefersDark()
{
#if QT_VERSION >= 0x060500
  const Qt::ColorScheme scheme = QGuiApplication::styleHints()->colorScheme();
  if (scheme != Qt::ColorScheme::Unknown) {
    return scheme == Qt::ColorScheme::Dark;
  }
#endif
  return IsDark(DesktopPalette());
}

QPalette Theme::Palette(bool dark)
{
  struct Colors
  {
    QColor window, text, base, alternate_base, button, highlight, link;
    QColor light, midlight, mid, dark, shadow, tooltip, tooltip_text;
  };
  const Colors c = dark ?
    Colors{
    QColor(0x1f, 0x22, 0x28), QColor(0xe3, 0xe6, 0xeb), QColor(0x30, 0x34, 0x3d),
    QColor(0x37, 0x3c, 0x46), QColor(0x33, 0x38, 0x42), QColor(0x3d, 0x7e, 0xea),
    QColor(0x6e, 0xa8, 0xfe),
    QColor(0x4a, 0x50, 0x5c), QColor(0x39, 0x3e, 0x48), QColor(0x33, 0x37, 0x40),
    QColor(0x14, 0x16, 0x1a), QColor(0x0b, 0x0c, 0x0e), QColor(0x0f, 0x11, 0x14),
    QColor(0xe3, 0xe6, 0xeb)} :
  Colors{
    QColor(0xf1, 0xf3, 0xf6), QColor(0x1d, 0x21, 0x29), QColor(0xff, 0xff, 0xff),
    QColor(0xe9, 0xec, 0xf1), QColor(0xfb, 0xfc, 0xfd), QColor(0x2f, 0x6f, 0xde),
    QColor(0x1f, 0x5f, 0xcf),
    QColor(0xff, 0xff, 0xff), QColor(0xe6, 0xe9, 0xee), QColor(0xc4, 0xc9, 0xd2),
    QColor(0x98, 0x9f, 0xab), QColor(0x5f, 0x66, 0x73), QColor(0x1d, 0x21, 0x29),
    QColor(0xf4, 0xf5, 0xf7)};

  QPalette palette;
  palette.setColor(QPalette::Window, c.window);
  palette.setColor(QPalette::WindowText, c.text);
  palette.setColor(QPalette::Base, c.base);
  palette.setColor(QPalette::AlternateBase, c.alternate_base);
  palette.setColor(QPalette::Text, c.text);
  palette.setColor(QPalette::Button, c.button);
  palette.setColor(QPalette::ButtonText, c.text);
  palette.setColor(QPalette::BrightText, Qt::white);
  palette.setColor(QPalette::Highlight, c.highlight);
  palette.setColor(QPalette::HighlightedText, Qt::white);
  palette.setColor(QPalette::Link, c.link);
  palette.setColor(QPalette::LinkVisited, c.link);
  palette.setColor(QPalette::Light, c.light);
  palette.setColor(QPalette::Midlight, c.midlight);
  palette.setColor(QPalette::Mid, c.mid);
  palette.setColor(QPalette::Dark, c.dark);
  palette.setColor(QPalette::Shadow, c.shadow);
  palette.setColor(QPalette::ToolTipBase, c.tooltip);
  palette.setColor(QPalette::ToolTipText, c.tooltip_text);
  palette.setColor(QPalette::PlaceholderText, Mix(c.text, c.window, 0.45));

  const QColor disabled_text = Mix(c.text, c.window, 0.58);
  for (QPalette::ColorRole role :
    {QPalette::WindowText, QPalette::Text, QPalette::ButtonText})
  {
    palette.setColor(QPalette::Disabled, role, disabled_text);
  }
  palette.setColor(QPalette::Disabled, QPalette::Base, c.window);
  palette.setColor(QPalette::Disabled, QPalette::Button, c.window);
  palette.setColor(QPalette::Disabled, QPalette::Highlight, Mix(c.window, c.text, 0.25));
  palette.setColor(QPalette::Disabled, QPalette::HighlightedText, disabled_text);
  return palette;
}

QString Theme::StyleSheet(const QPalette & palette, const QFont & font)
{
  QFile file(":/style/mapviz.qss");
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    return QString();
  }
  QString sheet = QString::fromUtf8(file.readAll());

  const bool dark = IsDark(palette);
  const QColor window = palette.color(QPalette::Window);
  const QColor text = palette.color(QPalette::WindowText);
  const QColor button = palette.color(QPalette::Button);
  const QColor accent = palette.color(QPalette::Highlight);

  // Longer names first, so that replacing "@border@" can never clip a name
  // that merely starts the same way.
  const std::vector<std::pair<QString, QColor>> tokens = {
    {"header_selected", Mix(window, accent, dark ? 0.42 : 0.22)},
    {"button_pressed", Mix(button, text, 0.16)},
    {"border_strong", Mix(window, text, dark ? 0.38 : 0.34)},
    {"button_hover", Mix(button, text, 0.07)},
    {"header_hover", Mix(window, text, dark ? 0.17 : 0.13)},
    {"tooltip_text", palette.color(QPalette::ToolTipText)},
    {"scroll_hover", Mix(window, text, 0.45)},
    {"tooltip", palette.color(QPalette::ToolTipBase)},
    {"pressed", Mix(window, text, 0.18)},
    {"header", Mix(window, text, dark ? 0.12 : 0.08)},
    {"border", Mix(window, text, dark ? 0.22 : 0.18)},
    {"button", button},
    {"base", palette.color(QPalette::Base)},
    {"handle", dark ? Mix(window, text, 0.78) : palette.color(QPalette::Base)},
    {"window", window},
    {"scroll", Mix(window, text, 0.28)},
    {"accent", accent},
    {"hover", Mix(window, text, 0.09)},
    // A card sits on the window: on a light theme it is the (white) base
    // color, on a dark one a slightly raised shade of the window.
    {"card", dark ? Mix(window, text, 0.045) : palette.color(QPalette::Base)},
  };
  for (const auto & token : tokens) {
    sheet.replace("@" + token.first + "@", token.second.name());
  }

  // The display visibility check box, sized to the bold display title beside
  // it: as tall as the title's ascenders, plus a little for its frame.
  QFont bold = font;
  bold.setBold(true);
  const QFontMetrics metrics(bold);
  sheet.replace("@indicator@", QString::number(metrics.ascent() + 2));
  return sheet;
}

QIcon Theme::Icon(ThemeIcon icon)
{
  return QIcon(new PaintedIconEngine(icon));
}

QString Theme::ModeToString(ThemeMode mode)
{
  switch (mode) {
    case ThemeMode::Light:
      return "light";
    case ThemeMode::Dark:
      return "dark";
    case ThemeMode::System:
    default:
      return "system";
  }
}

ThemeMode Theme::ModeFromString(const QString & text)
{
  const QString lowered = text.trimmed().toLower();
  if (lowered == "light") {
    return ThemeMode::Light;
  }
  if (lowered == "dark") {
    return ThemeMode::Dark;
  }
  return ThemeMode::System;
}
}  // namespace mapviz
