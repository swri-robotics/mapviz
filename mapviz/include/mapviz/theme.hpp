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

#ifndef MAPVIZ__THEME_HPP_
#define MAPVIZ__THEME_HPP_

#include <QFont>
#include <QIcon>
#include <QPalette>
#include <QString>
#include <QWidget>

namespace mapviz
{
/// Which color scheme the application should use.
enum class ThemeMode
{
  System,  ///< Follow the desktop's light/dark preference
  Light,
  Dark
};

/// Icons that are painted at runtime in the current palette's colors, so they
/// stay sharp on high-DPI displays and legible in both color schemes.
enum class ThemeIcon
{
  Pin,
  Screenshot,
  Record,
  Pause,
  Stop,
  Recenter,
  Remove,
  ChevronRight,
  ChevronDown
};

/**
 * Mapviz's look and feel: a Fusion-based widget style with light and dark
 * palettes and a style sheet whose colors are derived from the palette.
 *
 * Nothing in here affects behavior; it only changes how widgets are drawn.
 */
class Theme
{
public:
  /// Applies the theme to the whole application.  Use this when mapviz owns
  /// the QApplication.
  static void Apply(ThemeMode mode);

  /// Applies only the style sheet to a widget and its children, using colors
  /// derived from that widget's palette.  Use this when mapviz is embedded in
  /// a host application (rqt) that owns the application-wide look.
  static void ApplyTo(QWidget * widget);

  /// The mode most recently passed to Apply().
  static ThemeMode CurrentMode();

  /// Reads / writes the user's preferred mode in the per-user settings.
  static ThemeMode SavedMode();
  static void SaveMode(ThemeMode mode);

  /// True if the desktop is using a dark color scheme.
  static bool SystemPrefersDark();

  static QPalette Palette(bool dark);

  /// The style sheet with every color resolved against the given palette, and
  /// every size against the given font.
  static QString StyleSheet(const QPalette & palette, const QFont & font = QFont());

  static QIcon Icon(ThemeIcon icon);

  static QString ModeToString(ThemeMode mode);
  static ThemeMode ModeFromString(const QString & text);
};
}  // namespace mapviz

#endif  // MAPVIZ__THEME_HPP_
