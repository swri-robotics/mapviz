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

#ifndef MAPVIZ__STATUS_LABEL_HPP_
#define MAPVIZ__STATUS_LABEL_HPP_

#include <QColor>
#include <QGuiApplication>
#include <QLabel>
#include <QPalette>
#include <QVariant>

#include <string>

namespace mapviz
{
/// Severity of the message shown in a plugin's status label.
enum class StatusLevel
{
  Info = 0,
  Warning = 1,
  Error = 2
};

/// Dynamic property that records a status label's severity, so that its color
/// can be refreshed when the application's color scheme changes.
constexpr char STATUS_LEVEL_PROPERTY[] = "mapviz_status_level";

/**
 * Returns the text color for a status message that is readable on either a
 * light or a dark background.
 */
inline QColor StatusColor(StatusLevel level, bool dark_background)
{
  switch (level) {
    case StatusLevel::Error:
      return dark_background ? QColor(0xff, 0x6b, 0x6b) : QColor(0xc6, 0x28, 0x28);
    case StatusLevel::Warning:
      return dark_background ? QColor(0xe3, 0xb3, 0x41) : QColor(0x85, 0x59, 0x00);
    case StatusLevel::Info:
    default:
      return dark_background ? QColor(0x56, 0xd3, 0x64) : QColor(0x1a, 0x7f, 0x37);
  }
}

/**
 * The text to show in a status label for a message: status text is a short
 * phrase, so a single trailing period is dropped.  An ellipsis ("Sending
 * route...") is kept, since it signals that something is in progress.
 */
inline std::string StatusText(const std::string & message)
{
  const size_t size = message.size();
  if (size > 0 && message[size - 1] == '.' && (size < 2 || message[size - 2] != '.')) {
    return message.substr(0, size - 1);
  }
  return message;
}

/**
 * Colors a status label for the given severity.
 *
 * The application palette decides between the light and dark variants rather
 * than the label's own palette, because plugins commonly force a white window
 * color onto their config widget regardless of the active theme.
 */
inline void SetStatusLevel(QLabel * label, StatusLevel level)
{
  const QPalette app_palette = QGuiApplication::palette();
  const bool dark = app_palette.color(QPalette::Window).lightness() <
    app_palette.color(QPalette::WindowText).lightness();
  const QColor color = StatusColor(level, dark);

  QPalette p(label->palette());
  p.setColor(QPalette::Text, color);
  p.setColor(QPalette::WindowText, color);
  label->setPalette(p);
  label->setProperty(STATUS_LEVEL_PROPERTY, static_cast<int>(level));
}
}  // namespace mapviz

#endif  // MAPVIZ__STATUS_LABEL_HPP_
