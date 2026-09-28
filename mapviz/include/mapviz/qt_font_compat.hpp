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
#ifndef MAPVIZ__QT_FONT_COMPAT_HPP_
#define MAPVIZ__QT_FONT_COMPAT_HPP_

#include <QFont>
#include <QString>
#include <QStringList>
#include <QtGlobal>

namespace mapviz
{
#if QT_VERSION < 0x060000
/// Converts a Qt 6 font weight, from 1 to 1000, to the nearest Qt 5 one.
inline int Qt5FontWeight(int qt6_weight)
{
  if (qt6_weight <= 150) {return QFont::Thin;}
  if (qt6_weight <= 250) {return QFont::ExtraLight;}
  if (qt6_weight <= 350) {return QFont::Light;}
  if (qt6_weight <= 450) {return QFont::Normal;}
  if (qt6_weight <= 550) {return QFont::Medium;}
  if (qt6_weight <= 650) {return QFont::DemiBold;}
  if (qt6_weight <= 750) {return QFont::Bold;}
  if (qt6_weight <= 850) {return QFont::ExtraBold;}
  return QFont::Black;
}
#endif

/**
 * Reads a font saved with QFont::toString(), by this or another Qt version.
 *
 * Qt 6 saves fonts with more fields, and weights on a different scale, than
 * Qt 5 can read, so a config saved on Qt 6 would lose its fonts on Qt 5.  On
 * Qt 5, the fields the two versions share are read from a Qt 6 font instead.
 *
 * @param[in]  description A font saved with QFont::toString().
 * @param[out] font        The font, if it could be read; otherwise unchanged.
 * @return Whether the font could be read.
 */
inline bool FontFromString(const QString & description, QFont & font)
{
  QFont read = font;
  if (read.fromString(description)) {
    font = read;
    return true;
  }

#if QT_VERSION < 0x060000
  // Qt 6 writes at least 16 fields, the first 9 in the same order as Qt 5.
  const QStringList fields = description.split(QLatin1Char(','));
  if (fields.size() < 16 || fields[0].isEmpty()) {
    return false;
  }

  QFont converted;
  converted.setFamily(fields[0]);
  if (fields[1].toDouble() > 0.0) {
    converted.setPointSizeF(fields[1].toDouble());
  }
  if (fields[2].toInt() > 0) {
    converted.setPixelSize(fields[2].toInt());
  }
  converted.setStyleHint(static_cast<QFont::StyleHint>(fields[3].toInt()));
  converted.setWeight(Qt5FontWeight(fields[4].toInt()));
  converted.setStyle(static_cast<QFont::Style>(fields[5].toInt()));
  converted.setUnderline(fields[6].toInt() != 0);
  converted.setStrikeOut(fields[7].toInt() != 0);
  converted.setFixedPitch(fields[8].toInt() != 0);
  font = converted;
  return true;
#else
  return false;
#endif
}
}  // namespace mapviz

#endif  // MAPVIZ__QT_FONT_COMPAT_HPP_
