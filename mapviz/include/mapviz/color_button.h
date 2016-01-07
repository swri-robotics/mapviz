// *****************************************************************************
//
// Copyright (c) 2015, Southwest Research Institute® (SwRI®)
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
// ARE DISCLAIMED. IN NO EVENT SHALL Southwest Research Institute® BE LIABLE 
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
// OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.
//
// *****************************************************************************
#ifndef MAPVIZ_COLOR_BUTTON_H_
#define MAPVIZ_COLOR_BUTTON_H_

#include <QPushButton>
#include <QColor>

namespace mapviz
{
/**
 * The ColorButton widget provides a color display that the user can
 * click on to select a new color.  You can use this widget in Qt
 * Designer by placing a QPushButton and promoting it to a custom
 * widget.  You have to setup the promoted widget once in each .ui file:
 *
 * Base class name: QPushButton
 * Promoted class name: mapviz::ColorButton
 * Include file name: mapviz/color_button.h
 * Global Include: True
 */
class ColorButton : public QPushButton
{
  Q_OBJECT;

  QColor color_;
  
 public:
  ColorButton(QWidget *parent=0);
  
  const QColor& color() const { return color_; }

 Q_SIGNALS:
  // Emitted when the color is changed by user interaction.
  void colorEdited(const QColor &color);
  // Emitted when the color is changed by user interaction or programatically.
  void colorChanged(const QColor &color);

 public Q_SLOTS:
  void setColor(const QColor &color);

 private Q_SLOTS:
  void handleClicked();
};  // class ColorButton
}  // namespace mapviz
#endif  // MAPVIZ_COLOR_BUTTON_H_
