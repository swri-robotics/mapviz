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
#include <mapviz/color_button.h>

#include <QColorDialog>

namespace mapviz
{
ColorButton::ColorButton(QWidget *parent)
  :
  QPushButton(parent)
{
  setColor(Qt::black);
  QObject::connect(this, SIGNAL(clicked(bool)),
                   this, SLOT(handleClicked()));
}

void ColorButton::setColor(const QColor &color)
{
  if (!color.isValid() || color == color_) {
    return;
  }
  
  color_ = color;
  // This was a very strange bug.  On initialization, the constructor
  // would set the stylesheet to black, then the external owner would
  // call setColor to change the color to something else.  We would
  // properly set the internal color_ and stylesheet, but it would
  // continue to display black.  If the user changed the color, it
  // would change properly.  Calling setStylesheet() twice fixes the
  // behavior.
  setStyleSheet("background: " + color_.name());
  setStyleSheet("background: " + color_.name());
  Q_EMIT colorChanged(color_);
}

void ColorButton::handleClicked()
{
  // Note: We do not pass ourself as the parent or else the dialog
  // will inherit our color as the background!
  QColor new_color = QColorDialog::getColor(color_);
  if (!new_color.isValid() || new_color == color_) {
    return;
  }
  setColor(new_color);
  Q_EMIT colorEdited(new_color);
}
}  // namespace mapviz
