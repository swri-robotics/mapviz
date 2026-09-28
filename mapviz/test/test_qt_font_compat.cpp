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
#include <gtest/gtest.h>
#include <QFont>
#include <QGuiApplication>
#include <QString>

#include <mapviz/qt_font_compat.hpp>

using mapviz::FontFromString;

namespace
{
int Weight(const QFont & font) {return static_cast<int>(font.weight());}
}  // namespace

TEST(FontFromString, ReadsWhatThisQtVersionSaves)
{
  QFont saved("Sans Serif", 26);
  saved.setBold(true);
  saved.setItalic(true);

  QFont font;
  ASSERT_TRUE(FontFromString(saved.toString(), font));

  EXPECT_EQ("Sans Serif", font.family());
  EXPECT_EQ(26, font.pointSize());
  EXPECT_TRUE(font.bold());
  EXPECT_TRUE(font.italic());
}

TEST(FontFromString, ReadsAFontSavedByQt5)
{
  QFont font;
  ASSERT_TRUE(FontFromString("Sans Serif,26,-1,5,75,1,1,0,0,0", font));

  EXPECT_EQ("Sans Serif", font.family());
  EXPECT_EQ(26, font.pointSize());
  EXPECT_EQ(static_cast<int>(QFont::Bold), Weight(font));
  EXPECT_TRUE(font.italic());
  EXPECT_TRUE(font.underline());
}

TEST(FontFromString, ReadsAFontSavedByQt6)
{
  // Qt 5 rejects this format, so configs saved on Qt 6 used to lose their
  // fonts when opened on Qt 5.
  QFont font;
  ASSERT_TRUE(FontFromString("Sans Serif,26,-1,5,700,1,1,0,0,0,0,0,0,0,0,1", font));

  EXPECT_EQ("Sans Serif", font.family());
  EXPECT_EQ(26, font.pointSize());
  EXPECT_EQ(static_cast<int>(QFont::Bold), Weight(font));
  EXPECT_TRUE(font.italic());
  EXPECT_TRUE(font.underline());
  EXPECT_FALSE(font.strikeOut());
}

TEST(FontFromString, ReadsAFontSavedByQt6WithAStyleName)
{
  QFont font;
  ASSERT_TRUE(FontFromString("Sans Serif,12,-1,5,400,0,0,0,0,0,0,0,0,0,0,1,Regular", font));

  EXPECT_EQ("Sans Serif", font.family());
  EXPECT_EQ(12, font.pointSize());
  EXPECT_EQ(static_cast<int>(QFont::Normal), Weight(font));
}

TEST(FontFromString, KeepsTheFontWhenItCannotBeRead)
{
  QFont font("Monospace", 17);

  // A bare family name is a valid font, but a truncated description isn't.
  EXPECT_FALSE(FontFromString("Sans Serif,26,-1,5", font));
  EXPECT_FALSE(FontFromString("", font));

  EXPECT_EQ("Monospace", font.family());
  EXPECT_EQ(17, font.pointSize());
}

int main(int argc, char ** argv)
{
  // Fonts need a QGuiApplication.  The test runs headless; see the ENV set on
  // this target in CMakeLists.txt.
  QGuiApplication app(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
