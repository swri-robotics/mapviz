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

#include <QApplication>
#include <QColor>
#include <QImage>
#include <QLabel>
#include <QPalette>
#include <QString>

#include <cmath>
#include <string>

#include <mapviz/config_item.hpp>
#include <mapviz/mapviz_plugin.hpp>
#include <mapviz/widgets.hpp>
#include <mapviz/status_label.hpp>
#include <mapviz/theme.hpp>

using mapviz::StatusLevel;
using mapviz::Theme;
using mapviz::ThemeIcon;
using mapviz::ThemeMode;

namespace
{
/// The smallest concrete plugin, for exercising the status label helpers.
class StatusPlugin : public mapviz::MapvizPlugin
{
public:
  bool Initialize(QOpenGLWidget *) override {return true;}
  void Shutdown() override {}
  void PrintError(const std::string &) override {}
  void PrintInfo(const std::string &) override {}
  void PrintWarning(const std::string &) override {}

protected:
  void Draw(double, double, double) override {}
  void Transform() override {}
  void LoadConfig(const YAML::Node &, const std::string &) override {}
  void SaveConfig(YAML::Emitter &, const std::string &) override {}
};

double Luminance(const QColor & color)
{
  auto channel = [](double value) {
      return value <= 0.03928 ? value / 12.92 : std::pow((value + 0.055) / 1.055, 2.4);
    };
  return 0.2126 * channel(color.redF()) + 0.7152 * channel(color.greenF()) +
         0.0722 * channel(color.blueF());
}

/// WCAG contrast ratio; 4.5 is the threshold for body text.
double Contrast(const QColor & a, const QColor & b)
{
  const double la = Luminance(a);
  const double lb = Luminance(b);
  return (std::max(la, lb) + 0.05) / (std::min(la, lb) + 0.05);
}
}  // namespace

TEST(Theme, PalettesAreLightAndDark)
{
  const QPalette light = Theme::Palette(false);
  const QPalette dark = Theme::Palette(true);

  EXPECT_GT(light.color(QPalette::Window).lightness(), 200);
  EXPECT_LT(dark.color(QPalette::Window).lightness(), 60);
}

TEST(Theme, TextIsReadableInBothPalettes)
{
  for (bool dark : {false, true}) {
    const QPalette palette = Theme::Palette(dark);
    EXPECT_GE(
      Contrast(palette.color(QPalette::WindowText), palette.color(QPalette::Window)), 7.0);
    EXPECT_GE(Contrast(palette.color(QPalette::Text), palette.color(QPalette::Base)), 7.0);
    EXPECT_GE(
      Contrast(palette.color(QPalette::ButtonText), palette.color(QPalette::Button)), 7.0);
    EXPECT_GE(
      Contrast(
        palette.color(QPalette::HighlightedText), palette.color(QPalette::Highlight)), 3.0);
  }
}

TEST(Theme, StatusColorsAreReadableOnTheirBackground)
{
  for (bool dark : {false, true}) {
    const QPalette palette = Theme::Palette(dark);
    for (StatusLevel level : {StatusLevel::Info, StatusLevel::Warning, StatusLevel::Error}) {
      const QColor color = mapviz::StatusColor(level, dark);
      EXPECT_GE(Contrast(color, palette.color(QPalette::Window)), 4.5)
        << "level " << static_cast<int>(level) << (dark ? " dark" : " light");
    }
  }
}

TEST(Theme, StyleSheetResolvesEveryColor)
{
  for (bool dark : {false, true}) {
    const QString sheet = Theme::StyleSheet(Theme::Palette(dark));
    EXPECT_FALSE(sheet.isEmpty());
    // An '@' left behind would be a placeholder with no matching color.
    EXPECT_FALSE(sheet.contains('@')) << sheet.mid(sheet.indexOf('@'), 30).toStdString();
  }
}

TEST(StatusText, DropsOneTrailingPeriod)
{
  EXPECT_EQ("No messages received", mapviz::StatusText("No messages received."));
  EXPECT_EQ("No topic", mapviz::StatusText("No topic"));
  EXPECT_EQ("Sending route...", mapviz::StatusText("Sending route..."));
  EXPECT_EQ("Failed: a.b", mapviz::StatusText("Failed: a.b."));
  EXPECT_EQ("", mapviz::StatusText("."));
  EXPECT_EQ("", mapviz::StatusText(""));
}

TEST(StatusText, PluginStatusLabelsDropTheTrailingPeriod)
{
  Theme::Apply(ThemeMode::Light);
  StatusPlugin plugin;
  QLabel label;

  plugin.PrintWarningHelper(&label, "Waiting for transform.", 0.0);
  EXPECT_EQ(QString("Waiting for transform"), label.text());
  EXPECT_EQ(
    mapviz::StatusColor(StatusLevel::Warning, false), label.palette().color(QPalette::Text));

  plugin.PrintInfoHelper(&label, "Sending route...", 0.0);
  EXPECT_EQ(QString("Sending route..."), label.text());
}

TEST(Theme, ModeNamesRoundTrip)
{
  for (ThemeMode mode : {ThemeMode::System, ThemeMode::Light, ThemeMode::Dark}) {
    EXPECT_EQ(mode, Theme::ModeFromString(Theme::ModeToString(mode)));
  }
  EXPECT_EQ(ThemeMode::Dark, Theme::ModeFromString(" Dark "));
  EXPECT_EQ(ThemeMode::System, Theme::ModeFromString(""));
  EXPECT_EQ(ThemeMode::System, Theme::ModeFromString("solarized"));
}

TEST(Theme, EveryIconPaintsSomething)
{
  for (ThemeIcon icon : {
    ThemeIcon::Pin, ThemeIcon::Screenshot, ThemeIcon::Record, ThemeIcon::Pause,
    ThemeIcon::Stop, ThemeIcon::Recenter, ThemeIcon::Remove, ThemeIcon::ChevronRight,
    ThemeIcon::ChevronDown})
  {
    const QImage image = Theme::Icon(icon).pixmap(QSize(32, 32)).toImage();
    ASSERT_FALSE(image.isNull());

    bool painted = false;
    for (int y = 0; y < image.height() && !painted; y++) {
      for (int x = 0; x < image.width() && !painted; x++) {
        painted = qAlpha(image.pixel(x, y)) > 0;
      }
    }
    EXPECT_TRUE(painted) << "icon " << static_cast<int>(icon);
  }
}

TEST(Theme, ApplySwitchesThePaletteAndRecolorsStatusLabels)
{
  QLabel label;

  Theme::Apply(ThemeMode::Light);
  EXPECT_EQ(ThemeMode::Light, Theme::CurrentMode());
  EXPECT_EQ(
    Theme::Palette(false).color(QPalette::Window), QApplication::palette().color(QPalette::Window));
  mapviz::SetStatusLevel(&label, StatusLevel::Error);
  EXPECT_EQ(mapviz::StatusColor(StatusLevel::Error, false), label.palette().color(QPalette::Text));

  // The label was colored for a light background; switching themes has to
  // recolor it, or it would be left dim on the dark one.
  Theme::Apply(ThemeMode::Dark);
  EXPECT_EQ(
    Theme::Palette(true).color(QPalette::Window), QApplication::palette().color(QPalette::Window));
  EXPECT_EQ(mapviz::StatusColor(StatusLevel::Error, true), label.palette().color(QPalette::Text));
  EXPECT_EQ(
    mapviz::StatusColor(StatusLevel::Error, true), label.palette().color(QPalette::WindowText));
}

TEST(Theme, SizesFollowTheApplicationFont)
{
  const QFont original = QApplication::font();
  QFont small = original;
  small.setPointSizeF(9);
  QApplication::setFont(small);
  const int small_icon = mapviz::IconWidget::IconSize();
  const QString small_sheet = Theme::StyleSheet(Theme::Palette(false));

  mapviz::ConfigItem item;
  item.RefreshAppearance();
  const int small_chevron = item.ui_.signlabel->width();

  QFont large = original;
  large.setPointSizeF(18);
  QApplication::setFont(large);
  const int large_icon = mapviz::IconWidget::IconSize();
  item.RefreshAppearance();

  EXPECT_GT(large_icon, small_icon);
  EXPECT_GT(item.ui_.signlabel->width(), small_chevron);
  EXPECT_NE(small_sheet, Theme::StyleSheet(Theme::Palette(false)));

  // Header icons are drawn on a 16-unit canvas sized for the current font.
  const QPixmap icon = mapviz::IconWidget::CreatePixmap();
  EXPECT_NEAR(16.0, icon.width() / icon.devicePixelRatio(), 0.5);
  EXPECT_NEAR(large_icon * qApp->devicePixelRatio(), icon.width(), 1.0);

  QApplication::setFont(original);
}

int main(int argc, char ** argv)
{
  // Widgets need a QApplication.  The test runs headless; see the ENV set on
  // this test in CMakeLists.txt.
  QApplication app(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
