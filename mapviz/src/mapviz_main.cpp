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

#include <QCoreApplication>
#include <QGuiApplication>

#include "mapviz/mapviz.hpp"
#include "mapviz/mapviz_application.hpp"

int main(int argc, char ** argv)
{
  // Initialize ROS; spinning on the Node is handled in mapviz.cpp
  rclcpp::init(argc, argv);

  // Initialize Qt resources
  Q_INIT_RESOURCE(icons);

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
  // Qt 6 always scales the interface to each monitor's scale factor; Qt 5 only
  // does if asked before the application object exists.  Without this, mapviz
  // on Qt 5 is drawn at 1x everywhere: tiny on a high-DPI monitor, and the
  // same size on every monitor.  Fractional factors (1.25, 1.5) are passed
  // through rather than rounded, as Qt 6 does.  QT_ENABLE_HIGHDPI_SCALING and
  // the other QT_* scaling variables still take precedence.
  QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
  QCoreApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);
#if QT_VERSION >= QT_VERSION_CHECK(5, 14, 0)
  QGuiApplication::setHighDpiScaleFactorRoundingPolicy(
    Qt::HighDpiScaleFactorRoundingPolicy::PassThrough);
#endif
#endif

  // Initialize QT
  mapviz::MapvizApplication app(argc, argv);

  // Start mapviz
  mapviz::Mapviz mapviz(true, argc, argv);
  app.setLogger(mapviz.GetNode()->get_logger());
  mapviz.show();

  return mapviz::MapvizApplication::exec();
}
