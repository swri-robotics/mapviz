#ifndef MULTIRES_IMAGE_PLUGIN_H
#define MULTIRES_IMAGE_PLUGIN_H

// C++ standard libraries
#include <string>

// Boost libraries
#include <boost/filesystem.hpp>
#define BOOST_FILESYSTEM_VERSION 2

// QT libraries
#include <QGLWidget>
#include <QObject>
#include <QWidget>

#include <mapviz/mapviz_plugin.h>
#include <mapviz/map_canvas.h>
#include <multires_image/tile_set.h>

#include "multires_view.h"

// QT autogenerated files
#include "ui_multires_config.h"

namespace mapviz_plugins
{

  class MultiresImagePlugin : public mapviz::MapvizPlugin
  {

    Q_OBJECT
  
  public:

    MultiresImagePlugin();
    virtual ~MultiresImagePlugin();

    bool Initialize(QGLWidget* canvas);
    void Shutdown() {}
    
    void Draw(double x, double y, double scale);

    void Transform();
    
    void LoadConfiguration(const YAML::Node& node, const std::string& config_path);
    void SaveConfiguration(YAML::Emitter& emitter, const std::string& config_path);

    QWidget* GetConfigWidget(QWidget* parent);

  protected:
    void PrintError(const std::string& message);
    void PrintInfo(const std::string& message);
    void PrintWarning(const std::string& message);

  protected Q_SLOTS:
    void SelectFile();
    void AcceptConfiguration();

  private:
    bool     loaded_;
    double center_x_;
    double center_y_;
    QGLWidget* canvas_;
    TileSet* tile_set_;
    MultiresView* tile_view_;

    Ui::multires_config ui_;
    QWidget* config_widget_;

    void InitializeTiles();
    void GetCenterPoint(double x, double y);
    boost::filesystem::path MakePathRelative(boost::filesystem::path path, boost::filesystem::path base);
  };
}

#endif /* MULTIRES_IMAGE_PLUGIN_H */
